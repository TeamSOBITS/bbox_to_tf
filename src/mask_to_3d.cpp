#include "image_to_position/mask_to_3d.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace image_to_position
{

MaskTo3D::MaskTo3D(const rclcpp::NodeOptions & options)
: Node("mask_to_3d", options)
{
  // Declare parameters
  base_frame_name_ = this->declare_parameter("base_frame_name", "base_footprint");
  mask_topic_ = this->declare_parameter("mask_topic_name", "/masks_array");
  cloud_topic_ = this->declare_parameter("cloud_topic_name", "/point_cloud");
  info_topic_ = this->declare_parameter("info_topic_name", "/camera_info");

  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.05);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 50);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 20000);
  noise_point_cloud_range_ = this->declare_parameter("noise_point_cloud_range", 0.01);
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.01);

  debug_ = this->declare_parameter("debug", false);
  bool execute_default = this->declare_parameter("execute_default", true);

  // Param info logging
  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  base_frame_name: %s", base_frame_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  mask topic: %s", mask_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  cloud topic: %s", cloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  info topic: %s", info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  cluster_tolerance: %f", cluster_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  min_cluster_size: %d", min_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "  max_cluster_size: %d", max_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "  noise_point_cloud_range: %f", noise_point_cloud_range_);
  RCLCPP_INFO(this->get_logger(), "  voxel_leaf_size: %f", voxel_leaf_size_);
  RCLCPP_INFO(this->get_logger(), "  debug: %s", debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  execute_default: %s", execute_default ? "true" : "false");

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  kdtree_.reset(new pcl::search::KdTree<PointT>);
  euclid_clustering_.setClusterTolerance(cluster_tolerance_);
  euclid_clustering_.setMinClusterSize(min_cluster_size_);
  euclid_clustering_.setMaxClusterSize(max_cluster_size_);
  euclid_clustering_.setSearchMethod(kdtree_);

  pub_obj_poses_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_3d_poses", 5);
  pub_object_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_3d_cloud", 1);

  run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "position/run_ctr", std::bind(&MaskTo3D::callback_runctr, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize subscriptions if default is true
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = execute_default;
  callback_runctr(request, response);
}

void MaskTo3D::callback_runctr(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) {
    if (!sub_masks_) {
      rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
      sub_masks_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::DetectMaskArray>>(this, mask_topic_);
      sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_, sensor_qos);
      sub_info_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, info_topic_);

      sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<MaskCloudSyncPolicy>>(
        MaskCloudSyncPolicy(200), *sub_masks_, *sub_pcl_, *sub_info_);
      sync_point_cloud_->registerCallback(&MaskTo3D::callback_MaskPointCloud, this);
    }
  } else {
    sync_point_cloud_.reset();
    sub_masks_.reset();
    sub_pcl_.reset();
    sub_info_.reset();
  }
  res->success = true;
}

void MaskTo3D::callback_MaskPointCloud(
  const std::shared_ptr<sobits_interfaces::msg::DetectMaskArray> mask_msg,
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_msg,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg)
{
  PointCloud::Ptr cloud_src_optical(new PointCloud());
  pcl::fromROSMsg(*pcl_msg, *cloud_src_optical);

  // Lookup TF transform once per callback
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform(
      base_frame_name_, pcl_msg->header.frame_id, 
      pcl_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    return;
  }

  vision_msgs::msg::Detection3DArray object_pose_array;
  object_pose_array.header = mask_msg->header;
  object_pose_array.header.frame_id = base_frame_name_;

  PointCloud::Ptr combined_cloud(new PointCloud());
  combined_cloud->header.frame_id = base_frame_name_;

  // Pre-allocate pointers to avoid heap allocations inside the loop
  PointCloud::Ptr mask_cloud_optical(new PointCloud());
  PointCloud::Ptr mask_cloud_base(new PointCloud());

  for (size_t i = 0; i < mask_msg->masks.size(); i++) {
    mask_cloud_optical->clear();
    mask_cloud_base->clear();
    mask_cloud_optical->header.frame_id = pcl_msg->header.frame_id;

    const auto& mask = mask_msg->masks[i];

    // Extract points belonging to the mask in the optical frame
    for (size_t j = 0; j < mask.pixel_x.size(); ++j) {
      int index = mask.pixel_y[j] * info_msg->width + mask.pixel_x[j];
      if (index >= 0 && index < static_cast<int>(cloud_src_optical->points.size())) {
        if (pcl::isFinite(cloud_src_optical->points[index])) {
          mask_cloud_optical->points.push_back(cloud_src_optical->points[index]);
        }
      }
    }

    if (mask_cloud_optical->points.empty()) continue;

    // Transform the extracted mask points to the base footprint
    pcl_ros::transformPointCloud(*mask_cloud_optical, *mask_cloud_base, transformStamped);
    mask_cloud_base->header.frame_id = base_frame_name_;

    // Process Clustering
    vision_msgs::msg::Detection3D object_pose = processMaskClustering(mask, info_msg, mask_cloud_base);

    if (!object_pose.results.empty()) {
      object_pose_array.detections.push_back(object_pose);
      *combined_cloud += *mask_cloud_base;
    }
  }

  if (debug_) {
    sensor_msgs::msg::PointCloud2 combined_cloud_msg;
    pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
    combined_cloud_msg.header = info_msg->header;
    combined_cloud_msg.header.frame_id = base_frame_name_;
    
    pub_object_cloud_->publish(combined_cloud_msg);
  }

  pub_obj_poses_->publish(object_pose_array);
}

vision_msgs::msg::Detection3D MaskTo3D::processMaskClustering(
  const sobits_interfaces::msg::DetectMask& mask,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo>& info_msg,
  PointCloud::Ptr& mask_cloud)
{
  vision_msgs::msg::Detection3D object_pose;
  object_pose.header = info_msg->header;
  object_pose.header.frame_id = base_frame_name_;

  if (mask_cloud->points.empty()) return object_pose;

  // VoxelGrid filter downsamples density
  PointCloud::Ptr cloud_filtered(new PointCloud());
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(mask_cloud);
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  vg.filter(*cloud_filtered);

  if (cloud_filtered->points.empty()) return object_pose;

  // Cluster to remove background noise (edges of mask bleeding onto background)
  kdtree_->setInputCloud(cloud_filtered);
  euclid_clustering_.setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  euclid_clustering_.extract(cluster_indices);

  if (cluster_indices.empty()) return object_pose;

  // Isolate the largest cluster as the main object
  PointCloud::Ptr main_object_cloud(new PointCloud());
  for (const auto& idx : cluster_indices[0].indices) {
    main_object_cloud->points.push_back(cloud_filtered->points[idx]);
  }
  mask_cloud = main_object_cloud;

  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid(*mask_cloud, xyz_centroid);
  
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*mask_cloud, min_pt, max_pt);

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = xyz_centroid.x();
  obj_pose.position.y = xyz_centroid.y();
  obj_pose.position.z = xyz_centroid.z();
  obj_pose.orientation.w = 1.0; // Default flat orientation

  // Transfer label/score information
  if (!mask.results.empty()) {
      object_pose.results.push_back(mask.results[0]);
  }
  
  object_pose.bbox.center = obj_pose;
  object_pose.bbox.size.x = max_pt.x() - min_pt.x();
  object_pose.bbox.size.y = max_pt.y() - min_pt.y();
  object_pose.bbox.size.z = max_pt.z() - min_pt.z();
  object_pose.id = mask.results[0].hypothesis.class_id;
  
  publishObjectTf(obj_pose, mask.results[0].hypothesis.class_id);

  return object_pose;
}

void MaskTo3D::publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = base_frame_name_;
  t.child_frame_id = object_id.empty() ? "mask_object" : object_id;
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation = pose.orientation;
  tfBroadcaster_->sendTransform(t);
}

}  // namespace image_to_position

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_to_position::MaskTo3D)
