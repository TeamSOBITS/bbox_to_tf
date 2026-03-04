#include "image_to_position/mask_to_3d.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/common.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace image_to_position
{

MaskTo3D::MaskTo3D(const rclcpp::NodeOptions & options)
: Node("mask_to_3d", options)
{
  base_frame_name_ = this->declare_parameter("base_frame_name", "base_footprint");
  std::string mask_topic = this->declare_parameter("mask_topic_name", "/masks_array");
  std::string cloud_topic = this->declare_parameter("cloud_topic_name", "/point_cloud");
  std::string info_topic = this->declare_parameter("info_topic_name", "/camera_info");

  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.05);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 50);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 20000);
  noise_point_cloud_range_ = this->declare_parameter("noise_point_cloud_range", 0.01);
  bool execute_default = this->declare_parameter("execute_default", true);

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
      sub_masks_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::DetectMaskArray>>(this, this->get_parameter("mask_topic_name").as_string());
      sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, this->get_parameter("cloud_topic_name").as_string(), sensor_qos);
      sub_info_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, this->get_parameter("info_topic_name").as_string());

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
  PointCloud::Ptr cloud_src(new PointCloud());
  pcl::fromROSMsg(*pcl_msg, *cloud_src);
  if (!tfBuffer_->canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp)) return;
  if (!pcl_ros::transformPointCloud(base_frame_name_, *cloud_src, *cloud_src, *tfBuffer_)) return;

  vision_msgs::msg::Detection3DArray object_pose_array;
  object_pose_array.header = mask_msg->header;
  object_pose_array.header.frame_id = base_frame_name_;

  PointCloud::Ptr combined_cloud(new PointCloud());
  combined_cloud->header.frame_id = base_frame_name_;

  for (size_t i = 0; i < mask_msg->masks.size(); i++) {
    PointCloud::Ptr mask_cloud(new PointCloud());
    mask_cloud->header.frame_id = base_frame_name_;

    vision_msgs::msg::Detection3D object_pose = processMaskClustering(mask_msg->masks[i], info_msg, cloud_src, mask_cloud);

    if (!object_pose.results.empty()) {
      object_pose_array.detections.push_back(object_pose);
      *combined_cloud += *mask_cloud;
    }
  }

  sensor_msgs::msg::PointCloud2 combined_cloud_msg;
  pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
  combined_cloud_msg.header = info_msg->header;
  combined_cloud_msg.header.frame_id = base_frame_name_;
  
  pub_object_cloud_->publish(combined_cloud_msg);
  pub_obj_poses_->publish(object_pose_array);
}

vision_msgs::msg::Detection3D MaskTo3D::processMaskClustering(
  const sobits_interfaces::msg::DetectMask& mask,
  const std::shared_ptr<sensor_msgs::msg::CameraInfo>& info_msg,
  const PointCloud::Ptr& point_cloud, 
  PointCloud::Ptr& mask_cloud)
{
  vision_msgs::msg::Detection3D object_pose;
  object_pose.header = info_msg->header;
  object_pose.header.frame_id = base_frame_name_;

  // Extract points belonging to the mask
  for (size_t i = 0; i < mask.pixel_x.size(); ++i) {
    int index = mask.pixel_y[i] * info_msg->width + mask.pixel_x[i];
    if (index >= 0 && index < static_cast<int>(point_cloud->points.size())) {
      if (checkNanInf(point_cloud->points[index])) {
        mask_cloud->points.push_back(point_cloud->points[index]);
      }
    }
  }

  if (mask_cloud->points.empty()) return object_pose;

  // Cluster to remove background noise (edges of mask bleeding onto background)
  kdtree_->setInputCloud(mask_cloud);
  euclid_clustering_.setInputCloud(mask_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  euclid_clustering_.extract(cluster_indices);

  if (cluster_indices.empty()) return object_pose;

  // Isolate the largest cluster as the main object
  PointCloud::Ptr main_object_cloud(new PointCloud());
  for (const auto& idx : cluster_indices[0].indices) {
    main_object_cloud->points.push_back(mask_cloud->points[idx]);
  }
  mask_cloud = main_object_cloud; // Update the reference cloud

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
  object_pose.id = mask.id;
  
  publishObjectTf(obj_pose, mask.id);

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

bool MaskTo3D::checkNanInf(const PointT& pt) const {
  return !((std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) || 
           (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)));
}

}  // namespace image_to_position

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_to_position::MaskTo3D)
