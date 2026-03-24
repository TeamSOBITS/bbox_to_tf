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
: LifecycleNode("mask_to_3d", options)
{
  // Declare parameters in the constructor
  this->declare_parameter("base_frame_name", "base_footprint");
  this->declare_parameter("mask_topic_name", "/masks_array");
  this->declare_parameter("cloud_topic_name", "/point_cloud");
  this->declare_parameter("info_topic_name", "/camera_info");

  this->declare_parameter("x_min", -10.0);
  this->declare_parameter("x_max", 10.0);
  this->declare_parameter("y_min", -10.0);
  this->declare_parameter("y_max", 10.0);
  this->declare_parameter("z_min", -10.0);
  this->declare_parameter("z_max", 10.0);

  this->declare_parameter("cluster_tolerance", 0.05);
  this->declare_parameter("min_cluster_size", 50);
  this->declare_parameter("max_cluster_size", 20000);
  this->declare_parameter("noise_point_cloud_range", 0.01);
  this->declare_parameter("voxel_leaf_size", 0.01);

  // this->declare_parameter("enable_id", false);
}

CallbackReturn MaskTo3D::on_configure(const rclcpp_lifecycle::State &)
{
  base_frame_name_ = this->get_parameter("base_frame_name").as_string();
  mask_topic_name_ = this->get_parameter("mask_topic_name").as_string();
  cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
  info_topic_name_ = this->get_parameter("info_topic_name").as_string();

  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  z_min_ = this->get_parameter("z_min").as_double();
  z_max_ = this->get_parameter("z_max").as_double();

  cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
  max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
  noise_point_cloud_range_ = this->get_parameter("noise_point_cloud_range").as_double();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

  // enable_id_ = this->get_parameter("enable_id").as_bool();

  RCLCPP_INFO(this->get_logger(), "Configuring MaskTo3D Node...");

  RCLCPP_INFO(this->get_logger(), "Base Frame Name: %s", base_frame_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Mask Topic Name: %s", mask_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Cloud Topic Name: %s", cloud_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Info Topic Name: %s", info_topic_name_.c_str());

  RCLCPP_INFO(this->get_logger(), "Clipping Bounds:");
  RCLCPP_INFO(this->get_logger(), "  x: [%f, %f]", x_min_, x_max_);
  RCLCPP_INFO(this->get_logger(), "  y: [%f, %f]", y_min_, y_max_);
  RCLCPP_INFO(this->get_logger(), "  z: [%f, %f]", z_min_, z_max_);

  RCLCPP_INFO(this->get_logger(), "Cluster Tolerance: %f", cluster_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Min Cluster Size: %d", min_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "Max Cluster Size: %d", max_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "Noise Point Cloud Range: %f", noise_point_cloud_range_);
  RCLCPP_INFO(this->get_logger(), "Voxel Leaf Size: %f", voxel_leaf_size_);

  // RCLCPP_INFO(this->get_logger(), "Enable ID: %s", enable_id_ ? "true" : "false");

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  kdtree_.reset(new pcl::search::KdTree<PointT>);
  euclid_clustering_.setClusterTolerance(cluster_tolerance_);
  euclid_clustering_.setMinClusterSize(min_cluster_size_);
  euclid_clustering_.setMaxClusterSize(max_cluster_size_);
  euclid_clustering_.setSearchMethod(kdtree_);

  pub_obj_poses_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
    this->get_name() + std::string("/object_3d_poses"), 5);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_name() + std::string("/object_3d_cloud"), 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MaskTo3D::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating MaskTo3D Node...");
  
  pub_obj_poses_->on_activate();
  pub_debug_cloud_->on_activate();

  rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_masks_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::DetectMaskArray, rclcpp_lifecycle::LifecycleNode>>(this, mask_topic_name_, sensor_qos);
  sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(this, cloud_topic_name_, sensor_qos);
  sub_info_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>>(this, info_topic_name_, sensor_qos);

  sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<MaskCloudSyncPolicy>>(
    MaskCloudSyncPolicy(200), *sub_masks_, *sub_pcl_, *sub_info_);
  sync_point_cloud_->registerCallback(&MaskTo3D::callback_MaskPointCloud, this);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MaskTo3D::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating MaskTo3D Node...");

  pub_obj_poses_->on_deactivate();
  pub_debug_cloud_->on_deactivate();

  sync_point_cloud_.reset();
  sub_masks_.reset();
  sub_pcl_.reset();
  sub_info_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MaskTo3D::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up MaskTo3D Node...");
  pub_obj_poses_.reset();
  pub_debug_cloud_.reset();
  tfBuffer_.reset();
  tfListener_.reset();
  tfBroadcaster_.reset();
  kdtree_.reset();
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn MaskTo3D::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down MaskTo3D Node...");
  sync_point_cloud_.reset();
  sub_masks_.reset();
  sub_pcl_.reset();
  sub_info_.reset();
  pub_obj_poses_.reset();
  pub_debug_cloud_.reset();
  
  return CallbackReturn::SUCCESS;
}

bool MaskTo3D::isRealisticPoint(const pcl::PointXYZ& pt) const {
  return pcl::isFinite(pt) && 
         pt.x >= x_min_ && pt.x <= x_max_ && 
         pt.y >= y_min_ && pt.y <= y_max_ && 
         pt.z >= z_min_ && pt.z <= z_max_;
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
        const auto& pt = cloud_src_optical->points[index];
        if (isRealisticPoint(pt)) {
          mask_cloud_optical->points.push_back(pt);
        }
      }
    }

    if (mask_cloud_optical->points.empty()) continue;

    mask_cloud_optical->width = mask_cloud_optical->points.size();
    mask_cloud_optical->height = 1;
    mask_cloud_optical->is_dense = true;
  
    // Transform the extracted mask points to the base footprint
    pcl_ros::transformPointCloud(*mask_cloud_optical, *mask_cloud_base, transformStamped);
    mask_cloud_base->header.frame_id = base_frame_name_;
    mask_cloud_base->width = mask_cloud_base->points.size();
    mask_cloud_base->height = 1;
    mask_cloud_base->is_dense = true;

    // Process Clustering
    vision_msgs::msg::Detection3D object_pose = processMaskClustering(mask, info_msg, mask_cloud_base);

    if (!object_pose.results.empty()) {
      object_pose_array.detections.push_back(object_pose);
      *combined_cloud += *mask_cloud_base;
    }
  }

  // Publish the combined point cloud of all detected objects for debugging
  if (pub_debug_cloud_->get_subscription_count() > 0) {
    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 combined_cloud_msg;
    pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
    combined_cloud_msg.header = info_msg->header;
    combined_cloud_msg.header.frame_id = base_frame_name_;
    pub_debug_cloud_->publish(combined_cloud_msg);
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

  mask_cloud->width = mask_cloud->points.size();
  mask_cloud->height = 1;
  mask_cloud->is_dense = true;

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
  main_object_cloud->height = 1;
  main_object_cloud->is_dense = true;
  mask_cloud = main_object_cloud;

  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid(*mask_cloud, xyz_centroid);
  
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*mask_cloud, min_pt, max_pt);

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position.x = xyz_centroid.x();
  obj_pose.position.y = xyz_centroid.y();
  obj_pose.position.z = xyz_centroid.z();
  obj_pose.orientation.w = 1.0;

  if (!mask.results.empty()) {
      object_pose.results.push_back(mask.results[0]);
  }
  
  object_pose.bbox.center = obj_pose;
  object_pose.bbox.size.x = max_pt.x() - min_pt.x();
  object_pose.bbox.size.y = max_pt.y() - min_pt.y();
  object_pose.bbox.size.z = max_pt.z() - min_pt.z();
  object_pose.id = mask.results.empty() ? "" : mask.results[0].hypothesis.class_id + "_" + mask.instance_id;
  
  publishObjectTf(obj_pose, object_pose.id);

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
