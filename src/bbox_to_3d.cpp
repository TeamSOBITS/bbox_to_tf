#include "image_to_position/bbox_to_3d.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/crop_box.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace image_to_position
{

BboxTo3D::BboxTo3D(const rclcpp::NodeOptions & options)
: LifecycleNode("bbox_to_3d", options)
{
  this->declare_parameter("base_frame_name", "base_footprint");
  this->declare_parameter("bbox_topic_name", "objects_rect");
  this->declare_parameter("cloud_topic_name", "dummy_pointcloud");
  this->declare_parameter("depth_image_topic_name", "dummy_image");
  this->declare_parameter("info_topic_name", "dummy_info");

  this->declare_parameter("x_min", -10.0);
  this->declare_parameter("x_max", 10.0);
  this->declare_parameter("y_min", -10.0);
  this->declare_parameter("y_max", 10.0);
  this->declare_parameter("z_min", -10.0);
  this->declare_parameter("z_max", 10.0);

  this->declare_parameter("cluster_tolerance", 0.01);
  this->declare_parameter("min_cluster_size", 100);
  this->declare_parameter("max_cluster_size", 20000);
  this->declare_parameter("noise_point_cloud_range", 0.01);
  this->declare_parameter("positioning_detection_mode", "point_cloud");
  this->declare_parameter("voxel_leaf_size", 0.01);

  this->declare_parameter("enable_id", false);
}

CallbackReturn BboxTo3D::on_configure(const rclcpp_lifecycle::State &)
{
  base_frame_name_ = this->get_parameter("base_frame_name").as_string();
  bbox_topic_name_ = this->get_parameter("bbox_topic_name").as_string();
  cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
  depth_topic_name_ = this->get_parameter("depth_image_topic_name").as_string();
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
  positioning_detection_mode_ = this->get_parameter("positioning_detection_mode").as_string();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

  enable_id_ = this->get_parameter("enable_id").as_bool();

  RCLCPP_INFO(this->get_logger(), "Configuring BboxTo3D Node...");

  RCLCPP_INFO(this->get_logger(), "Base Frame Name: %s", base_frame_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "BBox Topic Name: %s", bbox_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Cloud Topic Name: %s", cloud_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Depth Image Topic Name: %s", depth_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera Info Topic Name: %s", info_topic_name_.c_str());

  RCLCPP_INFO(this->get_logger(), "Clipping Bounds:");
  RCLCPP_INFO(this->get_logger(), "  x: [%f, %f]", x_min_, x_max_);
  RCLCPP_INFO(this->get_logger(), "  y: [%f, %f]", y_min_, y_max_);
  RCLCPP_INFO(this->get_logger(), "  z: [%f, %f]", z_min_, z_max_);

  RCLCPP_INFO(this->get_logger(), "Cluster Tolerance: %f", cluster_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Min Cluster Size: %d", min_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "Max Cluster Size: %d", max_cluster_size_);
  RCLCPP_INFO(this->get_logger(), "Noise Point Cloud Range: %f", noise_point_cloud_range_);
  RCLCPP_INFO(this->get_logger(), "Positioning Detection Mode: %s", positioning_detection_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Voxel Leaf Size: %f", voxel_leaf_size_);

  RCLCPP_INFO(this->get_logger(), "Enable ID: %s", enable_id_ ? "true" : "false");


  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  kdtree_.reset(new pcl::search::KdTree<PointT>);
  euclid_clustering_.setClusterTolerance(cluster_tolerance_);
  euclid_clustering_.setMinClusterSize(min_cluster_size_);
  euclid_clustering_.setMaxClusterSize(max_cluster_size_);
  euclid_clustering_.setSearchMethod(kdtree_);

  pub_obj_poses_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_3d_poses", 5);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_3d_cloud", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn BboxTo3D::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating BboxTo3D Node...");

  pub_obj_poses_->on_activate();
  pub_debug_cloud_->on_activate();

  rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_bboxes_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray, rclcpp_lifecycle::LifecycleNode>>(this, bbox_topic_name_);
  sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(this, cloud_topic_name_, sensor_qos);
  sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(this, depth_topic_name_, sensor_qos);
  sub_info_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>>(this, info_topic_name_, sensor_qos);

  if (positioning_detection_mode_ == "point_cloud" || positioning_detection_mode_ == "fast_point") {
    sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(
      BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_pcl_, *sub_info_);
    sync_point_cloud_->registerCallback(&BboxTo3D::callback_BBoxPointCloud, this);
  } else if (positioning_detection_mode_ == "depth_image") {
    sync_depth_image_ = std::make_shared<message_filters::Synchronizer<BBoxesDepthSyncPolicy>>(
      BBoxesDepthSyncPolicy(200), *sub_bboxes_, *sub_img_, *sub_info_);
    sync_depth_image_->registerCallback(&BboxTo3D::callback_BBoxDepthImage, this);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid positioning_detection_mode: %s", positioning_detection_mode_.c_str());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn BboxTo3D::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating BboxTo3D Node...");

  pub_obj_poses_->on_deactivate();
  pub_debug_cloud_->on_deactivate();

  sync_point_cloud_.reset();
  sync_depth_image_.reset();
  sub_bboxes_.reset();
  sub_pcl_.reset();
  sub_img_.reset();
  sub_info_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn BboxTo3D::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up BboxTo3D Node...");
  pub_obj_poses_.reset();
  pub_debug_cloud_.reset();
  tfBuffer_.reset();
  tfListener_.reset();
  tfBroadcaster_.reset();
  kdtree_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn BboxTo3D::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down BboxTo3D Node...");
  sync_point_cloud_.reset();
  sync_depth_image_.reset();
  sub_bboxes_.reset();
  sub_pcl_.reset();
  sub_img_.reset();
  sub_info_.reset();
  pub_obj_poses_.reset();
  pub_debug_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

bool BboxTo3D::isRealisticPoint(const pcl::PointXYZ& pt) const {
  return pcl::isFinite(pt) && 
         pt.x >= x_min_ && pt.x <= x_max_ && 
         pt.y >= y_min_ && pt.y <= y_max_ && 
         pt.z >= z_min_ && pt.z <= z_max_;
}

std::string BboxTo3D::generateObjectId(const std::string& base_id, size_t index) const {
  return enable_id_ ? (base_id + "_" + std::to_string(index)) : base_id;
}

geometry_msgs::msg::Quaternion BboxTo3D::get_quat_from_euler(const geometry_msgs::msg::Point& rpy) {
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(rpy.x, rpy.y, rpy.z);
  return tf2::toMsg(tf_quat);
}

void BboxTo3D::publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = base_frame_name_;
  t.child_frame_id = object_id;
  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;
  t.transform.rotation = pose.orientation;
  tfBroadcaster_->sendTransform(t);
}

vision_msgs::msg::Detection3D BboxTo3D::processBBoxClustering(
      const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
      const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
      const PointCloud::Ptr& cloud_src_optical, 
      const geometry_msgs::msg::TransformStamped& transform,
      PointCloud::Ptr& point_cloud_bbox_base) {

  vision_msgs::msg::Detection3D object_pose;
  object_pose.header = info_msg->header;
  object_pose.header.frame_id = base_frame_name_;

  int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
  int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
  int center_index = info_msg->width * center_y + center_x;
  
  PointCloud::Ptr point_cloud_bbox_optical(new PointCloud());
  point_cloud_bbox_optical->header.frame_id = cloud_src_optical->header.frame_id;

  if ((0 <= center_index) && (center_index < static_cast<int>(cloud_src_optical->points.size()))) {
    const auto& pt = cloud_src_optical->points[center_index];
    if (isRealisticPoint(pt)) {
      point_cloud_bbox_optical->points.push_back(pt);
    } else return object_pose;
  } else return object_pose;

  int width  = static_cast<int>(bbox_msg->bbox.size_x);
  int height = static_cast<int>(bbox_msg->bbox.size_y);
  double theta = bbox_msg->bbox.center.theta;
  
  for (int iy = 0; iy <= height / 2; iy++) {
    for (int ix = 0; ix <= width / 2; ix++) {
      if (ix == 0 && iy == 0) continue;
      
      int dx = ix * std::cos(theta) - iy * std::sin(theta);
      int dy = iy * std::cos(theta) + ix * std::sin(theta);

      // Symmetrically check all 4 quadrants
      std::vector<std::pair<int, int>> offsets = {
        {-dx, -dy}, {-dx, dy}, {dx, -dy}, {dx, dy}
      };

      for(const auto& off : offsets) {
        // Skip duplicate additions for zero-axis mirrors
        if (ix == 0 && off.first > 0) continue;
        if (iy == 0 && off.second > 0) continue;

        int w = center_x + off.first;
        int h = center_y + off.second;
        int pt_index = info_msg->width * h + w;

        if (pt_index >= 0 && pt_index < static_cast<int>(cloud_src_optical->points.size())) {
          const auto& pt = cloud_src_optical->points[pt_index];
          if (isRealisticPoint(pt)) {
            point_cloud_bbox_optical->points.push_back(pt);
          }
        }
      }
    }
  }

  if (point_cloud_bbox_optical->points.empty()) return object_pose;

  point_cloud_bbox_optical->width = point_cloud_bbox_optical->points.size();
  point_cloud_bbox_optical->height = 1;
  point_cloud_bbox_optical->is_dense = true;

  // Transform only the extracted optical cloud to base footprint
  pcl_ros::transformPointCloud(*point_cloud_bbox_optical, *point_cloud_bbox_base, transform);
  point_cloud_bbox_base->header.frame_id = base_frame_name_;

  // Store the transformed center point to use as a distance reference
  PointT reference_center = point_cloud_bbox_base->points[0];

  // VoxelGrid filter downsamples density
  PointCloud::Ptr cloud_filtered(new PointCloud());
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(point_cloud_bbox_base);
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  vg.filter(*cloud_filtered);

  if (cloud_filtered->points.empty()) return object_pose;

  // Cluster to remove background noise
  kdtree_->setInputCloud(cloud_filtered);
  euclid_clustering_.setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  euclid_clustering_.extract(cluster_indices);

  if (cluster_indices.empty()) return object_pose;

  Eigen::Vector4f min_pt, max_pt;
  double distance = std::numeric_limits<double>::max();
  PointCloud::Ptr best_cluster(new PointCloud());

  for (const auto& indices : cluster_indices) {
    Eigen::Vector4f tmp_min_pt, tmp_max_pt;
    pcl::getMinMax3D(*cloud_filtered, indices, tmp_min_pt, tmp_max_pt);
    
    // Distance from the original optical center
    double tmp_dis = std::sqrt(
      std::pow(((tmp_min_pt.x() + tmp_max_pt.x()) / 2.) - reference_center.x, 2) + 
      std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - reference_center.y, 2) + 
      std::pow(((tmp_min_pt.z() + tmp_max_pt.z()) / 2.) - reference_center.z, 2));

    if (distance > tmp_dis) {
      distance = tmp_dis;
      max_pt = tmp_max_pt;
      min_pt = tmp_min_pt;
      
      best_cluster->clear();
      for (int idx : indices.indices) {
        best_cluster->points.push_back(cloud_filtered->points[idx]);
      }
    }
  }

  best_cluster->width = best_cluster->points.size();
  best_cluster->height = 1;
  best_cluster->is_dense = true;

  *point_cloud_bbox_base = *best_cluster;

  pcl::CropBox<PointT> cropBox;
  cropBox.setMin(Eigen::Vector4f(min_pt.x()                           , min_pt.y() + noise_point_cloud_range_, min_pt.z() + noise_point_cloud_range_, 1.0));
  cropBox.setMax(Eigen::Vector4f(max_pt.x() - noise_point_cloud_range_, max_pt.y() - noise_point_cloud_range_, max_pt.z()                           , 1.0));
  cropBox.setInputCloud(point_cloud_bbox_base);
  cropBox.filter(*point_cloud_bbox_base);

  Eigen::Vector4f xyz_centroid;
  pcl::compute3DCentroid(*point_cloud_bbox_base, xyz_centroid);
  pcl::getMinMax3D(*point_cloud_bbox_base, min_pt, max_pt);

  geometry_msgs::msg::Point object_point, object_rotate;
  object_point.x = xyz_centroid.x();
  object_point.y = xyz_centroid.y();
  object_point.z = xyz_centroid.z();
  
  object_rotate.x = 0.;
  object_rotate.y = 0.;
  object_rotate.z = 0.;

  if ((xyz_centroid.x() - min_pt.x()) > (xyz_centroid.z() - min_pt.z() + noise_point_cloud_range_/4.)) {
    object_rotate.y =  M_PI/2.;
  } else if ((xyz_centroid.y() - min_pt.y()) > (xyz_centroid.z() - min_pt.z() + noise_point_cloud_range_/4.)) {
    object_rotate.x = -M_PI/2.;
    object_rotate.y =  M_PI/2.;
  }

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position = object_point;
  obj_pose.orientation = get_quat_from_euler(object_rotate);

  vision_msgs::msg::ObjectHypothesisWithPose ohwp;
  ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
  ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
  ohwp.pose.pose = obj_pose;
  ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
  
  object_pose.results.push_back(ohwp);
  object_pose.bbox.center = obj_pose;
  object_pose.bbox.size.x = max_pt.x() - min_pt.x();
  object_pose.bbox.size.y = max_pt.y() - min_pt.y();
  object_pose.bbox.size.z = max_pt.z() - min_pt.z();
  object_pose.id = bbox_msg->id;
  
  publishObjectTf(obj_pose, bbox_msg->id);

  return object_pose;
}

vision_msgs::msg::Detection3D BboxTo3D::processBBoxFastShot(
      const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
      const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
      const PointCloud::Ptr& cloud_src_optical, 
      const geometry_msgs::msg::TransformStamped& transform,
      PointCloud::Ptr& point_cloud_bbox_base) {

  vision_msgs::msg::Detection3D object_pose;
  object_pose.header = info_msg->header;
  object_pose.header.frame_id = base_frame_name_;

  int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
  int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
  int center_index = info_msg->width * center_y + center_x;
  
  geometry_msgs::msg::Point object_point, object_rotate;
  bool set_tf = false;

  PointCloud::Ptr single_pt_optical(new PointCloud());
  single_pt_optical->header.frame_id = cloud_src_optical->header.frame_id;

  if ((0 <= center_index) && (center_index < static_cast<int>(cloud_src_optical->points.size()))) {
    const auto& pt = cloud_src_optical->points[center_index];
    if (isRealisticPoint(pt)) {
      single_pt_optical->points.push_back(pt);
      set_tf = true;
    }
  }

  int width = static_cast<int>(bbox_msg->bbox.size_x);
  int height = static_cast<int>(bbox_msg->bbox.size_y);
  
  for (int row = 0; row < 3 && !set_tf; row++) {
    for (int col = 0; col < 3 && !set_tf; col++) {
      int sub_center_x = center_x - width / 2 + (col * width / 3) + width / 6;
      int sub_center_y = center_y - height / 2 + (row * height / 3) + height / 6;
      int sub_index = info_msg->width * sub_center_y + sub_center_x;

      if (sub_index >= 0 && sub_index < static_cast<int>(cloud_src_optical->points.size())) {
        const auto& pt = cloud_src_optical->points[sub_index];
        if (isRealisticPoint(pt)) {
          single_pt_optical->points.push_back(pt);
          set_tf = true;
        }
      }
    }
  }

  if (set_tf) {
    single_pt_optical->width = single_pt_optical->points.size();
    single_pt_optical->height = 1;
    single_pt_optical->is_dense = true;

    pcl_ros::transformPointCloud(*single_pt_optical, *point_cloud_bbox_base, transform);
    point_cloud_bbox_base->header.frame_id = base_frame_name_;

    object_point.x = point_cloud_bbox_base->points[0].x;
    object_point.y = point_cloud_bbox_base->points[0].y;
    object_point.z = point_cloud_bbox_base->points[0].z;

    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position = object_point;
    obj_pose.orientation = get_quat_from_euler(object_rotate);

    vision_msgs::msg::ObjectHypothesisWithPose ohwp;
    ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
    ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
    ohwp.pose.pose = obj_pose;
    ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
    
    object_pose.results.push_back(ohwp);
    object_pose.bbox.center = obj_pose;
    object_pose.bbox.size.x = 2*noise_point_cloud_range_;
    object_pose.bbox.size.y = 2*noise_point_cloud_range_;
    object_pose.bbox.size.z = 2*noise_point_cloud_range_;
    object_pose.id = bbox_msg->id;
    publishObjectTf(obj_pose, bbox_msg->id);
  }

  return object_pose;
}

vision_msgs::msg::Detection3D BboxTo3D::processBBoxDepthImage(
      const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
      const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
      const std::shared_ptr<sensor_msgs::msg::Image>       img_msg, 
      PointCloud::Ptr& point_cloud_bbox) {

  vision_msgs::msg::Detection3D object_pose;
  object_pose.header = info_msg->header;
  object_pose.header.frame_id = base_frame_name_;

  int bytes_per_pixel = img_msg->step / img_msg->width;
  int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
  int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
  int center_index = img_msg->step * center_y + center_x * bytes_per_pixel;

  geometry_msgs::msg::Point object_point, object_rotate;
  bool set_tf = false;

  if ((0 <= center_index) && (center_index < static_cast<int>(img_msg->data.size()))) {
    if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const float* data = reinterpret_cast<const float*>(&img_msg->data[center_index]);
      object_point.z = *data;
    } else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      const uint16_t* data = reinterpret_cast<const uint16_t*>(&img_msg->data[center_index]);
      object_point.z = static_cast<float>(*data) / 1000.;
    } else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32SC1) {
      const int32_t* data = reinterpret_cast<const int32_t*>(&img_msg->data[center_index]);
      object_point.z = static_cast<float>(*data) / 1000.;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", img_msg->encoding.c_str());
      return object_pose;
    }
    if (std::isfinite(object_point.z)) {
      double fx = info_msg->k[0];
      double fy = info_msg->k[4];
      double cx = info_msg->k[2];
      double cy = info_msg->k[5];

      PointT pt_check;
      pt_check.z = object_point.z;
      pt_check.x = (center_x - cx) * object_point.z / fx;
      pt_check.y = (center_y - cy) * object_point.z / fy;

      if (isRealisticPoint(pt_check)) {
        object_point.x = pt_check.x;
        object_point.y = pt_check.y;
        set_tf = true;
      }
    }
  }

  if (set_tf) {
    double fx = info_msg->k[0];
    double fy = info_msg->k[4];
    double cx = info_msg->k[2];
    double cy = info_msg->k[5];

    object_point.x = (center_x - cx) * object_point.z / fx;
    object_point.y = (center_y - cy) * object_point.z / fy;

    geometry_msgs::msg::PointStamped object_point_stamped;
    object_point_stamped.header = info_msg->header;
    object_point_stamped.point = object_point;

    try {
      tfBuffer_->transform(object_point_stamped, object_point_stamped, base_frame_name_);
      object_point = object_point_stamped.point;
    } catch (tf2::TransformException &ex) {set_tf = false;}
  }

  if (set_tf) {
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position = object_point;
    obj_pose.orientation = get_quat_from_euler(object_rotate);

    PointT pt;
    pt.x = obj_pose.position.x; pt.y = obj_pose.position.y; pt.z = obj_pose.position.z;
    point_cloud_bbox->points.push_back(pt);
    
    point_cloud_bbox->width = point_cloud_bbox->points.size();
    point_cloud_bbox->height = 1;
    point_cloud_bbox->is_dense = true;

    vision_msgs::msg::ObjectHypothesisWithPose ohwp;
    ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
    ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
    ohwp.pose.pose = obj_pose;
    ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
    
    object_pose.results.push_back(ohwp);
    object_pose.bbox.center = obj_pose;
    object_pose.bbox.size.x = 2*noise_point_cloud_range_;
    object_pose.bbox.size.y = 2*noise_point_cloud_range_;
    object_pose.bbox.size.z = 2*noise_point_cloud_range_;
    object_pose.id = bbox_msg->id;
    publishObjectTf(obj_pose, bbox_msg->id);
  }

  return object_pose;
}

void BboxTo3D::callback_BBoxPointCloud(
    const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2>      pcl_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>       info_msg) {

  PointCloud::Ptr cloud_src_optical(new PointCloud());
  pcl::fromROSMsg(*pcl_msg, *cloud_src_optical);

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
  object_pose_array.header = bbox_msg->header;
  object_pose_array.header.frame_id = base_frame_name_;

  PointCloud::Ptr combined_cloud(new PointCloud());
  combined_cloud->header.frame_id = base_frame_name_;

  PointCloud::Ptr point_cloud_bbox(new PointCloud());

  for (size_t i = 0; i < bbox_msg->detections.size(); i++) {
    auto detection = std::make_shared<vision_msgs::msg::Detection2D>(bbox_msg->detections[i]);
    detection->id = generateObjectId(detection->id, i);

    point_cloud_bbox->clear();
    vision_msgs::msg::Detection3D object_pose;
    
    // Process Clustering
    if (positioning_detection_mode_ == "point_cloud") {
      object_pose = processBBoxClustering(detection, info_msg, cloud_src_optical, transformStamped, point_cloud_bbox);
    } else if (positioning_detection_mode_ == "fast_point") {
      object_pose = processBBoxFastShot(detection, info_msg, cloud_src_optical, transformStamped, point_cloud_bbox);
    }

    if (!object_pose.results.empty()) {
      object_pose_array.detections.push_back(object_pose);
      *combined_cloud += *point_cloud_bbox;
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

void BboxTo3D::callback_BBoxDepthImage(
    const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::Image>            img_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>       info_msg) {

  if (!tfBuffer_->canTransform(base_frame_name_, info_msg->header.frame_id, info_msg->header.stamp)) return;

  vision_msgs::msg::Detection3DArray object_pose_array;
  object_pose_array.header = info_msg->header;
  object_pose_array.header.frame_id = base_frame_name_;

  PointCloud::Ptr combined_cloud(new PointCloud());
  combined_cloud->header.frame_id = base_frame_name_;

  PointCloud::Ptr point_cloud_bbox(new PointCloud());

  for (size_t i = 0; i < bbox_msg->detections.size(); i++) {
    auto detection = std::make_shared<vision_msgs::msg::Detection2D>(bbox_msg->detections[i]);
    detection->id = generateObjectId(detection->id, i);

    point_cloud_bbox->clear();
    point_cloud_bbox->header.frame_id = base_frame_name_;

    vision_msgs::msg::Detection3D object_pose;
    if (positioning_detection_mode_ == "depth_image") {
      object_pose = processBBoxDepthImage(detection, info_msg, img_msg, point_cloud_bbox);
    }

    if (!object_pose.results.empty()) {
      object_pose_array.detections.push_back(object_pose);
      *combined_cloud += *point_cloud_bbox;
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

}  // namespace image_to_position

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_to_position::BboxTo3D)
