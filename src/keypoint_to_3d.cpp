#include "image_to_position/keypoint_to_3d.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/point_tests.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <vector>
#include <algorithm>

namespace image_to_position
{

KeyTo3D::KeyTo3D(const rclcpp::NodeOptions & options)
: LifecycleNode("key_to_3d", options)
{
  this->declare_parameter("base_frame_name", "base_footprint");
  this->declare_parameter("keypoints_topic_name", "pose_array");
  this->declare_parameter("cloud_topic_name", "dummy_pointcloud");
  this->declare_parameter("depth_image_topic_name", "dummy_image");
  this->declare_parameter("info_topic_name", "dummy_info");

  this->declare_parameter("x_min", -10.0);
  this->declare_parameter("x_max", 10.0);
  this->declare_parameter("y_min", -10.0);
  this->declare_parameter("y_max", 10.0);
  this->declare_parameter("z_min", -10.0);
  this->declare_parameter("z_max", 10.0);

  this->declare_parameter("positioning_detection_mode", "point_cloud");
  this->declare_parameter("keypoint_patch_size", 5);

  this->declare_parameter("enable_id", false);
}

CallbackReturn KeyTo3D::on_configure(const rclcpp_lifecycle::State &)
{
  base_frame_name_ = this->get_parameter("base_frame_name").as_string();
  keypoint_2d_topic_name_ = this->get_parameter("keypoints_topic_name").as_string();
  cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
  depth_topic_name_ = this->get_parameter("depth_image_topic_name").as_string();
  info_topic_name_ = this->get_parameter("info_topic_name").as_string();

  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  z_min_ = this->get_parameter("z_min").as_double();
  z_max_ = this->get_parameter("z_max").as_double();

  positioning_detection_mode_ = this->get_parameter("positioning_detection_mode").as_string();
  keypoint_patch_size_ = this->get_parameter("keypoint_patch_size").as_int();

  enable_id_ = this->get_parameter("enable_id").as_bool();

  RCLCPP_INFO(this->get_logger(), "Configuring KeyTo3D Node...");

  RCLCPP_INFO(this->get_logger(), "Base Frame Name: %s", base_frame_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Keypoints Topic Name: %s", keypoint_2d_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Cloud Topic Name: %s", cloud_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Depth Image Topic Name: %s", depth_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera Info Topic Name: %s", info_topic_name_.c_str());

  RCLCPP_INFO(this->get_logger(), "Clipping Bounds:");
  RCLCPP_INFO(this->get_logger(), "  x: [%f, %f]", x_min_, x_max_);
  RCLCPP_INFO(this->get_logger(), "  y: [%f, %f]", y_min_, y_max_);
  RCLCPP_INFO(this->get_logger(), "  z: [%f, %f]", z_min_, z_max_);

  RCLCPP_INFO(this->get_logger(), "Positioning Detection Mode: %s", positioning_detection_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Keypoint Patch Size: %d", keypoint_patch_size_);

  RCLCPP_INFO(this->get_logger(), "Enable ID: %s", enable_id_ ? "true" : "false");

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  pub_key_3d_ = this->create_publisher<sobits_interfaces::msg::KeyPointArray>("keypoint_3d_array", 5);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keypoint_3d_cloud", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KeyTo3D::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating KeyTo3D Node...");

  pub_key_3d_->on_activate();
  pub_debug_cloud_->on_activate();

  rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;

  // Enable IPC explicitly for the subscriber
  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  sub_key_2d_array_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray, rclcpp_lifecycle::LifecycleNode>>(this, keypoint_2d_topic_name_);
  sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(this, cloud_topic_name_, sensor_qos);
  sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(this, depth_topic_name_, sensor_qos);
  sub_info_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>>(this, info_topic_name_, sensor_qos);

  if (positioning_detection_mode_ == "point_cloud") {
    sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<KeysCloudSyncPolicy>>(
      KeysCloudSyncPolicy(200), *sub_key_2d_array_, *sub_pcl_, *sub_info_);
    sync_point_cloud_->registerCallback(&KeyTo3D::callback_KeyPointCloud, this);
  } else if (positioning_detection_mode_ == "depth_image") {
    sync_depth_image_ = std::make_shared<message_filters::Synchronizer<KeysDepthSyncPolicy>>(
      KeysDepthSyncPolicy(200), *sub_key_2d_array_, *sub_img_, *sub_info_);
    sync_depth_image_->registerCallback(&KeyTo3D::callback_KeyDepthImage, this);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid positioning_detection_mode: %s", positioning_detection_mode_.c_str());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KeyTo3D::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating KeyTo3D Node.");

  pub_key_3d_->on_deactivate();
  pub_debug_cloud_->on_deactivate();

  sync_point_cloud_.reset();
  sync_depth_image_.reset();
  sub_key_2d_array_.reset();
  sub_pcl_.reset();
  sub_img_.reset();
  sub_info_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn KeyTo3D::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up KeyTo3D Node.");
  pub_key_3d_.reset();
  pub_debug_cloud_.reset();
  tfBuffer_.reset();
  tfListener_.reset();
  tfBroadcaster_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn KeyTo3D::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down KeyTo3D Node.");
  sync_point_cloud_.reset();
  sync_depth_image_.reset();
  sub_key_2d_array_.reset();
  sub_pcl_.reset();
  sub_img_.reset();
  sub_info_.reset();
  pub_key_3d_.reset();
  pub_debug_cloud_.reset();

  return CallbackReturn::SUCCESS;
}

bool KeyTo3D::isRealisticPoint(const pcl::PointXYZ& pt) const {
  return pcl::isFinite(pt) && 
         pt.x >= x_min_ && pt.x <= x_max_ && 
         pt.y >= y_min_ && pt.y <= y_max_ && 
         pt.z >= z_min_ && pt.z <= z_max_;
}

std::string KeyTo3D::generateObjectId(const std::string& base_id, size_t index) const {
  return enable_id_ ? (base_id + "_" + std::to_string(index)) : base_id;
}

geometry_msgs::msg::Quaternion KeyTo3D::get_quat_from_euler(const geometry_msgs::msg::Point& rpy) {
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(rpy.x, rpy.y, rpy.z);
  return tf2::toMsg(tf_quat);
}

void KeyTo3D::publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id) {
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

void KeyTo3D::processKeysTo3D(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg,
    const PointCloud::Ptr& cloud_src_optical, 
    const std::shared_ptr<sensor_msgs::msg::Image> img_msg,
    const geometry_msgs::msg::TransformStamped& transform) {

  sobits_interfaces::msg::KeyPointArray pose_3d_array;
  pose_3d_array.header = pose_2d_array_msg->header;
  pose_3d_array.header.frame_id = base_frame_name_;

  // Point cloud to accumulate the 5x5 patches for debugging
  PointCloud::Ptr debug_optical_cloud(new PointCloud());
  debug_optical_cloud->header.frame_id = info_msg->header.frame_id;

  int patch_radius = keypoint_patch_size_ / 2;

  for (size_t human_id = 0; human_id < pose_2d_array_msg->key_points_array.size(); human_id++) {
    sobits_interfaces::msg::KeyPoint pose_2d = pose_2d_array_msg->key_points_array[human_id];
    if (pose_2d.key_points.size() != pose_2d.key_names.size()) continue;

    sobits_interfaces::msg::KeyPoint pose_3d;
    pose_3d.key_names.clear();
    pose_3d.key_points.clear();
    pose_3d.score = pose_2d.score;

    for (size_t key_num = 0; key_num < pose_2d.key_points.size(); key_num++) {
      int point_x = static_cast<int>(pose_2d.key_points[key_num].x);
      int point_y = static_cast<int>(pose_2d.key_points[key_num].y);

      geometry_msgs::msg::Pose part_pose;
      geometry_msgs::msg::Point part_rotate;
      bool set_tf = false;

      geometry_msgs::msg::PointStamped pt_optical;
      pt_optical.header.frame_id = info_msg->header.frame_id;
      pt_optical.header.stamp = info_msg->header.stamp;

      std::vector<PointT> valid_points;

      if (positioning_detection_mode_ == "point_cloud" && cloud_src_optical) {
        for (int dy = -patch_radius; dy <= patch_radius; ++dy) {
          for (int dx = -patch_radius; dx <= patch_radius; ++dx) {
            int nx = point_x + dx;
            int ny = point_y + dy;
            if (nx >= 0 && nx < static_cast<int>(info_msg->width) && ny >= 0 && ny < static_cast<int>(info_msg->height)) {
              int n_index = ny * info_msg->width + nx;
              if (n_index < static_cast<int>(cloud_src_optical->points.size())) {
                const auto& pt = cloud_src_optical->points[n_index];
                if (isRealisticPoint(pt)) {
                  valid_points.push_back(pt);
                }
              }
            }
          }
        }
      } else if (positioning_detection_mode_ == "depth_image" && img_msg) {
        double fx = info_msg->k[0];
        double fy = info_msg->k[4];
        double cx = info_msg->k[2];
        double cy = info_msg->k[5];
        int bytes_per_pixel = img_msg->step / img_msg->width;

        for (int dy = -patch_radius; dy <= patch_radius; ++dy) {
          for (int dx = -patch_radius; dx <= patch_radius; ++dx) {
            int nx = point_x + dx;
            int ny = point_y + dy;
            if (nx >= 0 && nx < static_cast<int>(info_msg->width) && ny >= 0 && ny < static_cast<int>(info_msg->height)) {
              int n_index = img_msg->step * ny + nx * bytes_per_pixel;
              if (n_index >= static_cast<int>(img_msg->data.size())) continue;

              float d = 0.0;
              bool valid = false;

              if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                d = *reinterpret_cast<const float*>(&img_msg->data[n_index]);
                if (std::isfinite(d) && d > 0.01) valid = true;
              } else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                d = static_cast<float>(*reinterpret_cast<const uint16_t*>(&img_msg->data[n_index])) / 1000.0f;
                if (d > 0.01) valid = true;
              } else if (img_msg->encoding == sensor_msgs::image_encodings::TYPE_32SC1) {
                d = static_cast<float>(*reinterpret_cast<const int32_t*>(&img_msg->data[n_index])) / 1000.0f;
                if (d > 0.01) valid = true;
              } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", img_msg->encoding.c_str());
                return;
              }

              if (valid) {
                PointT pt_check;
                pt_check.z = d;
                pt_check.x = (nx - cx) * d / fx;
                pt_check.y = (ny - cy) * d / fy;
                if (isRealisticPoint(pt_check)) {
                  valid_points.push_back(pt_check);
                }
              }
            }
          }
        }
      }

      if (!valid_points.empty()) {
        if (pub_debug_cloud_->get_subscription_count() > 0) {
          for (const auto& pt : valid_points) {
            debug_optical_cloud->points.push_back(pt);
          }
        }

        std::sort(valid_points.begin(), valid_points.end(),[](const PointT& a, const PointT& b) {
          return a.z < b.z;
        });
        
        PointT median_pt = valid_points[valid_points.size() / 2];
        pt_optical.point.x = median_pt.x;
        pt_optical.point.y = median_pt.y;
        pt_optical.point.z = median_pt.z;
        set_tf = true;
      }

      if (set_tf) {
        geometry_msgs::msg::PointStamped pt_base;
        tf2::doTransform(pt_optical, pt_base, transform);

        part_pose.position = pt_base.point;
        part_rotate.x = 0.; 
        part_rotate.y = 0.; 
        part_rotate.z = 0.; 
        part_pose.orientation = get_quat_from_euler(part_rotate);

        pose_3d.key_names.push_back(pose_2d.key_names[key_num]);
        pose_3d.key_points.push_back(part_pose.position);

        publishObjectTf(part_pose, generateObjectId(pose_2d.key_names[key_num], human_id));
      }
    }

    if (!pose_3d.key_names.empty()) {
      pose_3d_array.key_points_array.push_back(pose_3d);
    }
  }

  pub_key_3d_->publish(pose_3d_array);

  if (pub_debug_cloud_->get_subscription_count() > 0 && !debug_optical_cloud->points.empty()) {
    debug_optical_cloud->width = debug_optical_cloud->points.size();
    debug_optical_cloud->height = 1;
    debug_optical_cloud->is_dense = true;

    PointCloud::Ptr debug_base_cloud(new PointCloud());
    pcl_ros::transformPointCloud(*debug_optical_cloud, *debug_base_cloud, transform);
    
    sensor_msgs::msg::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(*debug_base_cloud, debug_cloud_msg);
    debug_cloud_msg.header.frame_id = base_frame_name_;
    debug_cloud_msg.header.stamp = info_msg->header.stamp;
    pub_debug_cloud_->publish(debug_cloud_msg);
  }
}

void KeyTo3D::callback_KeyPointCloud(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2>         pcl_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg) {
  
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

  if (positioning_detection_mode_ == "point_cloud") {
    processKeysTo3D(pose_2d_array_msg, info_msg, cloud_src_optical, nullptr, transformStamped);
  }
}

void KeyTo3D::callback_KeyDepthImage(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::Image>               img_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg) {
  
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_->lookupTransform(
      base_frame_name_, info_msg->header.frame_id, 
      info_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    return;
  }

  if (positioning_detection_mode_ == "depth_image") {
    processKeysTo3D(pose_2d_array_msg, info_msg, nullptr, img_msg, transformStamped);
  }
}

}  // namespace image_to_position

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_to_position::KeyTo3D)
