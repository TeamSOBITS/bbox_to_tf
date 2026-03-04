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
: Node("key_to_3d", options)
{
  base_frame_name_ = this->declare_parameter("base_frame_name", "base_footprint");
  keypoint_2d_topic_name_ = this->declare_parameter("keypoints_topic_name", "pose_array");
  cloud_topic_name_ = this->declare_parameter("cloud_topic_name", "dummy_pointcloud");
  depth_topic_name_ = this->declare_parameter("depth_image_topic_name", "dummy_image");
  info_topic_name_ = this->declare_parameter("info_topic_name", "dummy_info");

  positioning_detection_mode_ = this->declare_parameter("positioning_detection_mode", "point_cloud");
  keypoint_patch_size_ = this->declare_parameter("keypoint_patch_size", 5);

  enable_id_ = this->declare_parameter("enable_id", false);
  debug_ = this->declare_parameter("publish_debug_cloud", true);
  bool execute_default = this->declare_parameter("execute_default", true);

  // Param info logging
  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  base_frame_name: %s", base_frame_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  keypoints topic: %s", keypoint_2d_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  cloud topic: %s", cloud_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  depth topic: %s", depth_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  info topic: %s", info_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  keypoint_patch_size: %d", keypoint_patch_size_);
  RCLCPP_INFO(this->get_logger(), "  positioning_detection_mode: %s", positioning_detection_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "  enable_id: %s", enable_id_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  debug: %s", debug_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  execute_default: %s", execute_default ? "true" : "false");

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  pub_key_3d_ = this->create_publisher<sobits_interfaces::msg::KeyPointArray>("keypoint_3d_array", 5);
  
  if (debug_) {
    pub_object_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("keypoint_3d_cloud", 1);
  }

  run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "keypoints/run_ctr", std::bind(&KeyTo3D::callback_runctr, this, std::placeholders::_1, std::placeholders::_2));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
  request->data = execute_default;
  callback_runctr(request, response);
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

      // Extract valid points from either PointCloud or DepthImage
      if (positioning_detection_mode_ == "point_cloud" && cloud_src_optical) {
        for (int dy = -patch_radius; dy <= patch_radius; ++dy) {
          for (int dx = -patch_radius; dx <= patch_radius; ++dx) {
            int nx = point_x + dx;
            int ny = point_y + dy;
            if (nx >= 0 && nx < static_cast<int>(info_msg->width) && ny >= 0 && ny < static_cast<int>(info_msg->height)) {
              int n_index = ny * info_msg->width + nx;
              if (n_index < static_cast<int>(cloud_src_optical->points.size()) && pcl::isFinite(cloud_src_optical->points[n_index])) {
                valid_points.push_back(cloud_src_optical->points[n_index]);
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
              }

              if (valid) {
                PointT pt;
                pt.z = d;
                pt.x = (nx - cx) * d / fx;
                pt.y = (ny - cy) * d / fy;
                valid_points.push_back(pt);
              }
            }
          }
        }
      }

      // Process collected points to find median
      if (!valid_points.empty()) {
        if (debug_) {
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

  // Publish the debug point cloud translated to the base frame
  if (debug_ && !debug_optical_cloud->points.empty()) {
    PointCloud::Ptr debug_base_cloud(new PointCloud());
    pcl_ros::transformPointCloud(*debug_optical_cloud, *debug_base_cloud, transform);
    
    sensor_msgs::msg::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(*debug_base_cloud, debug_cloud_msg);
    debug_cloud_msg.header.frame_id = base_frame_name_;
    debug_cloud_msg.header.stamp = info_msg->header.stamp;
    pub_object_cloud_->publish(debug_cloud_msg);
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

void KeyTo3D::callback_runctr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  if (req->data) {
    rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
    if (!sub_key_2d_array_) sub_key_2d_array_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>>(this, keypoint_2d_topic_name_);
    if (!sub_pcl_)          sub_pcl_          = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_, sensor_qos);
    if (!sub_img_)          sub_img_          = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic_name_);
    if (!sub_info_)         sub_info_         = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, info_topic_name_);

    if (positioning_detection_mode_ == "point_cloud") {
      if (!sync_point_cloud_) sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<KeysCloudSyncPolicy>>(KeysCloudSyncPolicy(200), *sub_key_2d_array_, *sub_pcl_, *sub_info_);
      sync_point_cloud_->registerCallback(&KeyTo3D::callback_KeyPointCloud, this);
    } else if (positioning_detection_mode_ == "depth_image") {
      if (!sync_depth_image_) sync_depth_image_ = std::make_shared<message_filters::Synchronizer<KeysDepthSyncPolicy>>(KeysDepthSyncPolicy(200), *sub_key_2d_array_, *sub_img_, *sub_info_);
      sync_depth_image_->registerCallback(&KeyTo3D::callback_KeyDepthImage, this);
    }

  } else {
    if (sync_point_cloud_) sync_point_cloud_.reset();
    if (sync_depth_image_) sync_depth_image_.reset();
    if (sub_key_2d_array_) {
      sub_key_2d_array_->unsubscribe();
      sub_key_2d_array_ = nullptr;
    }
    if (sub_pcl_) {
      sub_pcl_->unsubscribe();
      sub_pcl_ = nullptr;
    }
    if (sub_img_) {
      sub_img_->unsubscribe();
      sub_img_ = nullptr;
    }
    if (sub_info_) {
      sub_info_->unsubscribe();
      sub_info_ = nullptr;
    }
  }
  res->success = true;
}

}  // namespace image_to_position

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_to_position::KeyTo3D)
