#ifndef IMAGE_TO_POSITION__KEYPOINT_TO_3D_HPP_
#define IMAGE_TO_POSITION__KEYPOINT_TO_3D_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "sobits_interfaces/msg/key_point_array.hpp"
#include "sobits_interfaces/msg/key_point.hpp"
#include <std_srvs/srv/set_bool.hpp>

namespace image_to_position
{

class KeyTo3D : public rclcpp::Node
{
public:
  explicit KeyTo3D(const rclcpp::NodeOptions & options);

private:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using KeysCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sobits_interfaces::msg::KeyPointArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo>;
  using KeysDepthSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sobits_interfaces::msg::KeyPointArray, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  // Callbacks
  void callback_KeyPointCloud(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg);

  void callback_KeyDepthImage(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::Image> img_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg);

  void callback_runctr(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // Processing Methods
  void processKeysTo3D(
    const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg,
    const PointCloud::Ptr& cloud_src_optical, 
    const std::shared_ptr<sensor_msgs::msg::Image> img_msg,
    const geometry_msgs::msg::TransformStamped& transform);

  // Helpers
  std::string generateObjectId(const std::string& base_id, size_t index) const;
  geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Point& rpy);
  void publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id);

  // Variables
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::string base_frame_name_;
  std::string keypoint_2d_topic_name_;
  std::string cloud_topic_name_;
  std::string depth_topic_name_;
  std::string info_topic_name_;

  std::string positioning_detection_mode_;
  int keypoint_patch_size_; 

  bool enable_id_;
  bool debug_;

  rclcpp::Publisher<sobits_interfaces::msg::KeyPointArray>::SharedPtr pub_key_3d_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         pub_object_cloud_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                  run_ctr_srv_;

  std::shared_ptr<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>> sub_key_2d_array_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>         sub_pcl_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>               sub_img_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>          sub_info_;

  std::shared_ptr<message_filters::Synchronizer<KeysCloudSyncPolicy>> sync_point_cloud_;
  std::shared_ptr<message_filters::Synchronizer<KeysDepthSyncPolicy>> sync_depth_image_;
};

}  // namespace image_to_position

#endif  // IMAGE_TO_POSITION__KEYPOINT_TO_3D_HPP_
