#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace image_to_position
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BboxTo3D : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BboxTo3D(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using BBoxesCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo>;
  using BBoxesDepthSyncPolicy = message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

  // Callbacks
  void callback_BBoxPointCloud(
    const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg);

  void callback_BBoxDepthImage(
    const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::Image> img_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg);

  // Processing Methods
  vision_msgs::msg::Detection3D processBBoxClustering(
    const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg,
    const PointCloud::Ptr& cloud_src_optical, 
    const geometry_msgs::msg::TransformStamped& transform,
    PointCloud::Ptr& point_cloud_bbox_base);

  vision_msgs::msg::Detection3D processBBoxFastShot(
    const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg,
    const PointCloud::Ptr& cloud_src_optical, 
    const geometry_msgs::msg::TransformStamped& transform,
    PointCloud::Ptr& point_cloud_bbox_base);

  vision_msgs::msg::Detection3D processBBoxDepthImage(
    const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg,
    const std::shared_ptr<sensor_msgs::msg::Image> img_msg, 
    PointCloud::Ptr& point_cloud_bbox);

  // Helpers
  std::string generateObjectId(const std::string& base_id, size_t index) const;
  geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Point& rpy);
  void publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id);
  bool isRealisticPoint(const pcl::PointXYZ& pt) const;

  // Variables
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::string base_frame_name_;
  std::string bbox_topic_name_;
  std::string cloud_topic_name_;
  std::string depth_topic_name_;
  std::string info_topic_name_;

  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double noise_point_cloud_range_;
  std::string positioning_detection_mode_;
  double voxel_leaf_size_;

  bool enable_id_;
  bool enable_auto_rotation_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>> pub_obj_poses_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>>      pub_debug_cloud_;

  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray, rclcpp_lifecycle::LifecycleNode>> sub_bboxes_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>      sub_pcl_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>            sub_img_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>>       sub_info_;

  std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>> sync_point_cloud_;
  std::shared_ptr<message_filters::Synchronizer<BBoxesDepthSyncPolicy>> sync_depth_image_;

  pcl::search::KdTree<PointT>::Ptr kdtree_;
  pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
};

}  // namespace image_to_position
