#ifndef IMAGE_TO_POSITION__BBOX_TO_3D_HPP_
#define IMAGE_TO_POSITION__BBOX_TO_3D_HPP_

#include <rclcpp/rclcpp.hpp>

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
#include <std_srvs/srv/set_bool.hpp>

namespace image_to_position
{

class BboxTo3D : public rclcpp::Node
{
public:
  explicit BboxTo3D(const rclcpp::NodeOptions & options);

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

  void callback_runctr(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

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

  double cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;
  double noise_point_cloud_range_;
  std::string positioning_detection_mode_;
  double voxel_leaf_size_;

  double min_realistic_depth_;
  double max_realistic_depth_;

  bool enable_id_;
  bool debug_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_obj_poses_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      pub_object_cloud_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr               run_ctr_srv_;

  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> sub_bboxes_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>      sub_pcl_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>            sub_img_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>       sub_info_;

  std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>> sync_point_cloud_;
  std::shared_ptr<message_filters::Synchronizer<BBoxesDepthSyncPolicy>> sync_depth_image_;

  pcl::search::KdTree<PointT>::Ptr kdtree_;
  pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
};

}  // namespace image_to_position

#endif  // IMAGE_TO_POSITION__BBOX_TO_3D_HPP_
