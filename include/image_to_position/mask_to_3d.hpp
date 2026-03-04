#ifndef IMAGE_TO_POSITION__MASK_TO_3D_HPP_
#define IMAGE_TO_POSITION__MASK_TO_3D_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include "sobits_interfaces/msg/detect_mask_array.hpp"

namespace image_to_position
{

class MaskTo3D : public rclcpp::Node
{
public:
  explicit MaskTo3D(const rclcpp::NodeOptions & options);

private:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;
  using MaskCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sobits_interfaces::msg::DetectMaskArray, 
    sensor_msgs::msg::PointCloud2, 
    sensor_msgs::msg::CameraInfo>;

  // Callbacks
  void callback_MaskPointCloud(
    const std::shared_ptr<sobits_interfaces::msg::DetectMaskArray> mask_msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> info_msg);

  void callback_runctr(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // Processing Methods
  vision_msgs::msg::Detection3D processMaskClustering(
    const sobits_interfaces::msg::DetectMask& mask,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo>& info_msg,
    PointCloud::Ptr& mask_cloud);

  void publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id);

  // Variables
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::string base_frame_name_;
  std::string mask_topic_;
  std::string cloud_topic_;
  std::string info_topic_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double noise_point_cloud_range_;
  double voxel_leaf_size_;
  bool debug_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_obj_poses_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr      pub_object_cloud_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr               run_ctr_srv_;

  std::shared_ptr<message_filters::Subscriber<sobits_interfaces::msg::DetectMaskArray>> sub_masks_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>           sub_pcl_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>            sub_info_;
  std::shared_ptr<message_filters::Synchronizer<MaskCloudSyncPolicy>>                   sync_point_cloud_;

  pcl::search::KdTree<PointT>::Ptr kdtree_;
  pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
};

}  // namespace image_to_position

#endif  // IMAGE_TO_POSITION__MASK_TO_3D_HPP_
