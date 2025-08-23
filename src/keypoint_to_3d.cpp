#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>





#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sobits_interfaces/msg/key_point_array.hpp"

#include "sobits_interfaces/msg/key_point.hpp"

#include <std_srvs/srv/set_bool.hpp>


typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobits_interfaces::msg::KeyPointArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo> KeysCloudSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sobits_interfaces::msg::KeyPointArray, sensor_msgs::msg::Image      , sensor_msgs::msg::CameraInfo> KeysDepthSyncPolicy;

class KeyTo3D : public rclcpp::Node {
  private:
    tf2_ros::Buffer               tfBuffer_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    std::string  base_frame_name_;

    std::string  keypoint_2d_topic_name_;
    std::string  cloud_topic_name_;
    std::string  depth_topic_name_;
    std::string  info_topic_name_;


    bool enable_id_;
    std::string positioning_detection_mode_;

    rclcpp::Publisher<sobits_interfaces::msg::KeyPointArray>::SharedPtr pub_key_3d_;


    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;

    std::shared_ptr<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>> sub_key_2d_array_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>         sub_pcl_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>               sub_img_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>          sub_info_;

    std::shared_ptr<message_filters::Synchronizer<KeysCloudSyncPolicy>> sync_point_cloud_;
    std::shared_ptr<message_filters::Synchronizer<KeysDepthSyncPolicy>> sync_depth_image_;




    std::string generateObjectId(const std::string& base_id, size_t index) const {
      return enable_id_ ? (base_id + "_" + std::to_string(index)) : base_id;
    }

    geometry_msgs::msg::Quaternion get_quat_from_euler(const geometry_msgs::msg::Point& rpy) {
      tf2::Quaternion tf_quat;
      tf_quat.setRPY(rpy.x, rpy.y, rpy.z);
      return tf2::toMsg(tf_quat);
    }

    void publishObjectTf(const geometry_msgs::msg::Pose &pose, const std::string &object_id) {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = base_frame_name_;
      transformStamped.child_frame_id = object_id;

      transformStamped.transform.translation.x = pose.position.x;
      transformStamped.transform.translation.y = pose.position.y;
      transformStamped.transform.translation.z = pose.position.z;
      transformStamped.transform.rotation = pose.orientation;

      tfBroadcaster_.sendTransform(transformStamped);
    }

    void processKeysTo3D(const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
                         const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg,
                         const PointCloud::Ptr& point_cloud, const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {

      sobits_interfaces::msg::KeyPointArray pose_3d_array;
      pose_3d_array.header = pose_2d_array_msg->header;
      pose_3d_array.header.frame_id = base_frame_name_;

      for (size_t human_id = 0; human_id < pose_2d_array_msg->key_points_array.size(); human_id++) {

        sobits_interfaces::msg::KeyPoint pose_2d = pose_2d_array_msg->key_points_array[human_id];
        if ( pose_2d.key_points.size() != pose_2d.key_names.size() ) continue;

        sobits_interfaces::msg::KeyPoint pose_3d;
        pose_3d.key_names.clear();
        pose_3d.key_points.clear();
        pose_3d.score = pose_2d.score;
        for (size_t key_num = 0; key_num < pose_2d.key_points.size(); key_num++) {
          int point_x = static_cast<int>(pose_2d.key_points[key_num].x);
          int point_y = static_cast<int>(pose_2d.key_points[key_num].y);
          int point_index;

          geometry_msgs::msg::Pose part_pose;
          geometry_msgs::msg::Point part_rotate;
          bool set_tf = false;

          if (positioning_detection_mode_ == "point_cloud") {
            point_index = info_msg->width * point_y + point_x;
            // Get the 3D Pose(x,y,z) from each 2D Pose(x,y) body part by refering to the Point Cloud
            if ((0 <= point_x) && (0 <= point_y) && (0 <= point_index) && (point_index < static_cast<int>(point_cloud->points.size()))) {
              if (checkNanInf(point_cloud->points[point_index])) {
                part_pose.position.x = point_cloud->points[point_index].x;
                part_pose.position.y = point_cloud->points[point_index].y;
                part_pose.position.z = point_cloud->points[point_index].z;
                part_rotate.x = 0.; // Roll  // TODO
                part_rotate.y = 0.; // Pitch // TODO
                part_rotate.z = 0.; // Yaw   // TODO
                part_pose.orientation = get_quat_from_euler(part_rotate);
                set_tf = true;
              }
            }
          } else if (positioning_detection_mode_ == "depth_image") {

            int bytes_per_pixel = img_msg->step / img_msg->width;
            point_index = img_msg->step * point_y + point_x * bytes_per_pixel;

            if (((0 <= point_index) && (point_index < static_cast<int>(img_msg->data.size())))) {
              set_tf = true;
              if (img_msg->encoding == "32FC1") {
                const float* data = reinterpret_cast<const float*>(&img_msg->data[point_index]);
                part_pose.position.z = *data;
              } else if (img_msg->encoding == "16UC1" || img_msg->encoding == "32SC1") {
                const void* ptr = &img_msg->data[point_index];
                int raw_value;
                std::memcpy(&raw_value, ptr, bytes_per_pixel);
                part_pose.position.z = static_cast<float>(raw_value) / 1000.;
              } else set_tf = false;
            }

            if (set_tf) {
              double fx = info_msg->k[0];
              double fy = info_msg->k[4];
              double cx = info_msg->k[2];
              double cy = info_msg->k[5];

              part_pose.position.x = (point_x - cx) * part_pose.position.z / fx;
              part_pose.position.y = (point_y - cy) * part_pose.position.z / fy;

              geometry_msgs::msg::PointStamped object_point_stamped;
              object_point_stamped.header = info_msg->header;
              object_point_stamped.point = part_pose.position;

              try {
                tfBuffer_.transform(object_point_stamped, object_point_stamped, base_frame_name_);
                part_pose.position = object_point_stamped.point;
                part_rotate.x = 0.; // Roll  // TODO
                part_rotate.y = 0.; // Pitch // TODO
                part_rotate.z = 0.; // Yaw   // TODO
                part_pose.orientation = get_quat_from_euler(part_rotate);
              } catch (tf2::TransformException &ex) {set_tf = false;}
            }

          } else return;

          if (set_tf) {
            pose_3d.key_names.push_back(pose_2d.key_names[key_num]);
            pose_3d.key_points.push_back(part_pose.position);

            // Send to TF
            publishObjectTf(part_pose, generateObjectId(pose_2d.key_names[key_num], human_id));
          }
        }

        // Introduce the data into the msg
        pose_3d_array.key_points_array.push_back(pose_3d);
      }

      // Publish the result of the 3D Pose estimation
      pub_key_3d_->publish(pose_3d_array);
    }

    void callback_KeyPointCloud(const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
                                const std::shared_ptr<sensor_msgs::msg::PointCloud2>         pcl_msg,
                                const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg) {
      std::shared_ptr<sensor_msgs::msg::Image> dummy_img;

      PointCloud::Ptr cloud_src(new PointCloud());
      pcl::fromROSMsg(*pcl_msg, *cloud_src);
      if (!tfBuffer_.canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp)) return;
      if (!pcl_ros::transformPointCloud(base_frame_name_, *cloud_src, *cloud_src, tfBuffer_)) return;

      if (positioning_detection_mode_ == "point_cloud") processKeysTo3D(pose_2d_array_msg, info_msg, cloud_src, dummy_img);
    }

    void callback_KeyDepthImage(const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
                                const std::shared_ptr<sensor_msgs::msg::Image>               img_msg,
                                const std::shared_ptr<sensor_msgs::msg::CameraInfo>          info_msg) {
      PointCloud::Ptr dummy_cloud(new PointCloud());
      if (positioning_detection_mode_ == "depth_image") processKeysTo3D(pose_2d_array_msg, info_msg, dummy_cloud, img_msg);
    }

    void callback_runctr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      if (req->data) {
        rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;
        if (!sub_key_2d_array_) sub_key_2d_array_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>>(this, keypoint_2d_topic_name_);
        if (!sub_pcl_)          sub_pcl_          = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_, sensor_qos_profile);
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

    bool checkNanInf(PointT pt) {
      return !((std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) || (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)));
    }

  public:
    KeyTo3D() : Node("key_to_3d"), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfBroadcaster_(this), sub_key_2d_array_(), sub_pcl_(), sub_img_(), sub_info_(), sync_point_cloud_(), sync_depth_image_() {

      this->declare_parameter("base_frame_name", "base_footprint");
      this->declare_parameter("keypoints_topic_name", "pose_array");
      this->declare_parameter("cloud_topic_name", "dummy_pointcloud");
      this->declare_parameter("depth_image_topic_name", "dummy_image");
      this->declare_parameter("info_topic_name", "dummy_info");
      this->declare_parameter("execute_default", true);

      this->declare_parameter("enable_id", false);
      this->declare_parameter("positioning_detection_mode", "point_cloud");


      base_frame_name_ = this->get_parameter("base_frame_name").as_string();
      keypoint_2d_topic_name_ = this->get_parameter("keypoints_topic_name").as_string();
      cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
      depth_topic_name_ = this->get_parameter("depth_image_topic_name").as_string();
      info_topic_name_ = this->get_parameter("info_topic_name").as_string();


      enable_id_ = this->get_parameter("enable_id").as_bool();
      positioning_detection_mode_ = this->get_parameter("positioning_detection_mode").as_string(); // "point_cloud", "depth_image"

      // ROS publishers
      pub_key_3d_ = this->create_publisher<sobits_interfaces::msg::KeyPointArray>("keypoint_3d_array", 5);

      // ROS service server
      run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>("keypoints/run_ctr", std::bind(&KeyTo3D::callback_runctr, this, std::placeholders::_1, std::placeholders::_2));

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      request->data = this->get_parameter("execute_default").as_bool();
      callback_runctr(request, response);
    }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyTo3D>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}