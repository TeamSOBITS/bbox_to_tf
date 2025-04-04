#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>
#include <numeric>

class BboxToTfDepthImage : public rclcpp::Node
{
public:
  BboxToTfDepthImage()
  : Node("bbox_to_tf_depth_image")
  {
    // パラメータの宣言
    this->declare_parameter<std::string>("base_frame_name", "camera_base");
    this->declare_parameter<std::string>("bbox_topic_name", "/yolo_ros/object_boxes");
    this->declare_parameter<std::string>("rgb_camera_info_topic_name", "/rgb/camera_info");
    this->declare_parameter<std::string>("depth_camera_info_topic_name", "/depth/camera_info");
    this->declare_parameter<std::string>("depth_image_topic_name", "/depth/image_raw");
    this->declare_parameter<int>("average_range", 1);

    // パラメータの取得
    base_frame_name_ = this->get_parameter("base_frame_name").as_string();
    bbox_topic_name_ = this->get_parameter("bbox_topic_name").as_string();
    rgb_camera_info_topic_name_ = this->get_parameter("rgb_camera_info_topic_name").as_string();
    depth_camera_info_topic_name_ = this->get_parameter("depth_camera_info_topic_name").as_string();
    depth_image_topic_name_ = this->get_parameter("depth_image_topic_name").as_string();
    average_range_ = this->get_parameter("average_range").as_int();

    // サブスクライバの作成
    bbox_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      bbox_topic_name_, 10, std::bind(&BboxToTfDepthImage::bboxCallback, this, std::placeholders::_1));
    rgb_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      rgb_camera_info_topic_name_, 10, std::bind(&BboxToTfDepthImage::rgbInfoCallback, this, std::placeholders::_1));
    depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_camera_info_topic_name_, 10, std::bind(&BboxToTfDepthImage::depthInfoCallback, this, std::placeholders::_1));
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_name_, 10, std::bind(&BboxToTfDepthImage::depthImageCallback, this, std::placeholders::_1));

    // TFブロードキャスタの作成
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void bboxCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    bbox_msg_ = msg;
    processBboxes();
  }

  void rgbInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    rgb_info_ = msg;
  }

  void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    depth_info_ = msg;
  }

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    depth_image_ = msg;
    processBboxes();
  }

  void processBboxes()
  {
    if (!bbox_msg_ || !rgb_info_ || !depth_info_ || !depth_image_) {
      return;
    }

    for (const auto& detection : bbox_msg_->detections) {
      float center_x_rgb = detection.bbox.center.position.x;
      float center_y_rgb = detection.bbox.center.position.y;

      double normalized_x = center_x_rgb / static_cast<double>(rgb_info_->width);
      double normalized_y = center_y_rgb / static_cast<double>(rgb_info_->height);

      int center_x_depth = static_cast<int>(normalized_x * depth_info_->width);
      int center_y_depth = static_cast<int>(normalized_y * depth_info_->height);

      std::vector<double> x_coords, y_coords, z_coords;

      for (int dy = -average_range_ + 1; dy < average_range_; ++dy) {
        for (int dx = -average_range_ + 1; dx < average_range_; ++dx) {
          int u = center_x_depth + dx;
          int v = center_y_depth + dy;

          if (u >= 0 && u < depth_info_->width && v >= 0 && v < depth_info_->height) {
            float depth_value = getDepthValue(u, v);
            if (std::isfinite(depth_value) && depth_value > 0) {
              double fx = depth_info_->k[0];
              double fy = depth_info_->k[4];
              double cx = depth_info_->k[2];
              double cy = depth_info_->k[5];

              x_coords.push_back((u - cx) * depth_value / fx);
              y_coords.push_back((v - cy) * depth_value / fy);
              z_coords.push_back(depth_value);
            }
          }
        }
      }

      if (!x_coords.empty()) {
        double avg_x = std::accumulate(x_coords.begin(), x_coords.end(), 0.0) / x_coords.size();
        double avg_y = std::accumulate(y_coords.begin(), y_coords.end(), 0.0) / y_coords.size();
        double avg_z = std::accumulate(z_coords.begin(), z_coords.end(), 0.0) / z_coords.size();

        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = depth_info_->header.frame_id;
        transform_stamped.child_frame_id = detection.results[0].hypothesis.class_id;
        transform_stamped.transform.translation.x = avg_x;
        transform_stamped.transform.translation.y = avg_y;
        transform_stamped.transform.translation.z = avg_z;
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform_stamped);
      }
    }
    bbox_msg_.reset();
  }

private:
  float getDepthValue(int u, int v) const
  {
    if (!depth_image_) {
      return std::nanf("");
    }
    if (depth_image_->encoding == "16UC1") {
      const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_image_->data.data());
      int index = v * depth_image_->width + u;
      if (index >= 0 && index < static_cast<int>(depth_image_->width * depth_image_->height)) {
        return static_cast<float>(depth_data[index]) / 1000.0f; // mm to meters
      }
    } else if (depth_image_->encoding == "32FC1") {
      const float* depth_data = reinterpret_cast<const float*>(depth_image_->data.data());
      int index = v * depth_image_->width + u;
      if (index >= 0 && index < static_cast<int>(depth_image_->width * depth_image_->height)) {
        return depth_data[index];
      }
    }
    return std::nanf("");
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  vision_msgs::msg::Detection2DArray::SharedPtr bbox_msg_;
  sensor_msgs::msg::CameraInfo::SharedPtr rgb_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr depth_info_;
  sensor_msgs::msg::Image::SharedPtr depth_image_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string base_frame_name_;
  std::string bbox_topic_name_;
  std::string rgb_camera_info_topic_name_;
  std::string depth_camera_info_topic_name_;
  std::string depth_image_topic_name_;
  int average_range_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BboxToTfDepthImage>());
  rclcpp::shutdown();
  return 0;
}