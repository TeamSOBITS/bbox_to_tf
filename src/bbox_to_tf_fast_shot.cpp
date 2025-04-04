#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include <iostream>
#include <string>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> BBoxesCloudSyncPolicy;

class BboxToTFFastShot : public rclcpp::Node {
private:
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    std::string base_frame_name_;
    std::string bbox_topic_name_;
    std::string cloud_topic_name_;
    std::string img_topic_name_;

    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> sub_bboxes_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_pcl_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img_;
    std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>> sync_;

    PointCloud::Ptr cloud_transformed_;

    bool publish_tf(const pcl::PointXYZ& point, const std::string& frame_id, const std::string& location) {
        if (!std::isnan(point.x) && !std::isinf(point.x) &&
            !std::isnan(point.y) && !std::isinf(point.y) &&
            !std::isnan(point.z) && !std::isinf(point.z)) {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id = base_frame_name_;
            transformStamped.child_frame_id = frame_id;

            transformStamped.transform.translation.x = point.x;
            transformStamped.transform.translation.y = point.y;
            transformStamped.transform.translation.z = point.z;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            tfBroadcaster_.sendTransform(transformStamped);
            RCLCPP_INFO(this->get_logger(), "[BboxToTF] Successfully published TF for object: %s at %s (%f, %f, %f)",
                        frame_id.c_str(), location.c_str(), point.x, point.y, point.z);
            return true;
        }
        return false;
    }

    void callback_BBoxPCL(const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
                          const std::shared_ptr<sensor_msgs::msg::PointCloud2> pcl_msg,
                          const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {
        PointCloud cloud_src;
        pcl::fromROSMsg(*pcl_msg, cloud_src);

        RCLCPP_INFO(this->get_logger(), "[BboxToTF] Received bbox message with %zu detections", bbox_msg->detections.size());
        RCLCPP_INFO(this->get_logger(), "[BboxToTF] Received point cloud message with %zu points in frame: %s", cloud_src.points.size(), pcl_msg->header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "[BboxToTF] Received image message with width: %d, height: %d", img_msg->width, img_msg->height);

        bool can_tf = tfBuffer_.canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "[BboxToTF] canTransform(%s, %s) returned: %s", base_frame_name_.c_str(), pcl_msg->header.frame_id.c_str(), can_tf ? "true" : "false");
        if (!can_tf) {
            RCLCPP_WARN(this->get_logger(), "[BboxToTF] canTransform() failed. \"%s\" and \"%s\" might be wrong!", base_frame_name_.c_str(), pcl_msg->header.frame_id.c_str());
            return;
        }

        cloud_transformed_.reset(new PointCloud());
        bool is_tf_pcl = pcl_ros::transformPointCloud(base_frame_name_, cloud_src, *cloud_transformed_, tfBuffer_);
        RCLCPP_INFO(this->get_logger(), "[BboxToTF] transformPointCloud() returned: %s with %zu points", is_tf_pcl ? "true" : "false", cloud_transformed_->points.size());
        if (!is_tf_pcl) {
            RCLCPP_ERROR(this->get_logger(), "[BboxToTF] transformPointCloud() failed.");
            return;
        }

        for (size_t i = 0; i < bbox_msg->detections.size(); ++i) {
            const auto& detection = bbox_msg->detections[i];
            int original_center_x = static_cast<int>(detection.bbox.center.position.x);
            int original_center_y = static_cast<int>(detection.bbox.center.position.y);
            int original_index = img_msg->width * original_center_y + original_center_x;
            std::string object_id = detection.id + "_" + std::to_string(i);
            bool tf_published = false;

            int width = static_cast<int>(detection.bbox.size_x);
            int height = static_cast<int>(detection.bbox.size_y);

            std::vector<std::pair<int, int>> sub_centers;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    int sub_center_x = original_center_x - width / 2 + (col * width / 3) + width / 6;
                    int sub_center_y = original_center_y - height / 2 + (row * height / 3) + height / 6;
                    sub_centers.push_back({sub_center_x, sub_center_y});
                }
            }

            // 最初に元の中心をチェック
            if (original_index >= 0 && original_index < static_cast<int>(cloud_transformed_->points.size())) {
                const auto& point = cloud_transformed_->points[original_index];
                if (publish_tf(point, object_id, "original center")) {
                    tf_published = true;
                }
            }

            if (!tf_published) {
                int count = 1;
                for (const auto& center : sub_centers) {
                    int sub_center_x = center.first;
                    int sub_center_y = center.second;
                    int sub_index = img_msg->width * sub_center_y + sub_center_x;

                    if (sub_index >= 0 && sub_index < static_cast<int>(cloud_transformed_->points.size())) {
                        const auto& point = cloud_transformed_->points[sub_index];
                        std::string location = "sub-center " + std::to_string(count);
                        if (publish_tf(point, object_id, location)) {
                            tf_published = true;
                            break; // 最初に見つかった有効な点で終了
                        }
                    }
                    count++;
                }
            }

            if (!tf_published) {
                RCLCPP_WARN(this->get_logger(), "[BboxToTF] Could not find a valid point at the center or sub-centers for object: %s", detection.id.c_str());
            }
        }
    }

public:
    BboxToTFFastShot() : Node("bbox_to_tf_fast_shot"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_), tfBroadcaster_(this) {
        this->declare_parameter("base_frame_name", "camera_base");
        this->declare_parameter("bbox_topic_name", "/yolo_ros/object_boxes");
        this->declare_parameter("cloud_topic_name", "/points2");
        this->declare_parameter("img_topic_name", "/rgb/image_raw");

        base_frame_name_ = this->get_parameter("base_frame_name").as_string();
        bbox_topic_name_ = this->get_parameter("bbox_topic_name").as_string();
        cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
        img_topic_name_ = this->get_parameter("img_topic_name").as_string();

        sub_bboxes_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, bbox_topic_name_);
        sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_);
        sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img_topic_name_);

        sync_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_pcl_, *sub_img_);
        sync_->registerCallback(&BboxToTFFastShot::callback_BBoxPCL, this);

        cloud_transformed_.reset(new PointCloud());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BboxToTFFastShot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}