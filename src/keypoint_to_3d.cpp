#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "sobits_interfaces/msg/key_point.hpp"
#include "sobits_interfaces/msg/key_point_array.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <iostream>
#include <unordered_map>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobits_interfaces::msg::KeyPointArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> KeyPointSPointSyncPolicy;

class KeyTo3D : public rclcpp::Node {
    private:
        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;

        std::string     base_frame_name_;
        std::string     keypoint_2d_topic_name_;
        std::string     cloud_topic_name_;
        std::string     img_topic_name_;

        PointCloud::Ptr cloud_transformed_;

        rclcpp::Publisher<sobits_interfaces::msg::KeyPointArray>::SharedPtr pub_result_3d_array_;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;

        std::shared_ptr<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>> sub_key_2d_array_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>         sub_pcl_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>               sub_img_;

        std::shared_ptr<message_filters::Synchronizer<KeyPointSPointSyncPolicy>>            sync_;

        void callback_KeyPointsPCL(const std::shared_ptr<sobits_interfaces::msg::KeyPointArray> pose_2d_array_msg,
                                   const std::shared_ptr<sensor_msgs::msg::PointCloud2>         pcl_msg,
                                   const std::shared_ptr<sensor_msgs::msg::Image>               img_msg) {
            PointCloud cloud_src;

			// Transform ROS cloud to PCL
            pcl::fromROSMsg(*pcl_msg, cloud_src);

            bool can_tf = tfBuffer_.canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
            if (!can_tf) {
                RCLCPP_ERROR(this->get_logger(), "[Key To 3D] canTransform() failed. \"%s\" and \"%s\" might be wrong!)", base_frame_name_.c_str(), pcl_msg->header.frame_id.c_str());
                return;
            }

            bool is_tf_pcl = pcl_ros::transformPointCloud(base_frame_name_, cloud_src, *cloud_transformed_, tfBuffer_);
            if (!is_tf_pcl) {
                RCLCPP_ERROR(this->get_logger(), "[Key To 3D] transformPointCloud() failed. PointCloud could not be transformed");
                return;
            }

            sobits_interfaces::msg::KeyPointArray pose_3d_array;
            pose_3d_array.header = pose_2d_array_msg->header;
            pose_3d_array.header.frame_id = base_frame_name_; ///

            // Human ID
            for (size_t human_id = 0; human_id < pose_2d_array_msg->key_points_array.size(); human_id++) {
                sobits_interfaces::msg::KeyPoint pose_2d = pose_2d_array_msg->key_points_array[human_id];
                if ( pose_2d.key_points.size() != pose_2d.key_names.size() ) continue;

                sobits_interfaces::msg::KeyPoint pose_3d;
                pose_3d.key_names.clear();
                pose_3d.key_points.clear();
                pose_3d.score = pose_2d.score;
                for (size_t key_num = 0; key_num < pose_2d.key_points.size(); key_num++) {
                    int point_x = (int)(pose_2d.key_points[key_num].x);
                    int point_y = (int)(pose_2d.key_points[key_num].y);

                    long index = (int)(img_msg->width) * point_y + point_x;

                    // Get the 3D Pose(x,y,z) from each 2D Pose(x,y) body part by refering to the Point Cloud
                    if ((0 <= point_x) && (0 <= point_y) && (0 <= index) && (index < (int)(cloud_transformed_->points.size()))) {
                        if (checkNanInf(cloud_transformed_->points[index])) {
                            geometry_msgs::msg::Point part_point;
                            part_point.x = cloud_transformed_->points[index].x;
                            part_point.y = cloud_transformed_->points[index].y;
                            part_point.z = cloud_transformed_->points[index].z;
                            pose_3d.key_names.push_back(pose_2d.key_names[key_num]);
                            pose_3d.key_points.push_back(part_point);

                            // Send to TF
                            geometry_msgs::msg::TransformStamped transformStamped;
                            transformStamped.header.stamp = pcl_msg->header.stamp;
                            transformStamped.header.frame_id = base_frame_name_;
                            transformStamped.child_frame_id = pose_2d.key_names[key_num];
                            transformStamped.transform.translation.x = part_point.x;
                            transformStamped.transform.translation.y = part_point.y;
                            transformStamped.transform.translation.z = part_point.z;
                            transformStamped.transform.rotation.x = 0.;
                            transformStamped.transform.rotation.y = 0.;
                            transformStamped.transform.rotation.z = 0.;
                            transformStamped.transform.rotation.w = 1.;
                            tfBroadcaster_.sendTransform(transformStamped);
                        }
                    }
                }

                // Introduce the data into the msg
                pose_3d_array.key_points_array.push_back(pose_3d);
            }

            // Publish the result of the 3D Pose estimation
            pub_result_3d_array_->publish(pose_3d_array);
        }

        void callback_RunCtr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            if (req->data) {
                sub_key_2d_array_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>>(this, keypoint_2d_topic_name_);
                sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_);
                sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img_topic_name_);
                sync_ = std::make_shared<message_filters::Synchronizer<KeyPointSPointSyncPolicy>>(KeyPointSPointSyncPolicy(200), *sub_key_2d_array_, *sub_pcl_, *sub_img_);
                sync_->registerCallback(&KeyTo3D::callback_KeyPointsPCL, this);
            } else {
                sub_key_2d_array_->unsubscribe();
                sub_key_2d_array_ = nullptr;
                sub_pcl_->unsubscribe();
                sub_pcl_ = nullptr;
                sub_img_->unsubscribe();
                sub_img_ = nullptr;
                sync_.reset();
            }
            res->success = true;
        }

        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
            else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
            return true;
        }

    public:
        KeyTo3D() : Node("key_to_3d"), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), tfBroadcaster_(this) {
            this->declare_parameter("base_frame_name", "base_footprint");
            this->declare_parameter("keypoints_topic_name", "/human_pose_2d/pose_array");
            this->declare_parameter("cloud_topic_name", "/camera/depth/points");
            this->declare_parameter("img_topic_name", "/camera/rgb/image_raw");
            this->declare_parameter("execute_default", true);


            base_frame_name_ = this->get_parameter("base_frame_name").as_string();
            keypoint_2d_topic_name_ = this->get_parameter("keypoints_topic_name").as_string();
            cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
            img_topic_name_ = this->get_parameter("img_topic_name").as_string();

            cloud_transformed_.reset(new PointCloud());

            // ROS publishers and subscribers
            pub_result_3d_array_ = this->create_publisher<sobits_interfaces::msg::KeyPointArray>("keypoint_3d_array", 5);

            // ROS service server
            run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>("3d/run_ctr", std::bind(&KeyTo3D::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));

            // Synchronize the 2D Key Point result and the Point Cloud
            if (this->get_parameter("execute_default").as_bool()) {
                sub_key_2d_array_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::KeyPointArray>>(this, keypoint_2d_topic_name_);
                sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_);
                sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img_topic_name_);
                sync_ = std::make_shared<message_filters::Synchronizer<KeyPointSPointSyncPolicy>>(KeyPointSPointSyncPolicy(200), *sub_key_2d_array_, *sub_pcl_, *sub_img_);
                sync_->registerCallback(&KeyTo3D::callback_KeyPointsPCL, this);
            }
        }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyTo3D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}