#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include "sobits_interfaces/msg/bounding_boxes.hpp"
#include "sobits_interfaces/msg/object_pose.hpp"
#include "sobits_interfaces/msg/object_pose_array.hpp"
#include "sobits_interfaces/srv/run_ctrl.hpp"

#include <iostream>
#include <unordered_map>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobits_interfaces::msg::BoundingBoxes, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> BBoxesCloudSyncPolicy;

class BboxToTF {
    private:
        rclcpp::Node::SharedPtr nd_;

        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;

        std::string                   node_name_;
        std::string                   base_frame_name_;
        std::string                   bbox_topic_name_;
        std::string                   cloud_topic_name_;
        std::string                   img_topic_name_;

        double                        cluster_tolerance;
        int                           min_clusterSize;
        int                           max_clusterSize;
        double                        noise_point_cloud_range;
        bool                          execute_flag_;

        PointCloud::Ptr cloud_transform;

        rclcpp::Publisher<sobits_interfaces::msg::ObjectPoseArray>::SharedPtr pub_obj_poses_;

        rclcpp::Service<sobits_interfaces::srv::RunCtrl>::SharedPtr run_ctr_srv_;

        std::shared_ptr<message_filters::Subscriber<sobits_interfaces::msg::BoundingBoxes>> sub_bboxes_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>   sub_cloud_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>         sub_img_;

        std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>         sync_;

        pcl::search::KdTree<PointT>::Ptr        kdtree_;
        pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;

        void callback_BBoxCloud(const std::shared_ptr<sobits_interfaces::msg::BoundingBoxes> bbox_msg,
                                const std::shared_ptr<sensor_msgs::msg::PointCloud2>   cloud_msg,
                                const std::shared_ptr<sensor_msgs::msg::Image>         img_msg ) {
            if (!execute_flag_) {
            	return;
            }
            else {
                std::string target_frame_name  = cloud_msg->header.frame_id;
                PointCloud cloud_src;

                pcl::fromROSMsg(*cloud_msg, cloud_src);

                bool can_tf = tfBuffer_.canTransform(base_frame_name_, target_frame_name, cloud_msg->header.stamp);
                if (!can_tf) {
                    RCLCPP_ERROR(nd_->get_logger(), "[BBox To TF] canTransform() failed. `base_frame_name_` and `target_frame_name` might be wrong!)");
                    return;
                }

                bool is_tf_pcl = pcl_ros::transformPointCloud(base_frame_name_, cloud_src, *cloud_transform, tfBuffer_);
                if (!is_tf_pcl) {
                    RCLCPP_ERROR(nd_->get_logger(), "[BBox To TF] transformPointCloud() failed. PointCloud could not be transformed");
                    return;
                }
                sobits_interfaces::msg::ObjectPoseArray object_pose_array;
                object_pose_array.header = bbox_msg->header;

                for (size_t i=0; i<bbox_msg->bounding_boxes.size(); i++) {
                    const sobits_interfaces::msg::BoundingBox& bbox = bbox_msg->bounding_boxes[i];
                    geometry_msgs::msg::Point pt;
                    long pcl_index = (int)(img_msg->width) * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2);
                    if ((pcl_index < 0) || ((int)(cloud_transform->points.size()) <= pcl_index)) continue;
                    if (!checkNanInf(cloud_transform->points[pcl_index])) continue;
                    pt.x = cloud_transform->points[pcl_index].x;
                    pt.y = cloud_transform->points[pcl_index].y;
                    pt.z = cloud_transform->points[pcl_index].z;

                    sobits_interfaces::msg::ObjectPose object_pose;
                    object_pose.class_name         = bbox.class_name;
                    object_pose.detect_id          = i;
                    object_pose.pose.position.x    = pt.x;
                    object_pose.pose.position.y    = pt.y;
                    object_pose.pose.position.z    = pt.z;
                    object_pose.pose.orientation.x = 0.0;
                    object_pose.pose.orientation.y = 0.0;
                    object_pose.pose.orientation.z = 0.0;
                    object_pose.pose.orientation.w = 1.0;

                    geometry_msgs::msg::TransformStamped transformStampedObj;
                    transformStampedObj.header.frame_id = base_frame_name_;
                    transformStampedObj.child_frame_id = bbox.class_name;
                    transformStampedObj.header.stamp = nd_->now();
                    transformStampedObj.transform.translation.x = pt.x;
                    transformStampedObj.transform.translation.y = pt.y;
                    transformStampedObj.transform.translation.z = pt.z;
                    transformStampedObj.transform.rotation.x = 0.0;
                    transformStampedObj.transform.rotation.y = 0.0;
                    transformStampedObj.transform.rotation.z = 0.0;
                    transformStampedObj.transform.rotation.w = 1.0;

                    object_pose_array.object_poses.push_back(object_pose);
                    tfBroadcaster_.sendTransform(transformStampedObj);
                }
                pub_obj_poses_->publish(object_pose_array);
            }
        }
        void callback_RunCtr(const std::shared_ptr<sobits_interfaces::srv::RunCtrl::Request> req, std::shared_ptr<sobits_interfaces::srv::RunCtrl::Response> res) {
            execute_flag_ = req->request;
            res->response = true;
        }
        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
            else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
            return true;
        }
    public:
        BboxToTF(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), tfBroadcaster_(nd_) {
            nd_->declare_parameter("node_name", "bbox_to_tf");
            nd_->declare_parameter("base_frame_name", "base_footprint");
            nd_->declare_parameter("bbox_topic_name", "objects_rect");
            nd_->declare_parameter("cloud_topic_name", "/points2");
            nd_->declare_parameter("img_topic_name", "/rgb/image_raw");
            nd_->declare_parameter("execute_default", true);

            nd_->declare_parameter("cluster_tolerance", 0.01);
            nd_->declare_parameter("min_clusterSize", 100);
            nd_->declare_parameter("max_clusterSize", 20000);
            nd_->declare_parameter("noise_point_cloud_range", 0.01);


            node_name_ = nd_->get_parameter("node_name").as_string();
            base_frame_name_ = nd_->get_parameter("base_frame_name").as_string();
            bbox_topic_name_ = nd_->get_parameter("bbox_topic_name").as_string();
            cloud_topic_name_ = nd_->get_parameter("cloud_topic_name").as_string();
            img_topic_name_ = nd_->get_parameter("img_topic_name").as_string();
            execute_flag_ = nd_->get_parameter("execute_default").as_bool();

            cluster_tolerance = nd_->get_parameter("cluster_tolerance").as_double();
            min_clusterSize = nd_->get_parameter("min_clusterSize").as_int();
            max_clusterSize = nd_->get_parameter("max_clusterSize").as_int();
            noise_point_cloud_range = nd_->get_parameter("noise_point_cloud_range").as_double();

            cloud_transform.reset(new PointCloud());

            kdtree_.reset(new pcl::search::KdTree<PointT>);
            euclid_clustering_.setClusterTolerance(cluster_tolerance);
            euclid_clustering_.setMinClusterSize(min_clusterSize);
            euclid_clustering_.setMaxClusterSize(max_clusterSize);
            euclid_clustering_.setSearchMethod(kdtree_);

            pub_obj_poses_ = nd_->create_publisher<sobits_interfaces::msg::ObjectPoseArray>(node_name_ + "/object_poses", 10);

            run_ctr_srv_ = nd_->create_service<sobits_interfaces::srv::RunCtrl>(node_name_ + "/run_ctr", std::bind(&BboxToTF::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));

            sub_bboxes_ = std::make_shared<message_filters::Subscriber<sobits_interfaces::msg::BoundingBoxes>>(nd_, bbox_topic_name_);
            sub_cloud_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(nd_, cloud_topic_name_);
            sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(nd_, img_topic_name_);
            sync_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_);
            sync_->registerCallback(&BboxToTF::callback_BBoxCloud, this);
        }
};




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nd = std::make_shared<rclcpp::Node>("bbox_to_tf");
    auto bbox_to_tf = std::make_shared<BboxToTF>(nd);
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}
