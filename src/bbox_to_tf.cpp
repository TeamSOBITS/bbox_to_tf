#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
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
#include "sobits_msgs/msg/bounding_boxes.hpp"
#include "sobits_msgs/msg/object_pose.hpp"
#include "sobits_msgs/msg/object_pose_array.hpp"
#include "sobits_msgs/srv/run_ctrl.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobits_msgs::msg::BoundingBoxes, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> BBoxesCloudSyncPolicy;

class BboxToTF{
public:
    BboxToTF(std::shared_ptr<rclcpp::Node> nd) : nd_(nd), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tf_listener_(tf_buffer_), tf_broadcaster_(nd_) {
        // Initialize parameters
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

        // Get parameters
        node_name_ = nd_->get_parameter("node_name").as_string();
        base_frame_name_ = nd_->get_parameter("base_frame_name").as_string();
        bbox_topic_name_ = nd_->get_parameter("bbox_topic_name").as_string();
        cloud_topic_name_ = nd_->get_parameter("cloud_topic_name").as_string();
        img_topic_name_ = nd_->get_parameter("img_topic_name").as_string();
        execute_flag_ = nd_->get_parameter("execute_default").as_bool();
        cluster_tolerance_ = nd_->get_parameter("cluster_tolerance").as_double();
        min_clusterSize_ = nd_->get_parameter("min_clusterSize").as_int();
        max_clusterSize_ = nd_->get_parameter("max_clusterSize").as_int();
        noise_point_cloud_range_ = nd_->get_parameter("noise_point_cloud_range").as_double();
        // nd_->get_parameter("node_name", node_name_);
        // nd_->get_parameter("base_frame_name", base_frame_name_);
        // nd_->get_parameter("bbox_topic_name", bbox_topic_name_);
        // nd_->get_parameter("cloud_topic_name", cloud_topic_name_);
        // nd_->get_parameter("img_topic_name", img_topic_name_);
        // nd_->get_parameter("execute_default", execute_flag_);
        // nd_->get_parameter("cluster_tolerance", cluster_tolerance_);
        // nd_->get_parameter("min_clusterSize", min_clusterSize_);
        // nd_->get_parameter("max_clusterSize", max_clusterSize_);
        // nd_->get_parameter("noise_point_cloud_range", noise_point_cloud_range_);

        // Initialize PointCloud
        cloud_transform.reset(new PointCloud());

        // Initialize publishers and subscribers
        pub_obj_poses_ = nd_->create_publisher<sobits_msgs::msg::ObjectPoseArray>(node_name_ + "/object_poses", 10);
        pub_object_cloud_ = nd_->create_publisher<sensor_msgs::msg::PointCloud2>(node_name_ + "/object_cloud", 1);

        // Initialize services
        run_ctr_srv_ = nd_->create_service<sobits_msgs::srv::RunCtrl>(node_name_ + "/run_ctr", std::bind(&BboxToTF::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize message filters and synchronizer
        sub_bboxes_ = std::make_shared<message_filters::Subscriber<sobits_msgs::msg::BoundingBoxes>>(nd_, bbox_topic_name_);
        sub_cloud_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(nd_, cloud_topic_name_);
        sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(nd_, img_topic_name_);
        sync_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_);
        sync_->registerCallback(&BboxToTF::callback_BBoxCloud, this);
    }

private:
    void callback_BBoxCloud(const sobits_msgs::msg::BoundingBoxes::SharedPtr bbox_msg, const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, const sensor_msgs::msg::Image::SharedPtr img_msg) {
        if (!execute_flag_) return;

        is_error_ = false;
        // PointCloud cloud_transform;
        pcl::fromROSMsg(*cloud_msg, *cloud_transform);

        geometry_msgs::msg::TransformStamped transformStampedFrame_;
        try {
            transformStampedFrame_ = tf_buffer_.lookupTransform(base_frame_name_, cloud_msg->header.frame_id, tf2::TimePointZero);
            pcl_ros::transformPointCloud(*cloud_transform, *cloud_transform, transformStampedFrame_);
            is_error_ = false;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(nd_->get_logger(), "Could NOT transform tf to: %s", ex.what());
            is_error_ = true;
        }

        if (is_error_) return;
        sobits_msgs::msg::ObjectPoseArray object_pose_array;
        object_pose_array.header = bbox_msg->header;
        for (int i = 0; i < bbox_msg->bounding_boxes.size(); ++i) {
            PointCloud::Ptr cloud_bbox(new PointCloud());
            const sobits_msgs::msg::BoundingBox& bbox = bbox_msg->bounding_boxes[i];

            size_t index = img_msg->width * ((bbox.ymax + bbox.ymin) / 2) + (bbox.xmax + bbox.xmin) / 2;                
            if (index < 0 || index >= cloud_transform->points.size()) continue;

            if (!checkNanInf(cloud_transform->points[img_msg->width * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)])) continue;
            cloud_bbox->points.push_back(cloud_transform->points[img_msg->width * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)]);
            double max_z = cloud_transform->points[img_msg->width * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
            double min_z = cloud_transform->points[img_msg->width * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
            cloud_bbox->header.frame_id = base_frame_name_;

            if ((0 <= (img_msg->width * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2))) && ((img_msg->width * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)) < cloud_transform->points.size())) {
                if (checkNanInf(cloud_transform->points[img_msg->width * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)])) cloud_bbox->points.push_back(cloud_transform->points[img_msg->width * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)]);
            }

            for (int iy = 0; iy <= (bbox.ymax - bbox.ymin) / 2; iy++) {
                for (int ix = 0; ix <= (bbox.xmax - bbox.xmin) / 2; ix++) {
                    if (ix == 0 && iy == 0) continue;
                    int index = img_msg->width * ((bbox.ymax + bbox.ymin) / 2 - iy) + (bbox.xmax + bbox.xmin) / 2 - ix;
                    if (index >= 0 && index < cloud_transform->points.size() && checkNanInf(cloud_transform->points[index])) {
                        cloud_bbox->points.push_back(cloud_transform->points[index]);
                    }
                    if (iy != 0) {
                        index = img_msg->width * ((bbox.ymax + bbox.ymin) / 2 + iy) + (bbox.xmax + bbox.xmin) / 2 - ix;
                        if (index >= 0 && index < cloud_transform->points.size() && checkNanInf(cloud_transform->points[index])) {
                            cloud_bbox->points.push_back(cloud_transform->points[index]);
                        }
                    }
                    if (ix != 0) {
                        index = img_msg->width * ((bbox.ymax + bbox.ymin) / 2 - iy) + (bbox.xmax + bbox.xmin) / 2 + ix;
                        if (index >= 0 && index < cloud_transform->points.size() && checkNanInf(cloud_transform->points[index])) {
                            cloud_bbox->points.push_back(cloud_transform->points[index]);
                        }
                        
                        if (iy != 0) {
                            index = img_msg->width * ((bbox.ymax + bbox.ymin) / 2 + iy) + (bbox.xmax + bbox.xmin) / 2 + ix;
                            if ((index >= 0 && index < cloud_transform->points.size()) && checkNanInf(cloud_transform->points[index])) {
                                cloud_bbox->points.push_back(cloud_transform->points[index]);
                            }
                        }
                    }
                }
            }
            RCLCPP_INFO(nd_->get_logger(), "next");

            kdtree_->setInputCloud(cloud_bbox);
            euclid_clustering_.setInputCloud(cloud_bbox);
            std::vector<pcl::PointIndices> cluster_indices;
            euclid_clustering_.extract(cluster_indices);
            if (cluster_indices.size() == 0) continue;

            Eigen::Vector4f min_pt, max_pt;
            double distance = std::numeric_limits<double>::max();
            if (cloud_transform->points.size() == 0) {
                if (!checkNanInf(cloud_transform->points[0])) continue;
            }
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(),
                                                                it_end = cluster_indices.end();
                                                                it != it_end;
                                                                it++) {
                Eigen::Vector4f tmp_min_pt, tmp_max_pt;
                pcl::getMinMax3D(*cloud_bbox, *it, tmp_min_pt, tmp_max_pt);
                double tmp_dis = std::sqrt(std::pow(((tmp_min_pt.x() + tmp_max_pt.x()) / 2.) - cloud_bbox->points[0].x, 2) +
                                            std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - cloud_bbox->points[0].y, 2) +
                                            std::pow(((tmp_min_pt.z() + tmp_max_pt.z()) / 2.) - cloud_bbox->points[0].z, 2));
                if (distance > tmp_dis) {
                    distance = tmp_dis;
                    max_pt = tmp_max_pt;
                    min_pt = tmp_min_pt;
                }
            }

            pcl::PassThrough<PointT> pass;
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_pt.x(), max_pt.x() - noise_point_cloud_range_);
            pass.setInputCloud(cloud_bbox);
            pass.filter(*cloud_bbox);

            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_pt.y() + noise_point_cloud_range_, max_pt.y() - noise_point_cloud_range_);
            pass.setInputCloud(cloud_bbox);
            pass.filter(*cloud_bbox);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_pt.z() + noise_point_cloud_range_, max_pt.z());
            pass.setInputCloud(cloud_bbox);
            pass.filter(*cloud_bbox);

            Eigen::Vector4f xyz_centroid;
            pcl::compute3DCentroid(*cloud_bbox, xyz_centroid);

            sobits_msgs::msg::ObjectPose object_pose;
            object_pose.class_name = bbox.class_name;
            object_pose.detect_id = i;
            object_pose.pose.position.x = xyz_centroid.x();
            object_pose.pose.position.y = xyz_centroid.y();
            object_pose.pose.position.z = xyz_centroid.z();
            object_pose.pose.orientation.w = 1.0;
            // RCLCPP_ERROR(nd_->get_logger(), "HELOOOO22 %s", bbox.class_name);

            geometry_msgs::msg::TransformStamped transformStampedObj;
            transformStampedObj.header.frame_id = base_frame_name_;
            transformStampedObj.child_frame_id = bbox.class_name;
            transformStampedObj.header.stamp = nd_->now();
            transformStampedObj.transform.translation.x = xyz_centroid.x();
            transformStampedObj.transform.translation.y = xyz_centroid.y();
            transformStampedObj.transform.translation.z = xyz_centroid.z();
            transformStampedObj.transform.rotation.w = 1.0;

            object_pose_array.object_poses.push_back(object_pose);
            tf_broadcaster_.sendTransform(transformStampedObj);

            // Convert cloud_bbox to sensor_msgs::msg::PointCloud2
            sensor_msgs::msg::PointCloud2 cloud_msg_out;
            pcl::toROSMsg(*cloud_bbox, cloud_msg_out);
            cloud_msg_out.header.frame_id = base_frame_name_;
            pub_object_cloud_->publish(cloud_msg_out);
        }
        pub_obj_poses_->publish(object_pose_array);
    }

    bool callback_RunCtr(const std::shared_ptr<sobits_msgs::srv::RunCtrl::Request> request, std::shared_ptr<sobits_msgs::srv::RunCtrl::Response> response) {
        response->response = request->request;
        execute_flag_ = request->request;
        return true;
    }

    bool checkNanInf(const PointT &pt) {
        return !(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) || std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z));
    }

    rclcpp::Node::SharedPtr nd_;

    rclcpp::Publisher<sobits_msgs::msg::ObjectPoseArray>::SharedPtr pub_obj_poses_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;
    rclcpp::Service<sobits_msgs::srv::RunCtrl>::SharedPtr run_ctr_srv_;
    std::shared_ptr<message_filters::Subscriber<sobits_msgs::msg::BoundingBoxes>> sub_bboxes_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_cloud_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img_;
    std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>> sync_;

    PointCloud::Ptr cloud_transform;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string node_name_;
    std::string base_frame_name_;
    std::string bbox_topic_name_;
    std::string cloud_topic_name_;
    std::string img_topic_name_;
    bool execute_flag_;
    double cluster_tolerance_;
    int min_clusterSize_;
    int max_clusterSize_;
    double noise_point_cloud_range_;
    bool is_error_;
    // cv_bridge::CvImagePtr cv_ptr_;
    // cv::Mat img_raw_;
    pcl::search::KdTree<PointT>::Ptr kdtree_;
    pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nd = std::make_shared<rclcpp::Node>("bbox_to_tf");
    auto bbox_to_tf = std::make_shared<BboxToTF>(nd);
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}