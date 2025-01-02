#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <iostream>
#include <unordered_map>

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> BBoxesCloudSyncPolicy;

class BboxTo3D : public rclcpp::Node {
    private:
        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;

        std::string                   base_frame_name_;
        std::string                   bbox_topic_name_;
        std::string                   cloud_topic_name_;
        std::string                   img_topic_name_;

        double                        cluster_tolerance;
        int                           min_clusterSize;
        int                           max_clusterSize;
        double                        noise_point_cloud_range;

        PointCloud::Ptr cloud_transformed_;

        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_obj_poses_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;

        std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> sub_bboxes_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>      sub_pcl_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>            sub_img_;

        std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>            sync_;

        pcl::search::KdTree<PointT>::Ptr        kdtree_;
        pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;

        void callback_BBoxPCL(const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
                              const std::shared_ptr<sensor_msgs::msg::PointCloud2>      pcl_msg,
                              const std::shared_ptr<sensor_msgs::msg::Image>            img_msg ) {
            PointCloud cloud_src;

			// Transform ROS cloud to PCL
            pcl::fromROSMsg(*pcl_msg, cloud_src);

            bool can_tf = tfBuffer_.canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp);
            if (!can_tf) {
                RCLCPP_ERROR(this->get_logger(), "[BBox To 3D] canTransform() failed. \"%s\" and \"%s\" might be wrong!)", base_frame_name_.c_str(), pcl_msg->header.frame_id.c_str());
                return;
            }

            bool is_tf_pcl = pcl_ros::transformPointCloud(base_frame_name_, cloud_src, *cloud_transformed_, tfBuffer_);
            if (!is_tf_pcl) {
                RCLCPP_ERROR(this->get_logger(), "[BBox To 3D] transformPointCloud() failed. PointCloud could not be transformed");
                return;
            }

            vision_msgs::msg::Detection3DArray object_pose_array;
            object_pose_array.header = bbox_msg->header;
            object_pose_array.header.frame_id = base_frame_name_; ///

            for (size_t i=0; i<bbox_msg->detections.size(); i++) {
                PointCloud::Ptr cloud_bbox_xyz(new PointCloud());
                sensor_msgs::msg::PointCloud2 cloud_bbox;
                const vision_msgs::msg::Detection2D& bbox = bbox_msg->detections[i];
                if ((((int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)) < 0) || ((int)(cloud_transformed_->points.size()) <= ((int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)))) continue;
                if (!checkNanInf(cloud_transformed_->points[(int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)])) continue;
                cloud_bbox_xyz->points.push_back(cloud_transformed_->points[(int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)]);
                cloud_bbox_xyz->header.frame_id = base_frame_name_;
                // if ((0 <= ((int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x))) && (((int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)) < (int)(cloud_transformed_->points.size()))) {
                //     if (checkNanInf(cloud_transformed_->points[(int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)])) cloud_bbox_xyz->points.push_back(cloud_transformed_->points[(int)(img_msg->width) * ((int)(bbox.bbox.center.position.y)) + (int)(bbox.bbox.center.position.x)]);
                // }
                for (int iy = 0; iy <= (int)(bbox.bbox.size_y/2.); iy++) {
                    for (int ix = 0; ix <= (int)(bbox.bbox.size_x/2.); ix++) {
                        int dx = ix * std::cos(bbox.bbox.center.theta) - iy * std::sin(bbox.bbox.center.theta);
                        int dy = iy * std::cos(bbox.bbox.center.theta) + ix * std::sin(bbox.bbox.center.theta);
                        if ((ix == 0) && (iy == 0)) continue;
                        long index;
                        index = (int)(img_msg->width) * ((int)(bbox.bbox.center.position.y) - dy) + (int)(bbox.bbox.center.position.x) - dx;
                        if ((0 <= index) && (index < (int)(cloud_transformed_->points.size()))) {
                            if (checkNanInf(cloud_transformed_->points[index])) cloud_bbox_xyz->points.push_back(cloud_transformed_->points[index]);
                        }
                        if (iy != 0) {
                            index = (int)(img_msg->width) * ((int)(bbox.bbox.center.position.y) + dy) + (int)(bbox.bbox.center.position.x) - dx;
                            if ((0 <= index) && (index < (int)(cloud_transformed_->points.size()))) {
                                if (checkNanInf(cloud_transformed_->points[index])) cloud_bbox_xyz->points.push_back(cloud_transformed_->points[index]);
                            }
                        }
                        if (ix != 0) {
                            index = (int)(img_msg->width) * ((int)(bbox.bbox.center.position.y) - dy) + (int)(bbox.bbox.center.position.x) + dx;
                            if ((0 <= index) && (index < (int)(cloud_transformed_->points.size()))) {
                                if (checkNanInf(cloud_transformed_->points[index])) cloud_bbox_xyz->points.push_back(cloud_transformed_->points[index]);
                            }
                            if (iy != 0) {
                                index = (int)(img_msg->width) * ((int)(bbox.bbox.center.position.y) + dy) + (int)(bbox.bbox.center.position.x) + dx;
                                if ((0 <= index) && (index < (int)(cloud_transformed_->points.size()))) {
                                    if (checkNanInf(cloud_transformed_->points[index])) cloud_bbox_xyz->points.push_back(cloud_transformed_->points[index]);
                                }
                            }
                        }
                    }
                }

                kdtree_->setInputCloud(cloud_bbox_xyz);
                euclid_clustering_.setInputCloud(cloud_bbox_xyz);
                std::vector<pcl::PointIndices> cluster_indices;
                euclid_clustering_.extract(cluster_indices);
                if (cluster_indices.size() == 0) continue;

                Eigen::Vector4f  min_pt, max_pt;
                double           distance = std::numeric_limits<double>::max();

                if (cloud_transformed_->points.size() == 0) continue;
                if (!checkNanInf(cloud_transformed_->points[0])) continue;

                for (std::vector<pcl::PointIndices>::const_iterator it     = cluster_indices.begin(),
                                                                    it_end = cluster_indices.end();
                                                                    it != it_end;
                                                                    it++) {
                    Eigen::Vector4f tmp_min_pt, tmp_max_pt;
                    pcl::getMinMax3D(*cloud_bbox_xyz, *it, tmp_min_pt, tmp_max_pt);
                    double tmp_dis = std::sqrt(std::pow(((tmp_min_pt.x() + tmp_max_pt.x()) / 2.) - cloud_bbox_xyz->points[0].x, 2)
                                                + std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - cloud_bbox_xyz->points[0].y, 2)
                                                + std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - cloud_bbox_xyz->points[0].z, 2));
                    
                    if (distance > tmp_dis) {
                        distance = tmp_dis;
                        max_pt   = tmp_max_pt;
                        min_pt   = tmp_min_pt;
                    }
                }

                pcl::PassThrough<PointT> pass;
                pass.setFilterFieldName("x");
                if ((max_pt.x() - noise_point_cloud_range) > min_pt.x()) pass.setFilterLimits(min_pt.x(), max_pt.x() - noise_point_cloud_range);
                else pass.setFilterLimits(min_pt.x(), max_pt.x());
                pass.setInputCloud(cloud_bbox_xyz);
                pass.filter(*cloud_bbox_xyz);

                pass.setFilterFieldName("y");
                if ((max_pt.y() - noise_point_cloud_range) > (min_pt.y() + noise_point_cloud_range)) pass.setFilterLimits(min_pt.y() + noise_point_cloud_range, max_pt.y() - noise_point_cloud_range);
                else pass.setFilterLimits(min_pt.y(), max_pt.y());
                pass.setInputCloud(cloud_bbox_xyz);
                pass.filter(*cloud_bbox_xyz);

                pass.setFilterFieldName("z");
                if (max_pt.z() > (min_pt.z() + noise_point_cloud_range)) pass.setFilterLimits(min_pt.z() + noise_point_cloud_range, max_pt.z());
                else pass.setFilterLimits(min_pt.z(), max_pt.z());
                pass.setInputCloud(cloud_bbox_xyz);
                pass.filter(*cloud_bbox_xyz);

                Eigen::Vector4f xyz_centroid;
                pcl::compute3DCentroid(*cloud_bbox_xyz, xyz_centroid);

                vision_msgs::msg::Detection3D object_pose;
                vision_msgs::msg::ObjectHypothesisWithPose ohwp;
                object_pose.header = object_pose_array.header;
                ohwp.hypothesis.class_id = bbox.results[0].hypothesis.class_id;
                ohwp.hypothesis.score = bbox.results[0].hypothesis.score;
                ohwp.pose.pose.position.x = xyz_centroid.x();
                ohwp.pose.pose.position.y = xyz_centroid.y();
                ohwp.pose.pose.position.z = xyz_centroid.z();
                ohwp.pose.pose.orientation.x = 0.;
                ohwp.pose.pose.orientation.y = 0.;
                ohwp.pose.pose.orientation.z = 0.;
                ohwp.pose.pose.orientation.w = 1.;
                ohwp.pose.covariance = bbox.results[0].pose.covariance;
                object_pose.results.push_back(ohwp);
                object_pose.bbox.center.position.x = xyz_centroid.x();
                object_pose.bbox.center.position.y = xyz_centroid.y();
                object_pose.bbox.center.position.z = xyz_centroid.z();
                object_pose.bbox.center.orientation.x = 0.;
                object_pose.bbox.center.orientation.y = 0.;
                object_pose.bbox.center.orientation.z = 0.;
                object_pose.bbox.center.orientation.w = 1.;
                object_pose.bbox.size.x = max_pt.x() - min_pt.x();
                object_pose.bbox.size.y = max_pt.y() - min_pt.y();
                object_pose.bbox.size.z = max_pt.z() - min_pt.z();
                object_pose.id = bbox.id;

                geometry_msgs::msg::TransformStamped transformStampedObj;
                transformStampedObj.header.frame_id = base_frame_name_;
                transformStampedObj.child_frame_id = bbox.id;
                transformStampedObj.header.stamp = this->now();
                transformStampedObj.transform.translation.x = xyz_centroid.x();
                transformStampedObj.transform.translation.y = xyz_centroid.y();
                transformStampedObj.transform.translation.z = xyz_centroid.z();
                transformStampedObj.transform.rotation.x = 0.0;
                transformStampedObj.transform.rotation.y = 0.0;
                transformStampedObj.transform.rotation.z = 0.0;
                transformStampedObj.transform.rotation.w = 1.0;

                object_pose_array.detections.push_back(object_pose);
                tfBroadcaster_.sendTransform(transformStampedObj);

                pcl::toROSMsg(*cloud_bbox_xyz, cloud_bbox);
                pub_object_cloud_->publish(cloud_bbox);
            }
            pub_obj_poses_->publish(object_pose_array);
        }

        void callback_RunCtr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            if (req->data) {
                sub_bboxes_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, bbox_topic_name_);
                sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_);
                sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img_topic_name_);
                sync_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_pcl_, *sub_img_);
                sync_->registerCallback(&BboxTo3D::callback_BBoxPCL, this);
            } else {
                sub_bboxes_->unsubscribe();
                sub_bboxes_ = nullptr;
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
        BboxTo3D() : Node("bbox_to_3d"), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), tfBroadcaster_(this) {
            this->declare_parameter("base_frame_name", "base_footprint");
            this->declare_parameter("bbox_topic_name", "objects_rect");
            this->declare_parameter("cloud_topic_name", "/points2");
            this->declare_parameter("img_topic_name", "/rgb/image_raw");
            this->declare_parameter("execute_default", true);

            this->declare_parameter("cluster_tolerance", 0.01);
            this->declare_parameter("min_clusterSize", 100);
            this->declare_parameter("max_clusterSize", 20000);
            this->declare_parameter("noise_point_cloud_range", 0.01);


            base_frame_name_ = this->get_parameter("base_frame_name").as_string();
            bbox_topic_name_ = this->get_parameter("bbox_topic_name").as_string();
            cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
            img_topic_name_ = this->get_parameter("img_topic_name").as_string();

            cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
            min_clusterSize = this->get_parameter("min_clusterSize").as_int();
            max_clusterSize = this->get_parameter("max_clusterSize").as_int();
            noise_point_cloud_range = this->get_parameter("noise_point_cloud_range").as_double();

            cloud_transformed_.reset(new PointCloud());

            kdtree_.reset(new pcl::search::KdTree<PointT>);
            euclid_clustering_.setClusterTolerance(cluster_tolerance);
            euclid_clustering_.setMinClusterSize(min_clusterSize);
            euclid_clustering_.setMaxClusterSize(max_clusterSize);
            euclid_clustering_.setSearchMethod(kdtree_);

            // ROS publishers and subscribers
            pub_obj_poses_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_3d_poses", 5);
            pub_object_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_3d_cloud", 1);

            // ROS service server
            run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>("3d/run_ctr", std::bind(&BboxTo3D::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));

            // Synchronize the bbox result and the Point Cloud
            if (this->get_parameter("execute_default").as_bool()) {
                sub_bboxes_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, bbox_topic_name_);
                sub_pcl_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_);
                sub_img_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img_topic_name_);
                sync_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_pcl_, *sub_img_);
                sync_->registerCallback(&BboxTo3D::callback_BBoxPCL, this);
            }
        }
};




int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BboxTo3D>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
