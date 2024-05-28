#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>

#include "sobits_msgs/BoundingBoxes.h"
#include "sobits_msgs/ObjectPose.h"
#include "sobits_msgs/ObjectPoseArray.h"
#include "sobits_msgs/RunCtrl.h"


typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sobits_msgs::BoundingBoxes, sensor_msgs::PointCloud2, sensor_msgs::Image> BBoxesCloudSyncPolicy;


class BboxToTF {
    private:
        ros::NodeHandle               nh_;
        ros::NodeHandle               pnh_;

        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        
        std::string                   base_frame_name_;

        std::string                   bbox_topic_name_;
        std::string                   cloud_topic_name_;
        std::string                   img_topic_name_;

        cv_bridge::CvImagePtr cv_ptr_;
        cv::Mat img_raw_;

        double                        cluster_tolerance;
        int                           min_clusterSize;
        int                           max_clusterSize;
        double                        noise_point_cloud_range;
        bool                          execute_flag_;
        bool                          is_error_;

        ros::Publisher                pub_obj_poses_;
        ros::Publisher                pub_object_cloud_;
        // ros::Publisher                pub_clusters_;

        ros::ServiceServer run_ctr_srv_;

        std::unique_ptr<message_filters::Subscriber<sobits_msgs::BoundingBoxes>> sub_bboxes_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>   sub_cloud_;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>         sub_img_;

        std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>    sync_;

        pcl::search::KdTree<PointT>::Ptr        kdtree_;
        pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
        pcl::VoxelGrid<PointT>                  voxel_;

        void callback_BBoxCloud(const sobits_msgs::BoundingBoxesConstPtr &bbox_msg,
                                const sensor_msgs::PointCloud2ConstPtr   &cloud_msg,
                                const sensor_msgs::ImageConstPtr         &img_msg ) {
            if (!execute_flag_) {
            	return;
            }
            else {
                is_error_ = false;
                PointCloud::Ptr cloud_transform(new PointCloud());
                pcl::fromROSMsg(*cloud_msg, *cloud_transform);
                geometry_msgs::TransformStamped transformStampedFrame_;
                try {
                    transformStampedFrame_ = tfBuffer_.lookupTransform(base_frame_name_, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
                    pcl_ros::transformPointCloud(*cloud_transform, *cloud_transform, transformStampedFrame_.transform);
                    is_error_ = false;
                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("Could NOT transform tf to: %s", ex.what());
                    is_error_ = true;
                }
                if (!is_error_) {
                    try {
                        cv_ptr_ = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
                        img_raw_ = cv_ptr_->image.clone();
                        if (img_raw_.empty()) {
                            ROS_ERROR("Input_image error");
                            is_error_ = true;
                        }
                    } catch ( cv_bridge::Exception &e ) {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        is_error_ = false;
                    }
                }
                if (!is_error_) {
                    sobits_msgs::ObjectPoseArray object_pose_array;
                    object_pose_array.header = bbox_msg->header;

                    for (int i=0; i<bbox_msg->bounding_boxes.size(); i++) {
                        PointCloud::Ptr cloud_bbox(new PointCloud());
                        const sobits_msgs::BoundingBox& bbox = bbox_msg->bounding_boxes[i];
                        if (((img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)) < 0) || (cloud_transform->points.size() <= (img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)))) continue;
                        if (!checkNanInf(cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)])) continue;
                        cloud_bbox->points.push_back(cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)]);
                        double max_z = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
                        double min_z = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
                        cloud_bbox->header.frame_id = base_frame_name_;
                        if ((0 <= (img_raw_.cols * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2))) && ((img_raw_.cols * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)) < cloud_transform->points.size())) {
                            if (checkNanInf(cloud_transform->points[img_raw_.cols * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)])) cloud_bbox->points.push_back(cloud_transform->points[img_raw_.cols * (int)((bbox.ymax + bbox.ymin)/2) + (int)((bbox.xmax + bbox.xmin)/2)]);
                        }
                        for (int iy = 0; iy <= (int)((bbox.ymax - bbox.ymin)/2); iy++) {
                            for (int ix = 0; ix <= (int)((bbox.xmax - bbox.xmin)/2); ix++) {
                                if ((ix == 0) && (iy == 0)) continue;
                                int index;
                                index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) - iy) + (int)((bbox.xmax + bbox.xmin)/2) - ix;
                                if ((0 <= index) && (index < cloud_transform->points.size())) {
                                    if (checkNanInf(cloud_transform->points[index])) cloud_bbox->points.push_back(cloud_transform->points[index]);
                                }
                                if (iy != 0) {
                                    index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy) + (int)((bbox.xmax + bbox.xmin)/2) - ix;
                                    if ((0 <= index) && (index < cloud_transform->points.size())) {
                                        if (checkNanInf(cloud_transform->points[index])) cloud_bbox->points.push_back(cloud_transform->points[index]);
                                    }
                                }
                                if (ix != 0) {
                                    index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) - iy) + (int)((bbox.xmax + bbox.xmin)/2) + ix;
                                    if ((0 <= index) && (index < cloud_transform->points.size())) {
                                        if (checkNanInf(cloud_transform->points[index])) cloud_bbox->points.push_back(cloud_transform->points[index]);
                                    }
                                    if (iy != 0) {
                                        index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy) + (int)((bbox.xmax + bbox.xmin)/2) + ix;
                                        if ((0 <= index) && (index < cloud_transform->points.size())) {
                                            if (checkNanInf(cloud_transform->points[index])) cloud_bbox->points.push_back(cloud_transform->points[index]);
                                        }
                                    }
                                }
                            }
                        }

                        kdtree_->setInputCloud(cloud_bbox);
                        euclid_clustering_.setInputCloud(cloud_bbox);
                        std::vector<pcl::PointIndices> cluster_indices;
                        euclid_clustering_.extract(cluster_indices);
                        if (cluster_indices.size() == 0) continue;

                        Eigen::Vector4f  min_pt, max_pt;
                        double           distance = std::numeric_limits<double>::max();

                        if (cloud_transform->points.size() == 0) {
                            if (!checkNanInf(cloud_transform->points[0])) continue;
                        }
                        for (std::vector<pcl::PointIndices>::const_iterator it     = cluster_indices.begin(),
                                                                            it_end = cluster_indices.end();
                                                                            it != it_end;
                                                                            it++) {
                            Eigen::Vector4f tmp_min_pt, tmp_max_pt;
                            pcl::getMinMax3D(*cloud_bbox, *it, tmp_min_pt, tmp_max_pt);
                            double tmp_dis = std::sqrt(std::pow(((tmp_min_pt.x() + tmp_max_pt.x()) / 2.) - cloud_bbox->points[0].x, 2)
                                                     + std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - cloud_bbox->points[0].y, 2)
                                                     + std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - cloud_bbox->points[0].z, 2));
                            
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
                        pass.setInputCloud(cloud_bbox);
                        pass.filter(*cloud_bbox);

                        pass.setFilterFieldName("y");
                        if ((max_pt.y() - noise_point_cloud_range) > (min_pt.y() + noise_point_cloud_range)) pass.setFilterLimits(min_pt.y() + noise_point_cloud_range, max_pt.y() - noise_point_cloud_range);
                        else pass.setFilterLimits(min_pt.y(), max_pt.y());
                        pass.setInputCloud(cloud_bbox);
                        pass.filter(*cloud_bbox);

                        pass.setFilterFieldName("z");
                        if (max_pt.z() > (min_pt.z() + noise_point_cloud_range)) pass.setFilterLimits(min_pt.z() + noise_point_cloud_range, max_pt.z());
                        else pass.setFilterLimits(min_pt.z(), max_pt.z());
                        pass.setInputCloud(cloud_bbox);
                        pass.filter(*cloud_bbox);

                        Eigen::Vector4f xyz_centroid;
                        pcl::compute3DCentroid(*cloud_bbox, xyz_centroid);
                        
                        geometry_msgs::PointStamped object_pt;
                        object_pt.header.frame_id = base_frame_name_;
                        object_pt.header.stamp    = ros::Time::now();
                        object_pt.point.x         = xyz_centroid.x();
                        object_pt.point.y         = xyz_centroid.y();
                        object_pt.point.y         = xyz_centroid.z();
                        sobits_msgs::ObjectPose object_pose;
                        object_pose.Class              = bbox.Class;
                        object_pose.detect_id          = i;
                        object_pose.pose.position.x    = xyz_centroid.x();
                        object_pose.pose.position.y    = xyz_centroid.y();
                        object_pose.pose.position.z    = xyz_centroid.z();
                        object_pose.pose.orientation.x = 0.0;
                        object_pose.pose.orientation.y = 0.0;
                        object_pose.pose.orientation.z = 0.0;
                        object_pose.pose.orientation.w = 1.0;

                        geometry_msgs::TransformStamped transformStampedObj;
                        transformStampedObj.header.frame_id = base_frame_name_;
                        transformStampedObj.child_frame_id = bbox.Class;
                        transformStampedObj.header.stamp = ros::Time::now();
                        transformStampedObj.transform.translation.x = xyz_centroid.x();
                        transformStampedObj.transform.translation.y = xyz_centroid.y();
                        transformStampedObj.transform.translation.z = xyz_centroid.z();
                        transformStampedObj.transform.rotation.x = 0.0;
                        transformStampedObj.transform.rotation.y = 0.0;
                        transformStampedObj.transform.rotation.z = 0.0;
                        transformStampedObj.transform.rotation.w = 1.0;

                        object_pose_array.object_poses.push_back(object_pose);
                        tfBroadcaster_.sendTransform(transformStampedObj);

                        pub_object_cloud_.publish(cloud_bbox);
                    }
                    pub_obj_poses_.publish(object_pose_array);
                }
            }
        }
        bool callback_RunCtr(sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res) {
            res.response = req.request;
            execute_flag_ = req.request;
            return true;
        }
        double euclidean_distance(PointT p1, PointT p2) {
            return (std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2)));
        }
        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
            else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
            return true;
        }
    public:
        BboxToTF() : tfListener_(tfBuffer_), nh_(), pnh_("~") {
            pnh_.param("base_frame_name", base_frame_name_, std::string("base_footprint"));
            pnh_.param("bbox_topic_name", bbox_topic_name_, std::string("objects_rect"));
            pnh_.param("cloud_topic_name", cloud_topic_name_, std::string("/points2"));
            pnh_.param("img_topic_name", img_topic_name_, std::string("/rgb/image_raw"));
            pnh_.param("execute_default", execute_flag_, true);

            pnh_.param("cluster_tolerance", cluster_tolerance, 0.01);
            pnh_.param("min_clusterSize", min_clusterSize, 100);
            pnh_.param("max_lusterSize", max_clusterSize, 20000);
            pnh_.param("noise_point_cloud_range", noise_point_cloud_range, 0.01);
            
            kdtree_.reset(new pcl::search::KdTree<PointT>);
            euclid_clustering_.setClusterTolerance(cluster_tolerance);
            euclid_clustering_.setMinClusterSize(min_clusterSize);
            euclid_clustering_.setMaxClusterSize(max_clusterSize);
            euclid_clustering_.setSearchMethod(kdtree_);
            pub_obj_poses_    = pnh_.advertise<sobits_msgs::ObjectPoseArray>("object_poses", 10);
            pub_object_cloud_ = pnh_.advertise<PointCloud>("object_cloud", 1);
            // pub_clusters_     = pnh_.advertise<visualization_msgs::MarkerArray>("clusters", 10);

            run_ctr_srv_ = pnh_.advertiseService("run_ctr", &BboxToTF::callback_RunCtr, this);

            sub_bboxes_.reset(new message_filters::Subscriber<sobits_msgs::BoundingBoxes>(nh_, bbox_topic_name_, 5));
            sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_name_, 5));
            sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, img_topic_name_, 5));
            
            sync_.reset(new message_filters::Synchronizer<BBoxesCloudSyncPolicy>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_));
            sync_->registerCallback(boost::bind(&BboxToTF::callback_BBoxCloud, this, _1, _2, _3));
        }
};




int main(int argc, char **argv) {
    ros::init(argc, argv, "bbox_to_tf");
    BboxToTF bbox_to_tf;
    ros::spin();
    return 0;
}
