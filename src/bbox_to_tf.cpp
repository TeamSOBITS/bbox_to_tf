// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/time_synchronizer.h>

// #include <ros/ros.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// // #include <sobits_msgs/utils/print.h>
// #include "sobits_msgs/BoundingBoxes.h"
// #include "sobits_msgs/ObjectPose.h"
// #include "sobits_msgs/ObjectPoseArray.h"

// #include <geometry_msgs/Vector3Stamped.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <pcl/common/common.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/conditional_removal.h>
// #include <pcl/filters/passthrough.h>

// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>

// #include <tf2/transform_datatypes.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/core.hpp>

// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
// #include <std_msgs/Bool.h>

// #include <iostream>


// typedef pcl::PointXYZ           PointT;
// typedef pcl::PointCloud<PointT> PointCloud;
// typedef message_filters::sync_policies::ApproximateTime<sobits_msgs::BoundingBoxes, sensor_msgs::PointCloud2, sensor_msgs::Image> BBoxesCloudSyncPolicy;

// class BboxToTF {
//     private:
//         ros::NodeHandle               nh_;

//         tf2_ros::Buffer               tfBuffer_;
//         tf2_ros::TransformListener    tfListener_;
//         tf2_ros::TransformBroadcaster tfBroadcaster_;
        
//         std::string              base_frame_name_;
//         std::string              map_frame_name_;
//         std::string              cloud_topic_name_;
//         std::string              img_topic_name_;

//         cv_bridge::CvImagePtr cv_ptr_;
//         cv::Mat img_raw_;
        
//         double                   min_obj_size_;
//         double                   obj_grasping_hight_rate;
//         double                   max_distance_to_object_;
//         double                   image_width;
//         double                   cluster_tolerance;
//         int                      min_clusterSize;
//         int                      max_clusterSize;
//         double                   leaf_size;
//         bool                     execute_flag_;

//         ros::Publisher pub_obj_poses_;
//         ros::Publisher pub_object_cloud_;
//         ros::Publisher pub_clusters_;

//         std::unique_ptr<message_filters::Subscriber<sobits_msgs::BoundingBoxes>> sub_bboxes_;
//         std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>        sub_cloud_;
//         std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>              sub_img_;

//         std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>         sync_;

//         ros::Subscriber sub_ctr_;

//         pcl::search::KdTree<PointT>::Ptr        kdtree_;
//         pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;
//         pcl::VoxelGrid<PointT>                  voxel_;


//         bool checkNanInf(PointT pt) {
//             if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
//             else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
//             return true;
//         }


//         bool checkNanInf(geometry_msgs::Point pt) {
//             if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
//             else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
//             return true;
//         }


//         void callback_BBoxCloud(const sobits_msgs::BoundingBoxesConstPtr &bbox_msg,
//                                 const sensor_msgs::PointCloud2ConstPtr   &cloud_msg,
//                                 const sensor_msgs::ImageConstPtr         &img_msg ) {
//             if(execute_flag_ == false){	return;	}

//             std::string     frame_id = cloud_msg->header.frame_id;
//             geometry_msgs::TransformStamped transformStampedFrame_;

//             // Convert the PointCloud2 message to a pcl::PointCloud
//             PointCloud::Ptr cloud_transform(new PointCloud());
//             pcl::fromROSMsg(*cloud_msg, *cloud_transform);
            
//             try {
//                 transformStampedFrame_ = tfBuffer_.lookupTransform(base_frame_name_, frame_id, ros::Time(0), ros::Duration(1.0));
//                 pcl_ros::transformPointCloud(*cloud_transform, *cloud_transform, transformStampedFrame_.transform);

//             } catch (tf2::TransformException &ex) {
//                 ROS_ERROR("Could NOT transform tf to: %s", ex.what());
//                 return;

//             } catch (...) {
//                 ROS_ERROR("ERROR obj_point_publisher 54 line");
//                 return;

//             }

//             try {
//                 cv_ptr_ = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
//                 img_raw_ = cv_ptr_->image.clone();

//             } catch ( cv_bridge::Exception &e ) {
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//                 return;

//             }
//             if (img_raw_.empty() == true) {
//                 ROS_ERROR("Input_image error");
//                 return;

//             }

//             int width = img_raw_.cols;
//             // ROS_INFO("width %d", width);

//             sobits_msgs::ObjectPoseArray object_pose_array;
//             object_pose_array.header = bbox_msg->header;

//             visualization_msgs::MarkerArray marker_array;

//             for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++) {
//                 PointCloud::Ptr cloud_bbox(new PointCloud());
//                 const sobits_msgs::BoundingBox& bbox = bbox_msg->bounding_boxes[i];

//                 // bboxのpoint_cloudを取得
//                 for (int iy = bbox.ymin; iy < bbox.ymax; iy++) {
//                     for (int ix = bbox.xmin; ix < bbox.xmax; ix++) {
//                         // int    point_num = cloud_transform->width * iy + ix;
//                         int    point_num = width * iy + ix; // DEBUG
//                         cloud_bbox->points.push_back(cloud_transform->points[point_num]);
//                     }
//                 }

//                 double max_z = -std::numeric_limits<double>::max();
//                 for (int it = 0; it < cloud_bbox->points.size(); it++) {
//                         if (max_z < cloud_bbox->points[it].z) {
//                             max_z = cloud_bbox->points[it].z;
//                         }
//                 }

//                 cloud_bbox->header.frame_id = base_frame_name_;
//                 pub_object_cloud_.publish(cloud_bbox);
//                 pcl::PassThrough<PointT> pass_z;
//                 pass_z.setFilterFieldName("z");
//                 /** 物体をカメラの位置的に見上げることはあまりないので、最大の位置はほぼ確実に物体の最大位置 **/
//                 pass_z.setFilterLimits(max_z - min_obj_size_, max_z);
//                 pass_z.setInputCloud(cloud_bbox);
//                 pass_z.filter(*cloud_bbox);

//                 // nanやinfのないpoint_cloudをpub
//                 PointCloud::Ptr cloud_bbox_no_nan(new PointCloud());
//                 for (int ip = 0; ip < cloud_bbox->points.size(); ip++) {
//                     if (checkNanInf(cloud_bbox->points[ip])) {
//                     cloud_bbox_no_nan->points.push_back(cloud_bbox->points[ip]);
//                     }
//                 }
//                 cloud_bbox_no_nan->header.frame_id = base_frame_name_;
//                 pub_object_cloud_.publish(cloud_bbox_no_nan);

//                 // 物体のpoint_cloudを取得
//                 kdtree_->setInputCloud(cloud_bbox_no_nan);
//                 euclid_clustering_.setInputCloud(cloud_bbox_no_nan);
//                 std::vector<pcl::PointIndices> cluster_indices;
//                 euclid_clustering_.extract(cluster_indices);
//                 if (cluster_indices.size() == 0) {
//                     continue;
//                 }

//                 std::vector<int> obj_it;
//                 Eigen::Vector4f  min_pt, max_pt;
//                 double           distance = std::numeric_limits<double>::max();

//                 for (std::vector<pcl::PointIndices>::const_iterator it     = cluster_indices.begin(),
//                                                                     it_end = cluster_indices.end();
//                                                                     it != it_end;
//                                                                     it++) {
//                     Eigen::Vector4f tmp_min_pt, tmp_max_pt;
//                     pcl::getMinMax3D(*cloud_bbox_no_nan, *it, tmp_min_pt, tmp_max_pt);
//                     double tmp_dis = std::sqrt(std::pow((tmp_min_pt.x() + tmp_max_pt.x()) / 2., 2)
//                                              + std::pow((tmp_min_pt.y() + tmp_max_pt.y()) / 2., 2));
                    
//                     if (distance > tmp_dis) {
//                         obj_it   = it->indices;
//                         distance = tmp_dis;
//                         max_pt   = tmp_max_pt;
//                         min_pt   = tmp_min_pt;
//                     }
//                 }

//                 if (max_distance_to_object_ > 0 && distance > max_distance_to_object_){
//                     continue;
//                 }

//                 visualization_msgs::Marker marker
//                     = makeMarker(base_frame_name_, "bounding_box", i, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);
//                 marker_array.markers.push_back(marker);

//                 Eigen::Vector4f xyz_centroid;
//                 pcl::compute3DCentroid(*cloud_bbox_no_nan, obj_it, xyz_centroid);
                
//                 geometry_msgs::PointStamped object_pt;
//                 object_pt.header.frame_id = base_frame_name_;
//                 object_pt.header.stamp    = ros::Time::now();
//                 object_pt.point.x         = xyz_centroid.x();
//                 object_pt.point.y         = xyz_centroid.y();
//                 object_pt.point.z         = (max_pt.z() - min_pt.z()) * obj_grasping_hight_rate + min_pt.z();

//                 geometry_msgs::PointStamped map2object_pt;
//                 geometry_msgs::TransformStamped transformStampedMap_;
//                 try {
//                     transformStampedMap_ = tfBuffer_.lookupTransform(base_frame_name_, map_frame_name_, ros::Time(0), ros::Duration(1.0));                    
//                     map2object_pt = tfBuffer_.transform(object_pt, map_frame_name_);

//                 } catch (tf2::TransformException &ex) {
//                     ROS_ERROR("Could NOT transform tf to: %s", ex.what());
//                     return;

//                 } catch (...) {
//                     ROS_ERROR("ERROR obj_point_publisher 110 line");
//                     return;
//                 }

//                 if (!checkNanInf(map2object_pt.point)) {
//                     continue;
//                 }

//                 sobits_msgs::ObjectPose object_pose;
//                 object_pose.Class              = bbox.Class;
//                 object_pose.detect_id          = i;
//                 object_pose.pose.position.x    = map2object_pt.point.x;
//                 object_pose.pose.position.y    = map2object_pt.point.y;
//                 object_pose.pose.position.z    = map2object_pt.point.z;
//                 object_pose.pose.orientation.x = 0.0;
//                 object_pose.pose.orientation.y = 0.0;
//                 object_pose.pose.orientation.z = 0.0;
//                 object_pose.pose.orientation.w = 1.0;

//                 geometry_msgs::TransformStamped transformStampedGeoObj;
//                 transformStampedGeoObj.header.frame_id = map_frame_name_;
//                 transformStampedGeoObj.child_frame_id = bbox.Class;
//                 transformStampedGeoObj.header.stamp = ros::Time::now();
//                 transformStampedGeoObj.transform.translation.x = map2object_pt.point.x;
//                 transformStampedGeoObj.transform.translation.y = map2object_pt.point.y;
//                 transformStampedGeoObj.transform.translation.z = map2object_pt.point.z;
//                 transformStampedGeoObj.transform.rotation.x = 0.0;
//                 transformStampedGeoObj.transform.rotation.y = 0.0;
//                 transformStampedGeoObj.transform.rotation.z = 0.0;
//                 transformStampedGeoObj.transform.rotation.w = 1.0;

//                 object_pose_array.object_poses.push_back(object_pose);
//                 tfBroadcaster_.sendTransform(transformStampedGeoObj);
//             }
            
//             pub_obj_poses_.publish(object_pose_array);
//             pub_clusters_.publish(marker_array);
//             // ros::spinOnce()
//         }

//         visualization_msgs::Marker makeMarker(const std::string &    frame_id,
//                                               const std::string &    marker_ns,
//                                               int                    marker_id,
//                                               const Eigen::Vector4f &min_pt,
//                                               const Eigen::Vector4f &max_pt,
//                                               float                  r,
//                                               float                  g,
//                                               float                  b,
//                                               float                  a) const {
//             visualization_msgs::Marker marker;
//             marker.header.frame_id = frame_id;
//             marker.header.stamp    = ros::Time::now();
//             marker.ns              = marker_ns;
//             marker.id              = marker_id;
//             marker.type            = visualization_msgs::Marker::CUBE;
//             marker.action          = visualization_msgs::Marker::ADD;

//             // 中心を求める
//             marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
//             marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
//             marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

//             marker.pose.orientation.x = 0.0;
//             marker.pose.orientation.y = 0.0;
//             marker.pose.orientation.z = 0.0;
//             marker.pose.orientation.w = 1.0;

//             marker.scale.x = max_pt.x() - min_pt.x();
//             marker.scale.y = max_pt.y() - min_pt.y();
//             marker.scale.z = max_pt.z() - min_pt.z();

//             marker.color.r = r;
//             marker.color.g = g;
//             marker.color.b = b;
//             marker.color.a = a;

//             marker.lifetime = ros::Duration(0.3);

//             return marker;
//         }
//         void callbackControl( const std_msgs::Bool& msg ) {
//             if ( msg.data ) std::cout << "[" << ros::this_node::getName() << "] Turn on the sensor subscriber\n" << std::endl;
//             else std::cout << "[" << ros::this_node::getName() << "] Turn off the sensor subscriber\n" << std::endl;
//             execute_flag_ = msg.data;
//             return;
//         }
//     public:
//         BboxToTF() : tfListener_(tfBuffer_) {
//             nh_.param("obj_under_rate", min_obj_size_, 0.08);
//             nh_.param("map_frame_name", map_frame_name_, std::string("map"));
//             nh_.param("base_frame_name", base_frame_name_, std::string("base_footprint"));
//             nh_.param("cloud_topic_name", cloud_topic_name_, std::string("/points2"));
//             nh_.param("img_topic_name", img_topic_name_, std::string("/rgb/image_raw"));
//             nh_.param("obj_grasping_hight_rate", obj_grasping_hight_rate, 0.6);
//             nh_.param("max_distance_to_object", max_distance_to_object_, -1.0);
//             nh_.param("image_width", image_width, 1024.);
//             nh_.param("execute_default", execute_flag_, true);

//             nh_.param("cluster_tolerance", cluster_tolerance, 0.01);
//             nh_.param("min_clusterSize", min_clusterSize, 100);
//             nh_.param("max_lusterSize", max_clusterSize, 20000);
//             nh_.param("leaf_size", leaf_size, 0.005);
            
//             kdtree_.reset(new pcl::search::KdTree<PointT>);
//             euclid_clustering_.setClusterTolerance(cluster_tolerance);
//             euclid_clustering_.setMinClusterSize(min_clusterSize);
//             euclid_clustering_.setMaxClusterSize(max_clusterSize);
//             euclid_clustering_.setSearchMethod(kdtree_);
//             voxel_.setLeafSize(leaf_size, leaf_size, leaf_size);

//             sub_bboxes_.reset(new message_filters::Subscriber<sobits_msgs::BoundingBoxes>(nh_, "objects_rect", 5));
//             sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_name_, 5));
//             sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, img_topic_name_, 5));
            
//             sync_.reset(new message_filters::Synchronizer<BBoxesCloudSyncPolicy>(
//                 BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_));
//             sync_->registerCallback(boost::bind(&BboxToTF::callback_BBoxCloud, this, _1, _2, _3));

//             sub_ctr_ = nh_.subscribe("detect_ctrl", 10, &BboxToTF::callbackControl, this);

//             pub_obj_poses_    = nh_.advertise<sobits_msgs::ObjectPoseArray>("object_poses", 10);
//             pub_object_cloud_ = nh_.advertise<PointCloud>("object_cloud", 1);
//             pub_clusters_     = nh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
//         }
// };


// int main(int argc, char **argv) {
//     ros::init(argc, argv, "bbox_to_tf_node");
//     BboxToTF bbox_to_tf;
//     ros::AsyncSpinner spinner(1);

//     spinner.start();
//     ros::waitForShutdown();
    
//     return 0;
// }


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

        tf2_ros::Buffer               tfBuffer_;
        tf2_ros::TransformListener    tfListener_;
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        
        std::string                   base_frame_name_;
        // std::string                   map_frame_name_;

        std::string                   bbox_topic_name_;
        std::string                   cloud_topic_name_;
        std::string                   img_topic_name_;

        cv_bridge::CvImagePtr cv_ptr_;
        cv::Mat img_raw_;
        
        // double                        min_obj_size_;
        // double                        obj_grasping_hight_rate;
        // double                        max_distance_to_object_;
        double                        cluster_tolerance;
        double                        leaf_size;
        double                        same_point_to_point_distance;
        int                           min_clusterSize;
        int                           max_clusterSize;
        bool                          execute_flag_;
        bool                          is_error_;

        ros::Publisher                pub_obj_poses_;
        // ros::Publisher                pub_object_cloud_;
        ros::Publisher                pub_clusters_;

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
            if (execute_flag_) {
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
                        is_error_ = true;
                    }
                }
                if (!is_error_) {
                    // int img_width = img_raw_.cols;

                    // sobits_msgs::ObjectPoseArray object_pose_array;
                    // object_pose_array.header = bbox_msg->header;

                    for (int i=0; i<bbox_msg->bounding_boxes.size(); i++) {
                        PointCloud::Ptr cloud_bbox(new PointCloud());
                        const sobits_msgs::BoundingBox& bbox = bbox_msg->bounding_boxes[i];
                        if (((img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)) < 0) || (cloud_transform->points.size() <= (img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)))) break;
                        if (checkNanInf(cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)])) break;
                        cloud_bbox->points.push_back(cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)]);
                        double max_z = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
                        double min_z = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
                        cloud_bbox->header.frame_id = base_frame_name_;
                        for (int iy = 0; iy < (int)((bbox.ymax + bbox.ymin)/2); iy++) {
                            for (int ix = 0; ix < (int)((bbox.xmax + bbox.xmin)/2); ix++) {
                                if ((ix == 0) && (iy == 0)) continue;
                                for (int pt_i=cloud_bbox->points.size()-1; 0<=pt_i; pt_i--) {
                                    if (pt_i < 0) break;
                                    if (checkNanInf(cloud_bbox->points[pt_i])) break;
                                    int index, ix_num, iy_num;
                                    ix_num = -ix;
                                    iy_num = -iy;
                                    index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy_num) + (int)((bbox.xmax + bbox.xmin)/2) + ix_num;
                                    if ((0 <= index) && (index < cloud_transform->points.size())) {
                                        if (!checkNanInf(cloud_transform->points[index])) {
                                            if (euclidean_distance(cloud_transform->points[index], cloud_bbox->points[pt_i]) <= same_point_to_point_distance) {
                                                cloud_bbox->points.push_back(cloud_transform->points[index]);
                                                if (max_z < cloud_transform->points[index].z) max_z = cloud_transform->points[index].z;
                                                if (min_z > cloud_transform->points[index].z) min_z = cloud_transform->points[index].z;
                                            }
                                        }
                                    }
                                    if (iy != 0) {
                                        ix_num = -ix;
                                        iy_num =  iy;
                                        index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy_num) + (int)((bbox.xmax + bbox.xmin)/2) + ix_num;
                                        if ((0 <= index) && (index < cloud_transform->points.size())) {
                                            if (!checkNanInf(cloud_transform->points[index])) {
                                                if (euclidean_distance(cloud_transform->points[index], cloud_bbox->points[pt_i]) <= same_point_to_point_distance) {
                                                    cloud_bbox->points.push_back(cloud_transform->points[index]);
                                                    if (max_z < cloud_transform->points[index].z) max_z = cloud_transform->points[index].z;
                                                    if (min_z > cloud_transform->points[index].z) min_z = cloud_transform->points[index].z;
                                                }
                                            }
                                        }
                                    }
                                    if (ix == 0) break;
                                    ix_num =  ix;
                                    iy_num = -iy;
                                    index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy_num) + (int)((bbox.xmax + bbox.xmin)/2) + ix_num;
                                    if ((0 <= index) && (index < cloud_transform->points.size())) {
                                        if (!checkNanInf(cloud_transform->points[index])) {
                                            if (euclidean_distance(cloud_transform->points[index], cloud_bbox->points[pt_i]) <= same_point_to_point_distance) {
                                                cloud_bbox->points.push_back(cloud_transform->points[index]);
                                                if (max_z < cloud_transform->points[index].z) max_z = cloud_transform->points[index].z;
                                                if (min_z > cloud_transform->points[index].z) min_z = cloud_transform->points[index].z;
                                            }
                                        }
                                    }
                                    if (iy != 0) {
                                        ix_num =  ix;
                                        iy_num =  iy;
                                        index = img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2) + iy_num) + (int)((bbox.xmax + bbox.xmin)/2) + ix_num;
                                        if ((0 <= index) && (index < cloud_transform->points.size())) {
                                            if (!checkNanInf(cloud_transform->points[index])) {
                                                if (euclidean_distance(cloud_transform->points[index], cloud_bbox->points[pt_i]) <= same_point_to_point_distance) {
                                                    cloud_bbox->points.push_back(cloud_transform->points[index]);
                                                    if (max_z < cloud_transform->points[index].z) max_z = cloud_transform->points[index].z;
                                                    if (min_z > cloud_transform->points[index].z) min_z = cloud_transform->points[index].z;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        ////////
                        pcl::PassThrough<PointT> pass_z;
                        pass_z.setFilterFieldName("z");
                        pass_z.setFilterLimits(max_z - min_z, max_z);
                        pass_z.setInputCloud(cloud_bbox);
                        pass_z.filter(*cloud_bbox);
                        ////////
                        // for (int ix = 0; ix < (int)((bbox.xmax + bbox.xmin)/2); ix++) {
                        //     cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2) + ix]
                        //     cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2) - ix]
                        //     if (std::sqrt(std::pow((tmp_min_pt.x() + tmp_max_pt.x()) / 2., 2) + std::pow((tmp_min_pt.y() + tmp_max_pt.y()) / 2., 2))) {}
                        //     if () {}
                        // }
                        // for (int iy = 0; iy < (int)((bbox.ymax + bbox.ymin)/2); iy++) {}
                        // PointT center_point, min_pt, max_pt;
                        // center_point.x = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].x;
                        // center_point.y = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].y;
                        // center_point.z = cloud_transform->points[img_raw_.cols * ((int)((bbox.ymax + bbox.ymin)/2)) + (int)((bbox.xmax + bbox.xmin)/2)].z;
                        // PointCloud::Ptr cloud_bbox(new PointCloud());
                        // const sobits_msgs::BoundingBox& bbox = bbox_msg->bounding_boxes[i];
                        // for (int iy = bbox.ymin; iy < bbox.ymax; iy++) {
                        //     for (int ix = bbox.xmin; ix < bbox.xmax; ix++) {
                        //         cloud_bbox->points.push_back(cloud_transform->points[img_raw_.cols * iy + ix]);
                        //     }
                        // }
                        // double max_z = -std::numeric_limits<double>::max();
                        // for (int it = 0; it < cloud_bbox->points.size(); it++) {
                        //         if (max_z < cloud_bbox->points[it].z) {
                        //             max_z = cloud_bbox->points[it].z;
                        //         }
                        // }
                    }
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
        // ((std::pow(cloud_transform->points[index].x - cloud_bbox->points[pt_i].x, 2) + 
        //   std::pow(cloud_transform->points[index].y - cloud_bbox->points[pt_i].y, 2) +
        //   std::pow(cloud_transform->points[index].z - cloud_bbox->points[pt_i].z, 2)))
        bool checkNanInf(PointT pt) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) return false;
            else if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) return false;
            return true;
        }
    public:
        BboxToTF() : tfListener_(tfBuffer_) {
            pub_obj_poses_    = nh_.advertise<sobits_msgs::ObjectPoseArray>("/bbox_to_tf/object_poses", 10);
            // pub_object_cloud_ = nh_.advertise<PointCloud>("object_cloud", 1);
            pub_clusters_     = nh_.advertise<visualization_msgs::MarkerArray>("/bbox_to_tf/clusters", 10);

            run_ctr_srv_ = nh_.advertiseService("/bbox_to_tf/swich_ctrl", &BboxToTF::callback_RunCtr, this);

            sub_bboxes_.reset(new message_filters::Subscriber<sobits_msgs::BoundingBoxes>(nh_, bbox_topic_name_, 5));
            sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_name_, 5));
            sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, img_topic_name_, 5));
            
            sync_.reset(new message_filters::Synchronizer<BBoxesCloudSyncPolicy>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_cloud_, *sub_img_));
            sync_->registerCallback(boost::bind(&BboxToTF::callback_BBoxCloud, this, _1, _2, _3));
            // sub_ctr_ = nh_.subscribe("detect_ctrl", 10, &BboxToTF::callbackControl, this);
        }
};




int main(int argc, char **argv) {
    ros::init(argc, argv, "bbox_to_tf_node");
    BboxToTF bbox_to_tf;
    ros::spin();
    return 0;
}