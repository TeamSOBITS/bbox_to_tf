#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <std_srvs/srv/set_bool.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CameraInfo> BBoxesCloudSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<vision_msgs::msg::Detection2DArray, sensor_msgs::msg::Image      , sensor_msgs::msg::CameraInfo> BBoxesDepthSyncPolicy;

class BboxTo3D : public rclcpp::Node {
  private:
    tf2_ros::Buffer               tfBuffer_;
    tf2_ros::TransformListener    tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    std::string  base_frame_name_;

    std::string  bbox_topic_name_;
    std::string  cloud_topic_name_;
    std::string  depth_topic_name_;
    std::string  info_topic_name_;

    double noise_point_cloud_range_;
    bool enable_id_;
    std::string positioning_detection_mode_;

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_obj_poses_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_cloud_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;

    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> sub_bboxes_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>      sub_pcl_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>            sub_img_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>       sub_info_;

    std::shared_ptr<message_filters::Synchronizer<BBoxesCloudSyncPolicy>> sync_point_cloud_;
    std::shared_ptr<message_filters::Synchronizer<BBoxesDepthSyncPolicy>> sync_depth_image_;

    pcl::search::KdTree<PointT>::Ptr kdtree_;
    pcl::EuclideanClusterExtraction<PointT> euclid_clustering_;

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

    vision_msgs::msg::Detection3D processBBoxClustering(
          const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
          const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
          const PointCloud::Ptr& point_cloud, PointCloud::Ptr& point_cloud_bbox) {

      vision_msgs::msg::Detection3D object_pose;
      object_pose.header = info_msg->header;
      object_pose.header.frame_id = base_frame_name_;

      int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
      int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
      int center_index = info_msg->width * center_y + center_x;
      geometry_msgs::msg::Point object_point;
      geometry_msgs::msg::Point object_rotate;

      if ((0 <= center_index) && (center_index < static_cast<int>(point_cloud->points.size()))) {
        if (checkNanInf(point_cloud->points[center_index])) point_cloud_bbox->points.push_back(point_cloud->points[center_index]);
        else return object_pose;
      } else return object_pose;

      int width  = static_cast<int>(bbox_msg->bbox.size_x);
      int height = static_cast<int>(bbox_msg->bbox.size_y);
      for (int iy = 0; iy <= (int)(height / 2.); iy++) {
        for (int ix = 0; ix <= (int)(width / 2.); ix++) {
          int dx = ix * std::cos(bbox_msg->bbox.center.theta) - iy * std::sin(bbox_msg->bbox.center.theta);
          int dy = iy * std::cos(bbox_msg->bbox.center.theta) + ix * std::sin(bbox_msg->bbox.center.theta);
          if ((ix == 0) && (iy == 0)) continue;

          int pt_index, w, h;
          w = static_cast<int>(bbox_msg->bbox.center.position.x) - dx;
          h = static_cast<int>(bbox_msg->bbox.center.position.y) - dy;
          pt_index = info_msg->width * h + w;
          if ((0 <= pt_index) && (pt_index < static_cast<int>(point_cloud->points.size()))) {
            if (checkNanInf(point_cloud->points[pt_index])) point_cloud_bbox->points.push_back(point_cloud->points[pt_index]);
          }
          if (iy != 0) {
            w = static_cast<int>(bbox_msg->bbox.center.position.x) - dx;
            h = static_cast<int>(bbox_msg->bbox.center.position.y) + dy;
            pt_index = info_msg->width * h + w;
            if ((0 <= pt_index) && (pt_index < static_cast<int>(point_cloud->points.size()))) {
              if (checkNanInf(point_cloud->points[pt_index])) point_cloud_bbox->points.push_back(point_cloud->points[pt_index]);
            }
          }
          if (ix != 0) {
            w = static_cast<int>(bbox_msg->bbox.center.position.x) + dx;
            h = static_cast<int>(bbox_msg->bbox.center.position.y) - dy;
            pt_index = info_msg->width * h + w;
            if ((0 <= pt_index) && (pt_index < static_cast<int>(point_cloud->points.size()))) {
              if (checkNanInf(point_cloud->points[pt_index])) point_cloud_bbox->points.push_back(point_cloud->points[pt_index]);
            }
            if (iy != 0) {
              w = static_cast<int>(bbox_msg->bbox.center.position.x) + dx;
              h = static_cast<int>(bbox_msg->bbox.center.position.y) + dy;
              pt_index = info_msg->width * h + w;
              if ((0 <= pt_index) && (pt_index < static_cast<int>(point_cloud->points.size()))) {
                if (checkNanInf(point_cloud->points[pt_index])) point_cloud_bbox->points.push_back(point_cloud->points[pt_index]);
              }
            }
          }
        }
      }

      kdtree_->setInputCloud(point_cloud_bbox);
      euclid_clustering_.setInputCloud(point_cloud_bbox);
      std::vector<pcl::PointIndices> cluster_indices;
      euclid_clustering_.extract(cluster_indices);
      if (cluster_indices.size() == 0) return object_pose;
      Eigen::Vector4f min_pt, max_pt;
      double distance = std::numeric_limits<double>::max();
      if (point_cloud_bbox->points.size() == 0) return object_pose;
      if (!checkNanInf(point_cloud_bbox->points[0])) return object_pose;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end();
                                                          it != it_end;
                                                          it++) {
        Eigen::Vector4f tmp_min_pt, tmp_max_pt;
        pcl::getMinMax3D(*point_cloud_bbox, *it, tmp_min_pt, tmp_max_pt);
        double tmp_dis = std::sqrt(std::pow(((tmp_min_pt.x() + tmp_max_pt.x()) / 2.) - point_cloud_bbox->points[0].x, 2) + std::pow(((tmp_min_pt.y() + tmp_max_pt.y()) / 2.) - point_cloud_bbox->points[0].y, 2) + std::pow(((tmp_min_pt.z() + tmp_max_pt.z()) / 2.) - point_cloud_bbox->points[0].z, 2));
        if (distance > tmp_dis) {
          distance = tmp_dis;
          max_pt = tmp_max_pt;
          min_pt = tmp_min_pt;
        }
      }

      pcl::CropBox<PointT> cropBox;
      cropBox.setMin(Eigen::Vector4f(min_pt.x()                           , min_pt.y() + noise_point_cloud_range_, min_pt.z() + noise_point_cloud_range_, 1.0));
      cropBox.setMax(Eigen::Vector4f(max_pt.x() - noise_point_cloud_range_, max_pt.y() - noise_point_cloud_range_, max_pt.z()                           , 1.0));
      cropBox.setInputCloud(point_cloud_bbox);
      cropBox.filter(*point_cloud_bbox);
      Eigen::Vector4f xyz_centroid;
      pcl::compute3DCentroid(*point_cloud_bbox, xyz_centroid);
      pcl::getMinMax3D(*point_cloud_bbox, min_pt, max_pt);

      object_point.x = xyz_centroid.x();
      object_point.y = xyz_centroid.y();
      object_point.z = xyz_centroid.z();
      object_rotate.x = 0.; // Roll  // TODO
      object_rotate.y = 0.; // Pitch // TODO
      object_rotate.z = 0.; // Yaw   // TODO
      if        ((xyz_centroid.x() - min_pt.x()) > (xyz_centroid.z() - min_pt.z() + noise_point_cloud_range_/4.)) {
        object_rotate.y =  M_PI/2.;
      } else if ((xyz_centroid.y() - min_pt.y()) > (xyz_centroid.z() - min_pt.z() + noise_point_cloud_range_/4.)) {
        object_rotate.x = -M_PI/2.;
        object_rotate.y =  M_PI/2.;
      }
      geometry_msgs::msg::Pose obj_pose;
      obj_pose.position = object_point;
      obj_pose.orientation = get_quat_from_euler(object_rotate);

      vision_msgs::msg::ObjectHypothesisWithPose ohwp;
      ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
      ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
      ohwp.pose.pose = obj_pose;
      ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
      object_pose.results.push_back(ohwp);
      object_pose.bbox.center = obj_pose;
      object_pose.bbox.size.x = max_pt.x() - min_pt.x();
      object_pose.bbox.size.y = max_pt.y() - min_pt.y();
      object_pose.bbox.size.z = max_pt.z() - min_pt.z();
      object_pose.id = bbox_msg->id;
      publishObjectTf(obj_pose, bbox_msg->id);

      return object_pose;
    }

    vision_msgs::msg::Detection3D processBBoxFastShot(
          const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
          const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
          const PointCloud::Ptr& point_cloud, PointCloud::Ptr& point_cloud_bbox) {

      vision_msgs::msg::Detection3D object_pose;
      object_pose.header = info_msg->header;
      object_pose.header.frame_id = base_frame_name_;

      int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
      int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
      int center_index = info_msg->width * center_y + center_x;
      geometry_msgs::msg::Point object_point;
      geometry_msgs::msg::Point object_rotate;
      bool set_tf = false;

      if ((0 <= center_index) && (center_index < static_cast<int>(point_cloud->points.size()))) {
        if (checkNanInf(point_cloud->points[center_index])) {
          object_point.x = point_cloud->points[center_index].x;
          object_point.y = point_cloud->points[center_index].y;
          object_point.z = point_cloud->points[center_index].z;
          object_rotate.x = 0.; // Roll  // TODO
          object_rotate.y = 0.; // Pitch // TODO
          object_rotate.z = 0.; // Yaw   // TODO
          set_tf = true;
        }
      }

      int width = static_cast<int>(bbox_msg->bbox.size_x);
      int height = static_cast<int>(bbox_msg->bbox.size_y);
      for (int row = 0; row < 3; row++) {
        if (set_tf) break;
        for (int col = 0; col < 3; col++) {
          int sub_center_x = center_x - width / 2 + (col * width / 3) + width / 6;
          int sub_center_y = center_y - height / 2 + (row * height / 3) + height / 6;
          int sub_index = info_msg->width * sub_center_y + sub_center_x;

          if (sub_index >= 0 && sub_index < static_cast<int>(point_cloud->points.size())) {
            if (checkNanInf(point_cloud->points[sub_index])) {
              object_point.x = point_cloud->points[sub_index].x;
              object_point.y = point_cloud->points[sub_index].y;
              object_point.z = point_cloud->points[sub_index].z;
              object_rotate.x = 0.; // Roll  // TODO
              object_rotate.y = 0.; // Pitch // TODO
              object_rotate.z = 0.; // Yaw   // TODO
              set_tf = true;
              break;
            }
          }
        }
      }

      if (set_tf) {
        geometry_msgs::msg::Pose obj_pose;
        obj_pose.position = object_point;
        obj_pose.orientation = get_quat_from_euler(object_rotate);

        vision_msgs::msg::ObjectHypothesisWithPose ohwp;
        PointT pt;
        pt.x = obj_pose.position.x; pt.y = obj_pose.position.y; pt.z = obj_pose.position.z;
        point_cloud_bbox->points.push_back(pt);

        ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
        ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
        ohwp.pose.pose = obj_pose;
        ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
        object_pose.results.push_back(ohwp);
        object_pose.bbox.center = obj_pose;
        object_pose.bbox.size.x = 2*noise_point_cloud_range_;
        object_pose.bbox.size.y = 2*noise_point_cloud_range_;
        object_pose.bbox.size.z = 2*noise_point_cloud_range_;
        object_pose.id = bbox_msg->id;
        publishObjectTf(obj_pose, bbox_msg->id);
      }

      return object_pose;
    }

    vision_msgs::msg::Detection3D processBBoxDepthImage(
          const std::shared_ptr<vision_msgs::msg::Detection2D> bbox_msg,
          const std::shared_ptr<sensor_msgs::msg::CameraInfo>  info_msg,
          const std::shared_ptr<sensor_msgs::msg::Image>       img_msg , PointCloud::Ptr& point_cloud_bbox) {

      vision_msgs::msg::Detection3D object_pose;
      object_pose.header = info_msg->header;
      object_pose.header.frame_id = base_frame_name_;

      int bytes_per_pixel = img_msg->step / img_msg->width;
      int center_x = static_cast<int>(bbox_msg->bbox.center.position.x);
      int center_y = static_cast<int>(bbox_msg->bbox.center.position.y);
      int center_index = img_msg->step * center_y + center_x * bytes_per_pixel;

      geometry_msgs::msg::Point object_point;
      geometry_msgs::msg::Point object_rotate;
      bool set_tf = false;

      if ((0 <= center_index) && (center_index < static_cast<int>(img_msg->data.size()))) {
        set_tf = true;
        if        (img_msg->encoding == "32FC1") {
          const float* data = reinterpret_cast<const float*>(&img_msg->data[center_index]);
          object_point.z = *data;
        } else if (img_msg->encoding == "16UC1") {
          const uint16_t* data = reinterpret_cast<const uint16_t*>(&img_msg->data[center_index]);
          object_point.z = static_cast<float>(*data) / 1000.;
        } else if (img_msg->encoding == "32SC1") {
          const int32_t* data = reinterpret_cast<const int32_t*>(&img_msg->data[center_index]);
          object_point.z = static_cast<float>(*data) / 1000.;
        } else set_tf = false;
      }

      if (set_tf) {
        double fx = info_msg->k[0];
        double fy = info_msg->k[4];
        double cx = info_msg->k[2];
        double cy = info_msg->k[5];

        object_point.x = (center_x - cx) * object_point.z / fx;
        object_point.y = (center_y - cy) * object_point.z / fy;

        geometry_msgs::msg::PointStamped object_point_stamped;
        object_point_stamped.header = info_msg->header;
        object_point_stamped.point = object_point;

        try {
          tfBuffer_.transform(object_point_stamped, object_point_stamped, base_frame_name_);
          object_point = object_point_stamped.point;
        } catch (tf2::TransformException &ex) {set_tf = false;}
      }

      if (set_tf) {
        object_rotate.x = 0.; // Roll  // TODO
        object_rotate.y = 0.; // Pitch // TODO
        object_rotate.z = 0.; // Yaw   // TODO

        geometry_msgs::msg::Pose obj_pose;
        obj_pose.position = object_point;
        obj_pose.orientation = get_quat_from_euler(object_rotate);

        vision_msgs::msg::ObjectHypothesisWithPose ohwp;
        PointT pt;
        pt.x = obj_pose.position.x; pt.y = obj_pose.position.y; pt.z = obj_pose.position.z;
        point_cloud_bbox->points.push_back(pt);

        ohwp.hypothesis.class_id = bbox_msg->results[0].hypothesis.class_id;
        ohwp.hypothesis.score = bbox_msg->results[0].hypothesis.score;
        ohwp.pose.pose = obj_pose;
        ohwp.pose.covariance = bbox_msg->results[0].pose.covariance;
        object_pose.results.push_back(ohwp);
        object_pose.bbox.center = obj_pose;
        object_pose.bbox.size.x = 2*noise_point_cloud_range_;
        object_pose.bbox.size.y = 2*noise_point_cloud_range_;
        object_pose.bbox.size.z = 2*noise_point_cloud_range_;
        object_pose.id = bbox_msg->id;
        publishObjectTf(obj_pose, bbox_msg->id);
      }

      return object_pose;
    }

    void callback_BBoxPointCloud(const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
                                 const std::shared_ptr<sensor_msgs::msg::PointCloud2>      pcl_msg,
                                 const std::shared_ptr<sensor_msgs::msg::CameraInfo>       info_msg) {

      PointCloud::Ptr cloud_src(new PointCloud());
      pcl::fromROSMsg(*pcl_msg, *cloud_src);
      if (!tfBuffer_.canTransform(base_frame_name_, pcl_msg->header.frame_id, pcl_msg->header.stamp)) return;
      if (!pcl_ros::transformPointCloud(base_frame_name_, *cloud_src, *cloud_src, tfBuffer_)) return;

      vision_msgs::msg::Detection3DArray object_pose_array;
      object_pose_array.header = bbox_msg->header;
      object_pose_array.header.frame_id = base_frame_name_;

      PointCloud::Ptr combined_cloud(new PointCloud());
      combined_cloud->header.frame_id = base_frame_name_;

      for (size_t i = 0; i < bbox_msg->detections.size(); i++) {

        auto detection = std::make_shared<vision_msgs::msg::Detection2D>(bbox_msg->detections[i]);
        detection->id = generateObjectId(detection->id, i);

        PointCloud::Ptr point_cloud_bbox(new PointCloud());
        point_cloud_bbox->header.frame_id = base_frame_name_;

        vision_msgs::msg::Detection3D object_pose;
        if      (positioning_detection_mode_ == "point_cloud") object_pose = processBBoxClustering(detection, info_msg, cloud_src, point_cloud_bbox);
        else if (positioning_detection_mode_ == "fast_point")  object_pose = processBBoxFastShot(  detection, info_msg, cloud_src, point_cloud_bbox);

        if (object_pose.results.size() != 0) {
          object_pose_array.detections.push_back(object_pose);
          *combined_cloud += *point_cloud_bbox;
        }
      }

      sensor_msgs::msg::PointCloud2 combined_cloud_msg;
      pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
      combined_cloud_msg.header = info_msg->header;
      combined_cloud_msg.header.frame_id = base_frame_name_;
      pub_object_cloud_->publish(combined_cloud_msg);
      pub_obj_poses_->publish(object_pose_array);
    }

    void callback_BBoxDepthImage(const std::shared_ptr<vision_msgs::msg::Detection2DArray> bbox_msg,
                                 const std::shared_ptr<sensor_msgs::msg::Image>            img_msg,
                                 const std::shared_ptr<sensor_msgs::msg::CameraInfo>       info_msg) {

      if (!tfBuffer_.canTransform(base_frame_name_, info_msg->header.frame_id, info_msg->header.stamp)) return;

      vision_msgs::msg::Detection3DArray object_pose_array;
      object_pose_array.header = info_msg->header;
      object_pose_array.header.frame_id = base_frame_name_;

      PointCloud::Ptr combined_cloud(new PointCloud());
      combined_cloud->header.frame_id = base_frame_name_;

      for (size_t i = 0; i < bbox_msg->detections.size(); i++) {

        auto detection = std::make_shared<vision_msgs::msg::Detection2D>(bbox_msg->detections[i]);
        detection->id = generateObjectId(detection->id, i);

        PointCloud::Ptr point_cloud_bbox(new PointCloud());
        point_cloud_bbox->header.frame_id = base_frame_name_;

        vision_msgs::msg::Detection3D object_pose;
        if (positioning_detection_mode_ == "depth_image") object_pose = processBBoxDepthImage(detection, info_msg, img_msg, point_cloud_bbox);

        // Introduce the data into the msg
        if (object_pose.results.size() != 0) {
          object_pose_array.detections.push_back(object_pose);
          *combined_cloud += *point_cloud_bbox;
        }
      }

      sensor_msgs::msg::PointCloud2 combined_cloud_msg;
      pcl::toROSMsg(*combined_cloud, combined_cloud_msg);
      combined_cloud_msg.header = info_msg->header;
      combined_cloud_msg.header.frame_id = base_frame_name_;

      // Publish the result of the 3D Pose estimation
      pub_object_cloud_->publish(combined_cloud_msg);
      pub_obj_poses_->publish(object_pose_array);
    }

    void callback_runctr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      if (req->data) {
        rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;
        if (!sub_bboxes_) sub_bboxes_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, bbox_topic_name_);
        if (!sub_pcl_)    sub_pcl_    = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_name_, sensor_qos_profile);
        if (!sub_img_)    sub_img_    = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic_name_);
        if (!sub_info_)   sub_info_   = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(this, info_topic_name_);

        if ((positioning_detection_mode_ == "point_cloud") || (positioning_detection_mode_ == "fast_point")) {
          if (!sync_point_cloud_) sync_point_cloud_ = std::make_shared<message_filters::Synchronizer<BBoxesCloudSyncPolicy>>(BBoxesCloudSyncPolicy(200), *sub_bboxes_, *sub_pcl_, *sub_info_);
          sync_point_cloud_->registerCallback(&BboxTo3D::callback_BBoxPointCloud, this);
        } else if (positioning_detection_mode_ == "depth_image") {
          if (!sync_depth_image_) sync_depth_image_ = std::make_shared<message_filters::Synchronizer<BBoxesDepthSyncPolicy>>(BBoxesDepthSyncPolicy(200), *sub_bboxes_, *sub_img_, *sub_info_);
          sync_depth_image_->registerCallback(&BboxTo3D::callback_BBoxDepthImage, this);
        }

      } else {
        if (sync_point_cloud_) sync_point_cloud_.reset();
        if (sync_depth_image_) sync_depth_image_.reset();
        if (sub_bboxes_) {
          sub_bboxes_->unsubscribe();
          sub_bboxes_ = nullptr;
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
    BboxTo3D() : Node("bbox_to_3d"), tfBuffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)), tfListener_(tfBuffer_), tfBroadcaster_(this), sub_bboxes_(), sub_pcl_(), sub_img_(), sub_info_(), sync_point_cloud_(), sync_depth_image_() {

      this->declare_parameter("base_frame_name", "base_footprint");
      this->declare_parameter("bbox_topic_name", "objects_rect");
      this->declare_parameter("cloud_topic_name", "dummy_pointcloud");
      this->declare_parameter("depth_image_topic_name", "dummy_image");
      this->declare_parameter("info_topic_name", "dummy_info");
      this->declare_parameter("execute_default", true);

      this->declare_parameter("cluster_tolerance", 0.01);
      this->declare_parameter("min_cluster_size", 100);
      this->declare_parameter("max_cluster_size", 20000);
      this->declare_parameter("noise_point_cloud_range", 0.01);
      this->declare_parameter("enable_id", false);
      this->declare_parameter("positioning_detection_mode", "point_cloud");


      base_frame_name_ = this->get_parameter("base_frame_name").as_string();
      bbox_topic_name_ = this->get_parameter("bbox_topic_name").as_string();
      cloud_topic_name_ = this->get_parameter("cloud_topic_name").as_string();
      depth_topic_name_ = this->get_parameter("depth_image_topic_name").as_string();
      info_topic_name_ = this->get_parameter("info_topic_name").as_string();

      noise_point_cloud_range_ = this->get_parameter("noise_point_cloud_range").as_double();
      enable_id_ = this->get_parameter("enable_id").as_bool();
      positioning_detection_mode_ = this->get_parameter("positioning_detection_mode").as_string(); // "point_cloud", "depth_image", "fast_point"

      kdtree_.reset(new pcl::search::KdTree<PointT>);
      euclid_clustering_.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double());
      euclid_clustering_.setMinClusterSize(this->get_parameter("min_cluster_size").as_int());
      euclid_clustering_.setMaxClusterSize(this->get_parameter("max_cluster_size").as_int());
      euclid_clustering_.setSearchMethod(kdtree_);

      // ROS publishers
      pub_obj_poses_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_3d_poses", 5);
      pub_object_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_3d_cloud", 1);

      // ROS service server
      run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>("position/run_ctr", std::bind(&BboxTo3D::callback_runctr, this, std::placeholders::_1, std::placeholders::_2));

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      request->data = this->get_parameter("execute_default").as_bool();
      callback_runctr(request, response);
    }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BboxTo3D>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}