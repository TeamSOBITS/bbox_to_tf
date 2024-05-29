<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# bbox_to_tf

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#Introduction">Introduction</a>
    </li>
    <li>
      <a href="#Getting Started">Getting Started</a>
      <ul>
        <li><a href="#Requirements">Requirements</a></li>
        <li><a href="#Installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#Composition of launch file">Composition of launch file</a></li>
    <li><a href="#Milestone">Milestone</a></li>
    <li><a href="#Acknowledgements">Acknowledgements</a></li>
  </ol>
</details>


<!-- レポジトリの概要 -->
## Introduction

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

This package converts the BoundingBox detected by image recognition into 3D coordinates (TF) using a point cloud.\
Basically, the BoundingBoxes detected by the RGB-D camera (sobits_msgs/BoundingBoxes), their images (sensor_msgs/Image), and the point clouds from that camera (sensor_msgs/PointCloud2) are used for 3D.

> [!NOTE]
> This package is basically used after image processing and object recognition, so it is installed by other packages.



<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- セットアップ -->
## Getting Started

This section describes how to set up this repository.\
Write this into the install.sh of the package on which it depends.

### Requirements



| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation

1. Change directory
   ```sh
   $ cd ~/catkin_ws/src
   ```
2. clone TeamSOBITS/human_feature_detect
   ```sh
   $ git clone https://github.com/TeamSOBITS/bbox_to_tf.git
   ```
3. compile
   ```sh
   $ cd ~/catkin_ws/
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- 実行・操作方法 -->
## Composition of launch file

This package is basically not launched by itself, but [bbox_to_tf.launch](/launch/bbox_to_tf.launch) is written as an include from the launch of the necessary image processing package.\
Therefore, this section describes the contents and structure of [bbox_to_tf.launch](/launch/bbox_to_tf.launch).

- node_name\
  node_name is the name of the node to be 3Dized.\
  Since there may not be a single image recognition node that you want to 3Dize, you can specify the node name to avoid conflicts even if there are multiple nodes.
  For example, if you want to convert image recognition by yolov8 and ssd to TF respectively, you can specify yolov8_to_tf_node and ssd_to_tf_node of each node_name to avoid conflicts.

- base_frame_name\
  base_frame_name is the base frame from which the coordinates are obtained in 3D.\
  If the coordinates of the robot are used as the base frame, base_footprint is used.\
  The 3D coordinates of each recognized object are generated using the center of the robot's feet as the origin (0,0,0).

- bbox_topic_name\
  bbox_topic_name is the name of the BoundingBox topic.
  Specifically, it specifies the name of the topic on which messages of type sobits_msgs/BoundingBoxes are flying.\
  This is a custom ROS message created by SOBITS on its own, so it is necessary to have sobits_msgs git clone.\
  However, it should already be git cloned in the install.sh of the packages that depend on this package.

- cloud_topic_name\
  cloud_topic_name is the topic name of the point cloud.
  Specifically, it specifies the name of the topic on which messages of type sensor_msgs/PointCloud2 are flying.\
  Based on the information in the BoundingBox, a point cloud is flown to the area.
  At that time, the position is obtained by classifying the point cloud as well as the distance of the object.

- img_topic_name\
  img_topic_name is the topic name of the image for which this image recognition is performed.\
  Specifically, it specifies the name of the topic on which messages of type sensor_msgs/Image are flying.\
  It is referenced to see the relationship between the image and the point cloud.

- execute_default\
  execute_default is whether or not TF is issued by default.\
  Even if it is OFF (false), it can be also turned ON (true) with run_ctr when it is used.

- cluster_tolerance\
  This is a threshold value that determines how close a group of points is considered to be the same object.\
  When a point cloud is flown into the BoundingBox, it is classified as a single object if the point cloud hits the object and is within the threshold.
  Therefore, if the threshold is too large, the search area for each point cloud will increase and processing will become slower.

- min_clusterSize\
  This is a threshold value that determines how many or fewer points should be rejected from the object point cloud.\
  When a point cloud is classified, if the number of points is less than or equal to this threshold, it is considered as noise and rejected.

- max_lusterSize\
  Threshold for how many or more point clouds should be rejected from the object's point cloud.\
  If the number of points in a point cloud is greater than this threshold, it is considered to be a completely different object (e.g., a floor point cloud for an object) and is rejected.

- noise_point_cloud_range\
  The amount of removal to remove noise surfaces from the object point cloud and bring it closer to the center coordinates.
  After extracting the object from the classified point cloud, the point cloud is further cut in the x, y, and z directions by this value, including the floor, back wall, and left and right walls.
  In this way, the point cloud can be reduced to a point cloud that covers only a small portion of the object.
  However, be careful not to increase the value too much, as this will remove many points from the object.


> [!NOTE]
> Basically, instead of modifying [bbox_to_tf.launch](/launch/bbox_to_tf.launch), please avoid conflicts by using include in the launch file of packages that depend on this package.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Services
The function can be turned ON/OFF(True/False) as run_ctr.
The naming is dependent on the node_name so that Topic names do not conflict.
- ON/OFF
  ```sh
  node_name + "/run_ctr" (Service: sobits_msgs/RunCtrl)
  ```

### Topic
Coordinates and point clouds viewed from base_frame_name are Publish.
The naming is dependent on the node_name so that Topic names do not conflict.
- coordinate of objects
  ```sh
  node_name + "/object_poses" (Topic: sobits_msgs::ObjectPoseArray)
  ```

- point cloud of objects
  ```sh
  node_name + "/object_poses" (Topic: pcl/PointCloud(pcl/PointXYZ))
  ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- マイルストーン -->
## Milestone

See the [open issues](issues-url) for a full list of proposed features (and known issues).


<!-- 参考文献 -->
## Acknowledgements

* [ROS Noetic](http://wiki.ros.org/noetic)

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/bbox_to_tf/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/bbox_to_tf/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/bbox_to_tf/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/bbox_to_tf/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[license-url]: LICENSE

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
<!-- []:  -->
