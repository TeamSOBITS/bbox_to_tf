<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# image_to_position

## Introduction

This package converts detection results (BoundingBoxes, Keypoints, Masks) into 3D coordinates (TF) by leveraging point cloud data.

Using point clouds from RGB-D cameras (sensor_msgs/PointCloud2) and image recognition results, this package estimates the 3D position of detected objects or keypoints. 
This package is implemented as **ROS 2 Lifecycle Nodes**, following standard ROS 2 lifecycle state machine management.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Getting Started

This section describes how to set up this repository.\

### Requirements

Please, make sure that you have the folloing environment ready.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Python | 3.12 |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Change directory
   ```sh
   $ cd ~/colcon_ws/src
   ```
2. Clone the repository
   ```sh
   $ git clone https://github.com/TeamSOBITS/image_to_position.git
   ```
3. Install the required dependencies.
   ```sh
   $ cd image_to_position/
   $ bash install.sh
   ```
4. Build the package:
   ```sh
   cd ~/colcon_ws/
   colcon build --symlink-install
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Usage
This package uses the ROS 2 Lifecycle system. After launching, nodes start in the `unconfigured` state. You must transition them to `active` to begin processing.

### Lifecycle Management
Example for `bbox_to_3d` node:
```sh
# Start the node
ros2 run image_to_position bbox_to_3d

# Check state
ros2 lifecycle get /bbox_to_3d

# Configure
ros2 lifecycle set /bbox_to_3d configure

# Activate
ros2 lifecycle set /bbox_to_3d activate
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Parameter Configuration (Spatial Clipping)
For improved performance, you should setup your configuration files located in the folder [config](./config/).


- `base_frame_name`:\
  base_frame_name is the base frame from which the coordinates are obtained in 3D.\
  If the coordinates of the robot are used as the base frame, base_footprint is used.\
  The 3D coordinates of each recognized object are generated using the center of the robot's feet as the origin (0,0,0).

- `bbox_topic_name`:\
  bbox_topic_name is the name of the BoundingBox topic.
  Specifically, it specifies the name of the topic on which messages of type sobits_interfaces/BoundingBoxes are flying.\
  This is a custom ROS message created by SOBITS on its own, so it is necessary to have sobits_interfaces git clone.\
  However, it should already be git cloned in the install.sh of the packages that depend on this package.

- `cloud_topic_name`:\
  cloud_topic_name is the topic name of the point cloud.
  Specifically, it specifies the name of the topic on which messages of type sensor_msgs/PointCloud2 are flying.\
  Based on the information in the BoundingBox, a point cloud is flown to the area.
  At that time, the position is obtained by classifying the point cloud as well as the distance of the object.

- `img_topic_name`:\
  img_topic_name is the topic name of the image for which this image recognition is performed.\
  Specifically, it specifies the name of the topic on which messages of type sensor_msgs/Image are flying.\
  It is referenced to see the relationship between the image and the point cloud.

- `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`:\
  Define the valid spatial range in the camera optical frame (in meters). This ignores background noise (walls, floors) and drastically improves processing speed.

- `cluster_tolerance`:\
  This is a threshold value that determines how close a group of points is considered to be the same object.\
  When a point cloud is flown into the BoundingBox, it is classified as a single object if the point cloud hits the object and is within the threshold.
  Therefore, if the threshold is too large, the search area for each point cloud will increase and processing will become slower.

- `min_clusterSize`:\
  This is a threshold value that determines how many or fewer points should be rejected from the object point cloud.\
  When a point cloud is classified, if the number of points is less than or equal to this threshold, it is considered as noise and rejected.

- `max_lusterSize`:\
  Threshold for how many or more point clouds should be rejected from the object's point cloud.\
  If the number of points in a point cloud is greater than this threshold, it is considered to be a completely different object (e.g., a floor point cloud for an object) and is rejected.

- `noise_point_cloud_range`:\
  The amount of removal to remove noise surfaces from the object point cloud and bring it closer to the center coordinates.
  After extracting the object from the classified point cloud, the point cloud is further cut in the x, y, and z directions by this value, including the floor, back wall, and left and right walls.
  In this way, the point cloud can be reduced to a point cloud that covers only a small portion of the object.
  However, be careful not to increase the value too much, as this will remove many points from the object.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## References

* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [Point Cloud Library (PCL)](https://pointclouds.org/)
* [Perception PCL](https://github.com/ros-perception/perception_pcl)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/image_to_position.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/image_to_position/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/image_to_position.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/image_to_position/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/image_to_position.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/image_to_position/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/image_to_position.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/image_to_position/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/image_to_position.svg?style=for-the-badge
[license-url]: LICENSE
