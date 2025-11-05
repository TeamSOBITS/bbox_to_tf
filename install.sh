#!/bin/bash

echo "╔══╣ Install: BBox to TF (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`


# Clone required packages
cd ..
git clone -b ${ROS_DISTRO}-devel https://github.com/TeamSOBITS/sobits_interfaces.git

# Install common dependencies
python3 -m pip install --break-system-packages -U pip
python3 -m pip install --break-system-packages\
    matplotlib 

sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Install: BBox to TF (FINISHED) ╠══╝"
