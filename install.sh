#!/bin/bash

echo "╔══╣ Install: BBox to TF (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`


# Clone required packages
cd ..
git clone -b feature/humble-devel https://github.com/TeamSOBITS/sobits_interfaces.git

# Install common dependencies
python3 -m pip install -U pip
python3 -m pip install \
    matplotlib \

sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-message-filters


# Go back to previous directory
cd ${DIR}


echo "╚══╣ Install: BBox to TF (FINISHED) ╠══╝"
