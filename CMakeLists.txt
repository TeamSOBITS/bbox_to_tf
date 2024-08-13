cmake_minimum_required(VERSION 3.5)
project(bbox_to_tf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(sobits_msgs REQUIRED)

add_executable(bbox_to_tf src/bbox_to_tf.cpp)

ament_target_dependencies(bbox_to_tf 
  rclcpp 
  sensor_msgs 
  std_msgs 
  tf2 
  tf2_ros 
  # cv_bridge 
  pcl_ros
  pcl_conversions
  # PCL
  # OpenCV
  sobits_msgs
)

install(TARGETS
  bbox_to_tf
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()