cmake_minimum_required(VERSION 3.8)
project(image_to_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

add_executable(bbox_to_3d src/bbox_to_3d.cpp)
ament_target_dependencies(bbox_to_3d rclcpp std_msgs std_srvs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros vision_msgs)

add_executable(keypoint_to_3d src/keypoint_to_3d.cpp)
ament_target_dependencies(keypoint_to_3d rclcpp std_msgs std_srvs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_interfaces)

add_executable(bbox_to_tf_fast_shot src/bbox_to_tf_fast_shot.cpp)
ament_target_dependencies(bbox_to_tf_fast_shot rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros vision_msgs)

install(TARGETS
  bbox_to_3d
  keypoint_to_3d
  bbox_to_tf_fast_shot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()