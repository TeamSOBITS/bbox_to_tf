cmake_minimum_required(VERSION 3.8)
project(image_to_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sobits_interfaces REQUIRED)

include_directories(include)

# TODO: update to component
add_executable(bbox_to_3d src/bbox_to_3d.cpp)
ament_target_dependencies(bbox_to_3d
  "rclcpp"
  "pcl_conversions"
  "pcl_ros"
  "tf2_ros"
  "vision_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "sobits_interfaces")

# TODO: update to component
add_executable(keypoint_to_3d src/keypoint_to_3d.cpp)
ament_target_dependencies(keypoint_to_3d
  "rclcpp"
  "pcl_conversions"
  "pcl_ros"
  "tf2_ros"
  "vision_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "std_srvs"
  "sobits_interfaces")

install(TARGETS bbox_to_3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS keypoint_to_3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
