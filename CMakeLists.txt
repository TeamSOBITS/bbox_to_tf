cmake_minimum_required(VERSION 3.8)
project(image_to_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sobits_interfaces REQUIRED)


include_directories(include)

# Component: Mask to 3D 
add_library(mask_to_3d_component SHARED src/mask_to_3d.cpp)
target_include_directories(mask_to_3d_component PRIVATE include)
ament_target_dependencies(mask_to_3d_component
  "rclcpp" "rclcpp_components" "pcl_conversions" "pcl_ros" "tf2_ros"
  "vision_msgs" "sensor_msgs" "geometry_msgs" "std_msgs" "std_srvs" "sobits_interfaces")
rclcpp_components_register_node(mask_to_3d_component
  PLUGIN "image_to_position::MaskTo3D"
  EXECUTABLE mask_to_3d)

# Component: Bbox to 3D 
add_library(bbox_to_3d_component SHARED src/bbox_to_3d.cpp)
target_include_directories(bbox_to_3d_component PRIVATE include)
ament_target_dependencies(bbox_to_3d_component
  "rclcpp" "rclcpp_components" "pcl_conversions" "pcl_ros" "tf2_ros"
  "vision_msgs" "sensor_msgs" "geometry_msgs" "std_msgs" "std_srvs" "sobits_interfaces")
rclcpp_components_register_node(bbox_to_3d_component
  PLUGIN "image_to_position::BboxTo3D"
  EXECUTABLE bbox_to_3d)

# Component: Keypoint to 3D 
add_library(keypoint_to_3d_component SHARED src/keypoint_to_3d.cpp)
target_include_directories(keypoint_to_3d_component PRIVATE include)
ament_target_dependencies(keypoint_to_3d_component
  "rclcpp" "rclcpp_components" "pcl_conversions" "pcl_ros" "tf2_ros"
  "vision_msgs" "sensor_msgs" "geometry_msgs" "std_msgs" "std_srvs" "sobits_interfaces")
rclcpp_components_register_node(keypoint_to_3d_component
  PLUGIN "image_to_position::KeyTo3D"
  EXECUTABLE keypoint_to_3d)


install(TARGETS
  mask_to_3d_component
  bbox_to_3d_component
  keypoint_to_3d_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  rviz
  config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
