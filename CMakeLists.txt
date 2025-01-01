cmake_minimum_required(VERSION 3.8)
project(image_to_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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


#--- for C++
include_directories(include)
add_executable(bbox_to_3d src/bbox_to_3d.cpp)
ament_target_dependencies(bbox_to_3d rclcpp std_msgs std_srvs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros vision_msgs)
add_executable(keypoint_to_3d src/keypoint_to_3d.cpp)
ament_target_dependencies(keypoint_to_3d rclcpp std_msgs std_srvs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_interfaces)
# add_executable(bbox_to_tf_fast_shot src/bbox_to_tf_fast_shot.cpp)
# ament_target_dependencies(bbox_to_tf_fast_shot rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_interfaces)
# add_executable(nearest_person_tf src/nearest_person_tf.cpp)
# ament_target_dependencies(nearest_person_tf rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_interfaces)

install(TARGETS
  bbox_to_3d
  keypoint_to_3d
  # bbox_to_tf_fast_shot
  # nearest_person_tf
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch

#-- for Config
install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config)
#--- end Config



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()