from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    node_name = LaunchConfiguration("node_name")
    node_name_cmd = DeclareLaunchArgument(
        "node_name", default_value="nearest_person_tf", description="Name of the node"
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name",
        description="Base frame name for the TF",
        default_value="camera_link",
    )

    bbox_topic_name = LaunchConfiguration("bbox_topic_name")
    bbox_topic_name_cmd = DeclareLaunchArgument(
        "bbox_topic_name",
        description="Bounding box topic name",
        default_value="/yolov8/objects_rect",
    )

    cloud_topic_name = LaunchConfiguration("cloud_topic_name")
    cloud_topic_name_cmd = DeclareLaunchArgument(
        "cloud_topic_name",
        description="Point cloud topic name",
        default_value="/camera/camera/depth/color/points",
    )

    img_topic_name = LaunchConfiguration("img_topic_name")
    img_topic_name_cmd = DeclareLaunchArgument(
        "img_topic_name",
        description="Image topic name",
        default_value="/camera/camera/color/image_raw",
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default", default_value="true", description="ON/OFF (bool)"
    )

    only_specific_object = LaunchConfiguration("only_specific_object")
    only_specific_object_cmd = DeclareLaunchArgument(
        "only_specific_object", default_value="true", description="publish only specific object (bool)"
    )

    cluster_tolerance = LaunchConfiguration("cluster_tolerance")
    cluster_tolerance_cmd = DeclareLaunchArgument(
        "cluster_tolerance", default_value="0.008", description="same point cloud range"
    )

    min_clusterSize = LaunchConfiguration("min_clusterSize")
    min_clusterSize_cmd = DeclareLaunchArgument(
        "min_clusterSize", default_value="10", description="clustering point cloud lower number of pieces"
    )

    max_clusterSize = LaunchConfiguration("max_clusterSize")
    max_clusterSize_cmd = DeclareLaunchArgument(
        "max_clusterSize", default_value="2000000", description="clustering point cloud upper number of pieces"
    )

    noise_point_cloud_range = LaunchConfiguration("noise_point_cloud_range")
    noise_point_cloud_range_cmd = DeclareLaunchArgument(
        "noise_point_cloud_range", default_value="0.01", description="limiter of object point cloud cut noise"
    )

    rviz = LaunchConfiguration("rviz")
    rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz2"
    )

    bbox_to_tf_node = Node(
        package="bbox_to_tf",
        executable="nearest_person_tf",
        name=node_name,
        output="screen",
        parameters=[
            {
                "node_name": node_name,
                "base_frame_name": base_frame_name,
                "bbox_topic_name": bbox_topic_name,
                "cloud_topic_name": cloud_topic_name,
                "img_topic_name": img_topic_name,
                "execute_default": execute_default,
                "cluster_tolerance": cluster_tolerance,
                "min_clusterSize": min_clusterSize,
                "max_clusterSize": max_clusterSize,
                "noise_point_cloud_range": noise_point_cloud_range,
                "only_specific_object": only_specific_object,
            }
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("bbox_to_tf"), "config", "bbox_to_tf.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        condition=launch.conditions.IfCondition(rviz),
    )

    return LaunchDescription(
        [
            node_name_cmd,
            base_frame_name_cmd,
            bbox_topic_name_cmd,
            cloud_topic_name_cmd,
            img_topic_name_cmd,
            execute_default_cmd,
            cluster_tolerance_cmd,
            min_clusterSize_cmd,
            max_clusterSize_cmd,
            noise_point_cloud_range_cmd,
            rviz_cmd,
            only_specific_object_cmd,
            bbox_to_tf_node,
            rviz_node,
        ]
    )