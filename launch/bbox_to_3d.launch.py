from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the nodes"
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name",
        description="Base frame name for the node",
        default_value="camera_base",
    )

    bbox_topic_name = LaunchConfiguration("bbox_topic_name")
    bbox_topic_name_cmd = DeclareLaunchArgument(
        "bbox_topic_name",
        description="Bounding box topic name",
        default_value="/yolo_ros/object_boxes",
    )

    cloud_topic_name = LaunchConfiguration("cloud_topic_name")
    cloud_topic_name_cmd = DeclareLaunchArgument(
        "cloud_topic_name",
        description="Point cloud topic name",
        default_value="/points2",
    )

    img_topic_name = LaunchConfiguration("img_topic_name")
    img_topic_name_cmd = DeclareLaunchArgument(
        "img_topic_name",
        description="Image topic name",
        default_value="/rgb/image_raw",
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default",
        description="Execute default behavior",
        default_value="true",
    )

    cluster_tolerance = LaunchConfiguration("cluster_tolerance")
    cluster_tolerance_cmd = DeclareLaunchArgument(
        "cluster_tolerance",
        description="Tolerance for clustering",
        default_value="0.01",
    )

    min_cluster_size = LaunchConfiguration("min_clusterSize")
    min_cluster_size_cmd = DeclareLaunchArgument(
        "min_clusterSize",
        description="Minimum cluster size",
        default_value="100",
    )

    max_cluster_size = LaunchConfiguration("max_clusterSize")
    max_cluster_size_cmd = DeclareLaunchArgument(
        "max_clusterSize",
        description="Maximum cluster size",
        default_value="20000",
    )

    noise_point_cloud_range = LaunchConfiguration("noise_point_cloud_range")
    noise_point_cloud_range_cmd = DeclareLaunchArgument(
        "noise_point_cloud_range",
        description="Range for noise filtering in the point cloud",
        default_value="0.01",
    )

    bbox_to_3d_cmd = Node(
        package='image_to_position',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                "base_frame_name": base_frame_name,
                "bbox_topic_name": bbox_topic_name,
                "cloud_topic_name": cloud_topic_name,
                "img_topic_name": img_topic_name,
                "execute_default": execute_default,
                "cluster_tolerance": cluster_tolerance,
                "min_clusterSize": min_cluster_size,
                "max_clusterSize": max_cluster_size,
                "noise_point_cloud_range": noise_point_cloud_range,
            }
        ]
    )

    return LaunchDescription([
        namespace_cmd,
        base_frame_name_cmd,
        bbox_topic_name_cmd,
        cloud_topic_name_cmd,
        img_topic_name_cmd,
        execute_default_cmd,
        cluster_tolerance_cmd,
        min_cluster_size_cmd,
        max_cluster_size_cmd,
        noise_point_cloud_range_cmd,
        bbox_to_3d_cmd,
    ])
