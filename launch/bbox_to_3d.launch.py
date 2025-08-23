from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name",
        description="Base frame name for the node (String)",
        default_value="camera_base",
    )

    bbox_topic_name = LaunchConfiguration("bbox_topic_name")
    bbox_topic_name_cmd = DeclareLaunchArgument(
        "bbox_topic_name",
        description="Bounding box topic name. (vision_msgs/msg/Detection2DArray)",
        default_value="/objects_rect",
    )

    cloud_topic_name = LaunchConfiguration("cloud_topic_name")
    cloud_topic_name_cmd = DeclareLaunchArgument(
        "cloud_topic_name",
        description="Point cloud topic name. (sensor_msgs/msg/PointCloud2)",
        default_value="/point_cloud",
    )

    depth_image_topic_name = LaunchConfiguration("depth_image_topic_name")
    depth_image_topic_name_cmd = DeclareLaunchArgument(
        "depth_image_topic_name",
        description="Image topic name. (sensor_msgs/msg/Image)",
        default_value="/depth_image_raw",
    )

    info_topic_name = LaunchConfiguration("info_topic_name")
    info_topic_name_cmd = DeclareLaunchArgument(
        "info_topic_name",
        description="Camera Infomations topic name. (sensor_msgs/msg/CameraInfo)",
        default_value="/camera_info",
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default",
        description="Execute default behavior (Bool)",
        default_value="true",
    )

    cluster_tolerance = LaunchConfiguration("cluster_tolerance")
    cluster_tolerance_cmd = DeclareLaunchArgument(
        "cluster_tolerance",
        description="Tolerance for clustering (Float)",
        default_value="0.01",
    )

    min_cluster_size = LaunchConfiguration("min_clusterSize")
    min_cluster_size_cmd = DeclareLaunchArgument(
        "min_clusterSize",
        description="Minimum cluster size (Int)",
        default_value="100",
    )

    max_cluster_size = LaunchConfiguration("max_clusterSize")
    max_cluster_size_cmd = DeclareLaunchArgument(
        "max_clusterSize",
        description="Maximum cluster size (Int)",
        default_value="20000",
    )

    noise_point_cloud_range = LaunchConfiguration("noise_point_cloud_range")
    noise_point_cloud_range_cmd = DeclareLaunchArgument(
        "noise_point_cloud_range",
        description="Range for noise filtering in the point cloud (Float)",
        default_value="0.01",
    )

    enable_id = LaunchConfiguration("enable_id")
    enable_id_cmd = DeclareLaunchArgument(
        "enable_id",
        description="Enable assigning IDs to detected objects (Bool)",
        default_value="false",
    )

    positioning_detection_mode = LaunchConfiguration("positioning_detection_mode")
    positioning_detection_mode_cmd = DeclareLaunchArgument(
        "positioning_detection_mode",
        description="method of 2D to 3D position. choose of ['point_cloud', 'depth_image']",
        default_value="point_cloud",
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
                "depth_image_topic_name": depth_image_topic_name,
                "info_topic_name": info_topic_name,
                "execute_default": execute_default,
                "cluster_tolerance": cluster_tolerance,
                "min_clusterSize": min_cluster_size,
                "max_clusterSize": max_cluster_size,
                "noise_point_cloud_range": noise_point_cloud_range,
                "enable_id": enable_id,
                "positioning_detection_mode": positioning_detection_mode,
            }
        ]
    )

    return LaunchDescription([
        namespace_cmd,
        base_frame_name_cmd,
        bbox_topic_name_cmd,
        cloud_topic_name_cmd,
        depth_image_topic_name_cmd,
        info_topic_name_cmd,
        execute_default_cmd,
        cluster_tolerance_cmd,
        min_cluster_size_cmd,
        max_cluster_size_cmd,
        noise_point_cloud_range_cmd,
        enable_id_cmd,
        positioning_detection_mode_cmd,
        bbox_to_3d_cmd,
    ])