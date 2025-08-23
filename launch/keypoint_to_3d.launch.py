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

    keypoints_topic_name = LaunchConfiguration("keypoints_topic_name")
    keypoints_topic_name_cmd = DeclareLaunchArgument(
        "keypoints_topic_name",
        description="Key Point topic name. (sobits_interfaces/msg/KeyPointArray)",
        default_value="keys_points",
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
        description="Image topic name",
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

    enable_id = LaunchConfiguration("enable_id")
    enable_id_cmd = DeclareLaunchArgument(
        "enable_id",
        description="Enable assigning IDs to detected objects (Bool)",
        default_value="false",
    )

    positioning_detection_mode = LaunchConfiguration("positioning_detection_mode")
    positioning_detection_mode_cmd = DeclareLaunchArgument(
        "positioning_detection_mode",
        description="method of 2D to 3D position. choose of ['point_cloud', 'depth_image', 'fast_point']",
        default_value="point_cloud",
    )

    keypoint_to_3d_cmd = Node(
        package='image_to_position',
        executable='keypoint_to_3d',
        name='keypoint_to_3d',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                "base_frame_name": base_frame_name,
                "keypoints_topic_name": keypoints_topic_name,
                "cloud_topic_name": cloud_topic_name,
                "depth_image_topic_name": depth_image_topic_name,
                "info_topic_name": info_topic_name,
                "execute_default": execute_default,
                "enable_id": enable_id,
                "positioning_detection_mode": positioning_detection_mode,
            }
        ]
    )

    return LaunchDescription([
        namespace_cmd,
        base_frame_name_cmd,
        keypoints_topic_name_cmd,
        cloud_topic_name_cmd,
        depth_image_topic_name_cmd,
        info_topic_name_cmd,
        execute_default_cmd,
        enable_id_cmd,
        positioning_detection_mode_cmd,
        keypoint_to_3d_cmd,
    ])
