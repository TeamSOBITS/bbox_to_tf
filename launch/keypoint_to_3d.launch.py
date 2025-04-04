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

    keypoints_topic_name = LaunchConfiguration("keypoints_topic_name")
    keypoints_topic_name_cmd = DeclareLaunchArgument(
        "keypoints_topic_name",
        description="Key Point topic name",
        default_value="objects_rect",
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
                "img_topic_name": img_topic_name,
                "execute_default": execute_default,
            }
        ]
    )

    return LaunchDescription([
        namespace_cmd,
        base_frame_name_cmd,
        keypoints_topic_name_cmd,
        cloud_topic_name_cmd,
        img_topic_name_cmd,
        execute_default_cmd,
        keypoint_to_3d_cmd,
    ])
