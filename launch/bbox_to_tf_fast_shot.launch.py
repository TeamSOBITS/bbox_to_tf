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
        description="Base frame name for the TF",
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

    bbox_to_tf_fast_shot_cmd = Node(
        package='image_to_position',
        executable='bbox_to_tf_fast_shot',
        name='bbox_to_tf_fast_shot',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                "base_frame_name": base_frame_name,
                "bbox_topic_name": bbox_topic_name,
                "cloud_topic_name": cloud_topic_name,
                "img_topic_name": img_topic_name,
            }
        ]
    )

    return LaunchDescription([
        namespace_cmd,
        base_frame_name_cmd,
        bbox_topic_name_cmd,
        cloud_topic_name_cmd,
        img_topic_name_cmd,
        bbox_to_tf_fast_shot_cmd,
    ])