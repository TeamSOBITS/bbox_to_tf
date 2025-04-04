from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='bbox_to_tf_fast_shot',
        description='Name of the bbox_to_tf node'
    )

    base_frame_name_arg = DeclareLaunchArgument(
        'base_frame_name',
        default_value='camera_base',
        description='Base frame name'
    )

    bbox_topic_name_arg = DeclareLaunchArgument(
        'bbox_topic_name',
        default_value='/yolo_ros/object_boxes',
        description='Topic name for BoundingBoxes (vision_msgs/Detection2DArray)'
    )

    rgb_camera_info_topic_name_arg = DeclareLaunchArgument(
        'rgb_camera_info_topic_name',
        default_value='/rgb/camera_info',
        description='Topic name for RGB CameraInfo (sensor_msgs/CameraInfo)'
    )

    depth_camera_info_topic_name_arg = DeclareLaunchArgument(
        'depth_camera_info_topic_name',
        default_value='/depth/camera_info',
        description='Topic name for Depth CameraInfo (sensor_msgs/CameraInfo)'
    )

    depth_image_topic_name_arg = DeclareLaunchArgument(
        'depth_image_topic_name',
        default_value='/depth/image_raw',
        description='Topic name for Depth Image (sensor_msgs/Image)'
    )

    average_range_arg = DeclareLaunchArgument(
        'average_range',
        default_value='1',
        description='Range for averaging depth points (1 for single point, 2 for 3x3, 3 for 5x5, etc.)'
    )

    execute_default_arg = DeclareLaunchArgument(
        'execute_default',
        default_value='true',
        description='ON/OFF (bool)'
    )

    cluster_tolerance_arg = DeclareLaunchArgument(
        'cluster_tolerance',
        default_value='0.008',
        description='Same point cloud range'
    )

    min_clusterSize_arg = DeclareLaunchArgument(
        'min_clusterSize',
        default_value='10',
        description='Clustering point cloud lower number of pieces'
    )

    max_clusterSize_arg = DeclareLaunchArgument(
        'max_clusterSize',
        default_value='2000000',
        description='Clustering point cloud upper number of pieces'
    )

    noise_point_cloud_range_arg = DeclareLaunchArgument(
        'noise_point_cloud_range',
        default_value='0.01',
        description='Limiter of object point cloud cut noise'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2'
    )

    bbox_to_tf_fast_shot_node = Node(
        package='image_to_position',
        executable='bbox_to_tf_fast_shot',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[{
            'node_name': LaunchConfiguration('node_name'),
            'base_frame_name': LaunchConfiguration('base_frame_name'),
            'bbox_topic_name': LaunchConfiguration('bbox_topic_name'),
            'rgb_camera_info_topic_name': LaunchConfiguration('rgb_camera_info_topic_name'),
            'depth_camera_info_topic_name': LaunchConfiguration('depth_camera_info_topic_name'),
            'depth_image_topic_name': LaunchConfiguration('depth_image_topic_name'),
            'average_range': LaunchConfiguration('average_range'),
            'execute_default': LaunchConfiguration('execute_default'),
            'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
            'min_clusterSize': LaunchConfiguration('min_clusterSize'),
            'max_clusterSize': LaunchConfiguration('max_clusterSize'),
            'noise_point_cloud_range': LaunchConfiguration('noise_point_cloud_range'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('image_to_position'), 'config', 'bbox_to_tf.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        node_name_arg,
        base_frame_name_arg,
        bbox_topic_name_arg,
        rgb_camera_info_topic_name_arg,
        depth_camera_info_topic_name_arg,
        depth_image_topic_name_arg,
        average_range_arg,
        execute_default_arg,
        cluster_tolerance_arg,
        min_clusterSize_arg,
        max_clusterSize_arg,
        noise_point_cloud_range_arg,
        rviz_arg,
        bbox_to_tf_fast_shot_node,
        rviz_node,
    ])