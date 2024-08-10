from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bbox_to_tf',
            executable='bbox_to_tf',
            name='bbox_to_tf',
            output='screen',
            # parameters=[{
            #     'base_frame_name': 'base_footprint',
            #     'bbox_topic_name': 'objects_rect',
            #     # 'cloud_topic_name': '/points2',
            #     'cloud_topic_name': '/camera/camera/depth/color/points',
            #     'img_topic_name': '/rgb/image_raw',
            #     'execute_default': True,
            #     'cluster_tolerance': 0.01,
            #     'min_clusterSize': 100,
            #     'max_clusterSize': 20000,
            #     'noise_point_cloud_range': 0.01,
            # }]
            parameters=[{
                'base_frame_name': 'camera_link',
                'bbox_topic_name': '/yolov8/objects_rect',
                'cloud_topic_name': '/camera/camera/depth/color/points',
                'img_topic_name': '/camera/camera/color/image_raw',
                'execute_default': True,
                'cluster_tolerance': 0.01,
                'min_clusterSize': 100,
                'max_clusterSize': 20000,
                'noise_point_cloud_range': 0.01,
            }]
        )
    ])