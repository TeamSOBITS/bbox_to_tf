import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory('image_to_position')
    default_params_file = os.path.join(pkg_dir, 'config', 'mask_to_3d.yaml')
    
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    container = ComposableNodeContainer(
        name='mask_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::MaskTo3D',
                name='mask_to_3d',
                namespace=namespace,
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/mask_to_3d' if '", namespace, "' else '/mask_to_3d'",
    ])

    configure_node = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', node_full_path, 'configure'],
        output='screen'
    )


    return LaunchDescription([
        params_file_arg,
        namespace_cmd,
        container,
        TimerAction(period=0.5, actions=[configure_node]),
    ])
