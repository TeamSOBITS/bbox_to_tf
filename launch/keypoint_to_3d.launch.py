import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, OrSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory('image_to_position')
    default_params_file = os.path.join(pkg_dir, 'config', 'keypoint_to_3d.yaml')
    
    params_file = LaunchConfiguration('params_file')
    auto_configure = LaunchConfiguration('auto_configure')
    auto_activate = LaunchConfiguration('auto_activate')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    auto_configure_arg = DeclareLaunchArgument(
        'auto_configure',
        default_value='False',
        description='Whether to configure the lifecycle node on startup'
    )
    auto_activate_arg = DeclareLaunchArgument(
        'auto_activate',
        default_value='False',
        description='Whether to activate the lifecycle node on startup'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    container = ComposableNodeContainer(
        name='keypoint_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::KeypointTo3D',
                name='keypoint_to_3d',
                namespace=namespace,
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/keypoint_to_3d' if '", namespace, "' else '/keypoint_to_3d'",
    ])

    configure_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" >/dev/null 2>&1; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" configure',
            node_full_path,
        ],
        output='screen'
    )
    activate_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" 2>/dev/null | grep -q "inactive"; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" activate',
            node_full_path,
        ],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        auto_configure_arg,
        auto_activate_arg,
        namespace_cmd,
        container,
        TimerAction(
            period=0.1,
            actions=[configure_node],
            condition=IfCondition(OrSubstitution(auto_configure, auto_activate)),
        ),
        TimerAction(
            period=0.2,
            actions=[activate_node],
            condition=IfCondition(auto_activate),
        ),
    ])
