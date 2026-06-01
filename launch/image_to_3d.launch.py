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
    mask_default_params_file = os.path.join(pkg_dir, 'config', 'mask_to_3d.yaml')
    bbox_default_params_file = os.path.join(pkg_dir, 'config', 'bbox_to_3d.yaml')
    keypoint_default_params_file = os.path.join(pkg_dir, 'config', 'keypoint_to_3d.yaml')

    mask_params_file = LaunchConfiguration('mask_params_file')
    bbox_params_file = LaunchConfiguration('bbox_params_file')
    keypoint_params_file = LaunchConfiguration('keypoint_params_file')
    auto_configure = LaunchConfiguration('auto_configure')
    auto_activate = LaunchConfiguration('auto_activate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    auto_configure_arg = DeclareLaunchArgument(
        "auto_configure",
        default_value="False",
        description="Whether to configure lifecycle nodes on startup",
    )
    auto_activate_arg = DeclareLaunchArgument(
        "auto_activate",
        default_value="False",
        description="Whether to activate lifecycle nodes on startup",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock.',
    )

    mask_params_arg = DeclareLaunchArgument(
        'mask_params_file',
        default_value=mask_default_params_file,
        description='Full path to the mask to 3D parameters file to use'
    )

    bbox_params_arg = DeclareLaunchArgument(
        'bbox_params_file',
        default_value=bbox_default_params_file,
        description='Full path to the bbox to 3D parameters file to use'
    )

    keypoint_params_arg = DeclareLaunchArgument(
        'keypoint_params_file',
        default_value=keypoint_default_params_file,
        description='Full path to the keypoint to 3D parameters file to use'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    container = ComposableNodeContainer(
        name='image_to_3d_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::BboxTo3D',
                name='bbox_to_3d',
                namespace=namespace,
                parameters=[bbox_params_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::MaskTo3D',
                name='mask_to_3d',
                namespace=namespace,
                parameters=[mask_params_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::KeypointTo3D',
                name='keypoint_to_3d',
                namespace=namespace,
                parameters=[keypoint_params_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    bbox_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/bbox_to_3d' if '", namespace, "' else '/bbox_to_3d'",
    ])
    bbox_configure_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" >/dev/null 2>&1; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" configure',
            bbox_node_full_path,
        ],
        output='screen'
    )
    bbox_activate_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" 2>/dev/null | grep -q "inactive"; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" activate',
            bbox_node_full_path,
        ],
        output='screen'
    )

    mask_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/mask_to_3d' if '", namespace, "' else '/mask_to_3d'",
    ])
    mask_configure_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" >/dev/null 2>&1; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" configure',
            mask_node_full_path,
        ],
        output='screen'
    )
    mask_activate_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" 2>/dev/null | grep -q "inactive"; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" activate',
            mask_node_full_path,
        ],
        output='screen'
    )

    keypoint_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/keypoint_to_3d' if '", namespace, "' else '/keypoint_to_3d'",
    ])
    keypoint_configure_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" >/dev/null 2>&1; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" configure',
            keypoint_node_full_path,
        ],
        output='screen'
    )
    keypoint_activate_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'until ros2 lifecycle get "$0" 2>/dev/null | grep -q "inactive"; do sleep 0.2; done; '
            'ros2 lifecycle set "$0" activate',
            keypoint_node_full_path,
        ],
        output='screen'
    )

    return LaunchDescription([
        auto_configure_arg,
        auto_activate_arg,
        use_sim_time_arg,
        mask_params_arg,
        bbox_params_arg,
        keypoint_params_arg,
        namespace_cmd,
        container,
        TimerAction(
            period=0.1,
            actions=[bbox_configure_node],
            condition=IfCondition(OrSubstitution(auto_configure, auto_activate)),
        ),
        TimerAction(
            period=0.1,
            actions=[mask_configure_node],
            condition=IfCondition(OrSubstitution(auto_configure, auto_activate)),
        ),
        TimerAction(
            period=0.1,
            actions=[keypoint_configure_node],
            condition=IfCondition(OrSubstitution(auto_configure, auto_activate)),
        ),
        TimerAction(
            period=0.2,
            actions=[bbox_activate_node],
            condition=IfCondition(auto_activate),
        ),
        TimerAction(
            period=0.2,
            actions=[mask_activate_node],
            condition=IfCondition(auto_activate),
        ),
        TimerAction(
            period=0.2,
            actions=[keypoint_activate_node],
            condition=IfCondition(auto_activate),
        ),
    ])
