import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
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
                parameters=[bbox_params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::MaskTo3D',
                name='mask_to_3d',
                namespace=namespace,
                parameters=[mask_params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_to_position',
                plugin='image_to_position::KeypointTo3D',
                name='keypoint_to_3d',
                namespace=namespace,
                parameters=[keypoint_params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    bbox_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/bbox_to_3d' if '", namespace, "' else '/bbox_to_3d'",
    ])
    bbox_configure_node = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', bbox_node_full_path, 'configure'],
        output='screen'
    )

    mask_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/mask_to_3d' if '", namespace, "' else '/mask_to_3d'",
    ])
    mask_configure_node = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', mask_node_full_path, 'configure'],
        output='screen'
    )

    keypoint_node_full_path = PythonExpression([
        "'/' + '", namespace, "' + '/keypoint_to_3d' if '", namespace, "' else '/keypoint_to_3d'",
    ])
    keypoint_configure_node = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', keypoint_node_full_path, 'configure'],
        output='screen'
    )

    return LaunchDescription([
        mask_params_arg,
        bbox_params_arg,
        keypoint_params_arg,
        namespace_cmd,
        container,
        TimerAction(period=0.5, actions=[bbox_configure_node]),
        TimerAction(period=0.5, actions=[mask_configure_node]),
        TimerAction(period=0.5, actions=[keypoint_configure_node]),
    ])
