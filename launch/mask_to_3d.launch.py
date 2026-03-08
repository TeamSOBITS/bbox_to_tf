import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_dir = get_package_share_directory('image_to_position')
    default_params_file = os.path.join(pkg_dir, 'config', 'mask_to_3d.yaml')
    
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    # Use LifecycleNode instead of ComposableNodeContainer for direct state control
    mask_node = LifecycleNode(
        package='image_to_position',
        executable='mask_to_3d',
        name='mask_to_3d',
        namespace='',
        parameters=[params_file],
        output='screen'
    )


    return LaunchDescription([
        params_file_arg,
        mask_node,
    ])
