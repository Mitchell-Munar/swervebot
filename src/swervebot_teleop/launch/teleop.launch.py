"""Launch joystick input and manual mode projection with configurable ROS names."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Resolve the input driver and profile once before starting the nodes."""
    driver = LaunchConfiguration('driver_type').perform(context)
    if driver not in ('game_controller', 'joy'):
        raise ValueError('driver_type must be game_controller or joy')
    namespace = LaunchConfiguration('namespace')
    parameters = {
        'autorepeat_rate': float(LaunchConfiguration('autorepeat_rate').perform(context)),
        'deadzone': float(LaunchConfiguration('driver_deadzone').perform(context)),
        'device_id': int(LaunchConfiguration('device_id').perform(context)),
    }
    return [
        Node(package='joy', executable='game_controller_node' if driver == 'game_controller'
             else 'joy_node', namespace=namespace, parameters=[parameters]),
        Node(package='swervebot_teleop', executable='teleop_node', namespace=namespace,
             parameters=[{'config_file': LaunchConfiguration('config_file')}],
             remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]),
    ]


def generate_launch_description():
    """Expose the joystick profile and integration settings without editing package files."""
    profile = Path(get_package_share_directory('swervebot_teleop')) / 'config/game_controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('driver_type', default_value='game_controller'),
        DeclareLaunchArgument('config_file', default_value=str(profile)),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('driver_deadzone', default_value='0.05'),
        DeclareLaunchArgument('autorepeat_rate', default_value='50.0'),
        OpaqueFunction(function=launch_setup),
    ])
