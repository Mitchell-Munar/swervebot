import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('swervebot_teleop')
    
    driver_type = LaunchConfiguration('driver_type').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)

    # Auto-assign matching default config file if none specified
    if not config_file:
        if driver_type == 'game_controller':
            config_file = os.path.join(pkg_share, 'config', 'game_controller.yaml')
        else:
            pass
            # setup for other controller types
            # ex: config_file = os.path.join(pkg_share, 'config', 'flight_stick_joy.yaml')

    # Select appropriate driver node
    if driver_type == 'game_controller':
        driver_node = Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            parameters=[{'autorepeat_rate': 50.0}]
        )
    else:
        driver_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 50.0
            }]
        )

    teleop_node = Node(
        package='swervebot_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{'config_file': config_file}],
        remappings=[
            ('/cmd_vel/teleop', '/cmd_vel')
        ]
    )

    return [driver_node, teleop_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_type',
            default_value='game_controller',
            description='Joystick driver type: "game_controller" (Xbox/PlayStation/Steam Deck) or "joy" (Custom/Flight sticks)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to custom YAML config file (leaves empty to use driver default)'
        ),
        OpaqueFunction(function=launch_setup)
    ])