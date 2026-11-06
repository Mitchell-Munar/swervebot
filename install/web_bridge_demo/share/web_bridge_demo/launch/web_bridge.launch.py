"""Launch the ping service and the Foxglove WebSocket bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml')),
            launch_arguments={
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
            }.items(),
        ),
        Node(package='web_bridge_demo', executable='ping_service_node', output='screen'),
    ])
