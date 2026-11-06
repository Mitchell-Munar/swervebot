"""Expose the Foxglove application bridge with robot clock settings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Delegate bridge configuration and startup to its upstream launch."""
    return LaunchDescription([
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])),
            launch_arguments={
                'address': LaunchConfiguration('foxglove_address'),
                'port': LaunchConfiguration('foxglove_port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])
