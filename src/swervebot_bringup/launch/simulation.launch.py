"""Expose the robot simulation from the top-level integration package."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Forward the simulation's public settings without duplicating orchestration."""
    controllers = Path(get_package_share_directory('swerve_drive_controller'))
    simulation = Path(get_package_share_directory('swervebot_sim'))
    defaults = {'simulation_config': str(simulation / 'config/simulation.yaml'),
                'enable_sensor_fusion': 'false', 'use_imu': 'true', 'gui': 'true', 'paused': 'false', 'use_sim_time': 'true',
                'controller_config': str(controllers / 'config/controllers.yaml')}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        DeclareLaunchArgument('enable_foxglove_bridge', default_value='true'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        # Resolve the shared bridge launch at runtime and inherit this entrypoint's clock.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('swervebot_bringup'), 'launch', 'foxglove_bridge.launch.py'])),
            condition=IfCondition(LaunchConfiguration('enable_foxglove_bridge')),
            launch_arguments={key: LaunchConfiguration(key) for key in
                              ('foxglove_address', 'foxglove_port', 'use_sim_time')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation / 'launch/simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
