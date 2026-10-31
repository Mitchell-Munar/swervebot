"""Expose the robot simulation from the top-level integration package."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Forward the simulation's public settings without duplicating orchestration."""
    controllers = Path(get_package_share_directory('swerve_drive_controller'))
    simulation = Path(get_package_share_directory('swervebot_sim'))
    defaults = {'simulation_config': str(simulation / 'config/simulation.yaml'),
                'gui': 'true', 'paused': 'false', 'use_sim_time': 'true',
                'controller_config': str(controllers / 'config/controllers.yaml')}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation / 'launch/simulation.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
