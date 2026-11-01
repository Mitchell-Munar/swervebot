"""Compose simulation with either manual mapping or saved-map navigation."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('slam', 'nav'):
        raise ValueError('mode must be slam or nav')
    if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
        raise ValueError('Gazebo requires use_sim_time:=true')
    if mode == 'nav' and not Path(LaunchConfiguration('map').perform(context)).is_file():
        raise ValueError('nav mode requires a readable map YAML')

    def include(package, filename, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', filename])),
            launch_arguments=arguments.items())

    clock = LaunchConfiguration('use_sim_time')
    actions = [include('swervebot_sim', 'simulation.launch.py', use_sim_time=clock,
                       gui=LaunchConfiguration('gui')),
               include('swervebot_navigation', 'mux.launch.py', use_sim_time=clock)]
    if mode == 'slam':
        actions.append(include('swervebot_mapping', 'mapping.launch.py', use_sim_time=clock,
                               slam_params_file=LaunchConfiguration('slam_params_file')))
    else:
        for name in ('localization', 'navigation'):
            actions.append(include('swervebot_navigation', name + '.launch.py',
                                   use_sim_time=clock, map=LaunchConfiguration('map'),
                                   params_file=LaunchConfiguration('params_file')))
    actions.append(Node(package='rviz2', executable='rviz2',
                        condition=IfCondition(LaunchConfiguration('rviz')),
                        arguments=['-d', PathJoinSubstitution([
                            FindPackageShare('swervebot_navigation'), 'rviz', 'mapping.rviz' if mode == 'slam' else 'navigation.rviz'])],
                        parameters=[{'use_sim_time': ParameterValue(clock, value_type=bool)}]))
    return actions


def generate_launch_description():
    defaults = {'mode': 'slam', 'use_sim_time': 'true', 'map': '', 'gui': 'true',
                'rviz': 'true', 'params_file': PathJoinSubstitution([
                    FindPackageShare('swervebot_navigation'), 'config', 'nav2.yaml']),
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare('swervebot_mapping'), 'config', 'slam.yaml'])}
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        OpaqueFunction(function=setup)])
