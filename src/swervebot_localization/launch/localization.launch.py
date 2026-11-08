"""Fuse holonomic wheel velocities with 6-DOF relative IMU orientation and gyro."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
import yaml

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def estimator(context):
    # Remove the IMU sensor entirely for a matched wheel-only EKF comparison.
    with open(LaunchConfiguration('ekf_config').perform(context), encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)['ekf_filter_node']['ros__parameters']
    enabled = LaunchConfiguration('use_imu').perform(context).lower()
    if enabled not in ('true', 'false'):
        raise ValueError('use_imu must be true or false')
    if enabled == 'false':
        parameters = {k: v for k, v in parameters.items() if not k.startswith('imu')}
    else:
        parameters['imu0'] = '/imu/data'
    return [Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                 output='screen', parameters=[parameters, {'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool)}],
                 remappings=[('odometry/filtered', '/odom')])]


def generate_launch_description():
    share = Path(get_package_share_directory('swervebot_localization'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mag_topic', default_value='/mag'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('ekf_config', default_value=str(share / 'config/ekf.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/imu_filter.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_imu')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'mag_topic': LaunchConfiguration('mag_topic'),
                              'output_topic': '/imu/data_prefiltered'}.items()),
        Node(package='swervebot_localization', executable='condition_measurements.py',
             name='condition_measurements',
             parameters=[str(share / 'config/conditioning.yaml'), clock,
                         {'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool)}], output='screen'),
        OpaqueFunction(function=estimator),
    ])
