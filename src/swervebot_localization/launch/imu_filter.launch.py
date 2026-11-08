"""Estimate relative 6-DOF orientation without publishing a competing base transform."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('swervebot_localization'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('filter_config', default_value=str(share / 'config/madgwick.yaml')),
        DeclareLaunchArgument('output_topic', default_value='/imu/data_prefiltered'),
        DeclareLaunchArgument('mag_topic', default_value='/mag'),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick', output='screen',
             parameters=[LaunchConfiguration('filter_config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                         value_type=bool)}],
             remappings=[('imu/data_raw', LaunchConfiguration('imu_topic')),
                         ('imu/mag', LaunchConfiguration('mag_topic')),
                         ('imu/data', LaunchConfiguration('output_topic'))]),
    ])
