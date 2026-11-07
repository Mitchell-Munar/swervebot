"""Fuse holonomic wheel velocities with 6-DOF relative IMU orientation and gyro."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('swervebot_localization'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mag_topic', default_value='/imu/mag'),
        DeclareLaunchArgument('ekf_config', default_value=str(share / 'config/ekf.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/imu_filter.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'mag_topic': LaunchConfiguration('mag_topic'),
                              'output_topic': '/imu/data_prefiltered'}.items()),
        Node(package='swervebot_localization', executable='condition_measurements.py',
             name='condition_measurements',
             parameters=[str(share / 'config/conditioning.yaml'), clock], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[LaunchConfiguration('ekf_config'), clock],
             remappings=[('odometry/filtered', '/odom')]),
    ])
