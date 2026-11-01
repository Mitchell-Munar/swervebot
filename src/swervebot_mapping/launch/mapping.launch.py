from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=PathJoinSubstitution([FindPackageShare('swervebot_mapping'),'config','slam.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py'])), launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time'),'slam_params_file':LaunchConfiguration('slam_params_file')}.items())])
