from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    config = LaunchConfiguration('params_file')
    actions = [DeclareLaunchArgument('use_sim_time', default_value='true'), DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([FindPackageShare('swervebot_navigation'),'config','twist_mux.yaml']))]
    actions.append(Node(package='twist_mux', executable='twist_mux', name='twist_mux', parameters=[config, clock], remappings=[('cmd_vel_out','cmd_vel')], output='screen'))
    return LaunchDescription(actions)
