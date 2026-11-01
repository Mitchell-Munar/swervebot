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
    actions = [DeclareLaunchArgument('use_sim_time', default_value='true'), DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([FindPackageShare('swervebot_navigation'),'config','nav2.yaml']))]
    actions.append(DeclareLaunchArgument('map'))
    actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[config, clock, {'yaml_filename':LaunchConfiguration('map')}], remappings=[], output='screen'))
    actions.append(Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[config, clock], remappings=[], output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', parameters=[clock, {'autostart':True,'node_names':['map_server', 'amcl']}]))
    return LaunchDescription(actions)
