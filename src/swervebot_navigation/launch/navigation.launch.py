from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    # Resolve upstream zone toggles to booleans and expand BT package paths.
    config = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={},
        value_rewrites={'KEEPOUT_ZONE_ENABLED': 'false', 'SPEED_ZONE_ENABLED': 'false'},
        convert_types=True), allow_substs=True)
    actions = [DeclareLaunchArgument('use_sim_time', default_value='true'), DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([FindPackageShare('swervebot_navigation'),'config','nav2.yaml']))]
    actions.append(Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[config, clock], remappings=[('cmd_vel','cmd_vel_raw')], output='screen'))
    actions.append(Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[config, clock], remappings=[], output='screen'))
    actions.append(Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', parameters=[config, clock], remappings=[], output='screen'))
    actions.append(Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[config, clock], remappings=[('cmd_vel','cmd_vel_raw')], output='screen'))
    actions.append(Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[config, clock], remappings=[], output='screen'))
    actions.append(Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', parameters=[config, clock], remappings=[('cmd_vel','cmd_vel_raw')], output='screen'))
    actions.append(Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', parameters=[config, clock], remappings=[], output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', parameters=[clock, {'autostart':True,'node_names':['controller_server', 'planner_server', 'smoother_server', 'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor']}]))
    return LaunchDescription(actions)
