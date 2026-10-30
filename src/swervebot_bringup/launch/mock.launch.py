"""Run this robot's complete description with mock ros2_control hardware."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep robot integration here while the controller demo remains independent."""
    controllers = Path(get_package_share_directory('swerve_drive_controller'))
    description = Path(get_package_share_directory('swervebot_description'))
    config = LaunchConfiguration('controller_config')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    robot = ParameterValue(Command([
        'xacro ', str(description / 'description/swervebot.urdf.xacro'),
        ' use_mock_hardware:=true use_sim:=false']), value_type=str)
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot, 'use_sim_time': clock}], output='screen')
    manager = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[config, {'use_sim_time': clock}],
        remappings=[('~/robot_description', 'robot_description')], output='screen')
    spawner = Node(
        package='controller_manager', executable='spawner',
        parameters=[{'use_sim_time': clock}],
        arguments=['joint_state_broadcaster', 'swerve_drive_controller', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '30',
                   '--param-file', config, '--activate-as-group'], output='screen')

    def checked_exit(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot mock controller activation failed'))]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('controller_config',
                              default_value=str(controllers / 'config/controllers_mock.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=checked_exit)),
        publisher, manager, spawner,
    ])
