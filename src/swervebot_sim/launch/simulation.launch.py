"""Run one swerve robot using Gazebo's embedded ros2_control manager."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _launch_simulation(context):
    """Resolve configuration and retain temporary files until shutdown."""
    def boolean(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    sim_share = Path(get_package_share_directory('swervebot_sim'))
    description_share = Path(get_package_share_directory('swervebot_description'))
    use_sim_time = boolean('use_sim_time')
    # gz_ros2_control forces true for its manager; fail rather than silently mix ROS clocks.
    if not use_sim_time:
        raise ValueError('gz_ros2_control forces use_sim_time=true; false would mix ROS clocks')
    gui, paused = boolean('gui'), boolean('paused')
    config_source = Path(LaunchConfiguration('controller_config').perform(context))
    with config_source.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    # The Gazebo plugin needs a filename, not a launch parameter dictionary. Materialize a copy,
    # leaving the user's YAML untouched and giving manager/controllers the same clock selection.
    for name in ('controller_manager', 'swerve_controller', 'joint_state_broadcaster'):
        parameters = config.setdefault(name, {}).setdefault('ros__parameters', {})
        parameters['use_sim_time'] = use_sim_time
    temporary = tempfile.TemporaryDirectory(prefix='swervebot_sim_')
    config_path = Path(temporary.name) / 'controllers.yaml'
    with config_path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    robot_description = xacro.process_file(
        str(description_share / 'description' / 'swervebot.urdf.xacro'),
        mappings={'use_sim': 'true', 'use_mock_hardware': 'false',
                  'controller_config': str(config_path)},
    ).toxml()

    def cleanup(_context):
        temporary.cleanup()

    def checked_exit(label, next_actions=()):
        def callback(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{label} failed: {event.returncode}'))]
            return list(next_actions)
        return callback

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='simulation_bridge',
        parameters=[{'config_file': str(sim_share / 'config' / 'bridge.yaml'),
                     'use_sim_time': use_sim_time}], output='screen')
    # Gazebo owns the only controller manager. Do not launch a separate ros2_control_node here.
    gz_args = ('-s ' if not gui else '') + ('-r ' if not paused else '')
    gz_args += '-v 3 ' + str(sim_share / 'worlds' / 'empty.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim')) /
                                          'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items())
    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_swervebot', output='screen',
        arguments=['-world', 'swervebot_empty', '-topic', '/robot_description',
                   '-name', 'swervebot', '-allow_renaming', 'false', '-z', '0.02'])
    spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['joint_state_broadcaster', 'swerve_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30', '--switch-timeout', '30',
                   '--param-file', str(config_path), '--activate-as-group'])
    # Process completion and service waits replace arbitrary startup sleeps. In paused mode,
    # activation completes once the user starts physics (switching occurs on a simulation tick).
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=checked_exit('Robot spawning', [spawner]))),
        RegisterEventHandler(OnProcessExit(
            target_action=spawner, on_exit=checked_exit('Controller activation'))),
        LogInfo(msg='Spawning one swervebot; paused startup requires Play to finish activation.'),
        state_publisher, bridge, gazebo, spawn,
    ]


def generate_launch_description():
    """Expose simulation clock, GUI, pause, and controller-file choices."""
    config = Path(get_package_share_directory('swervebot_controllers')) / 'config/controllers.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('paused', default_value='false', description='Start physics paused'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Required true by gz_ros2_control'),
        DeclareLaunchArgument('controller_config', default_value=str(config),
                              description='Controller YAML; launch adjusts a temporary copy'),
        OpaqueFunction(function=_launch_simulation),
    ])
