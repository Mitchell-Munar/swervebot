"""Run a reusable GenericSystem demo using only this package and standard ROS packages."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def mock_description(parameters):
    """Create a minimal kinematic model with N independently steered mock drive joints."""
    wheels, actuators = parameters['wheel_names'], parameters['actuator_names']
    fields = ('module_x', 'module_y', 'wheel_radii', 'wheel_offsets', 'steering_continuous',
              'steering_min', 'steering_max', 'steering_velocity_limits')
    if len(wheels) < 3 or len(actuators) != len(wheels) or any(
            len(parameters[field]) != len(wheels) for field in fields):
        raise ValueError('All module arrays must match at least three wheel/actuator names')
    robot = ET.Element('robot', name='swerve_mock')
    base = parameters.get('base_frame_id', 'base_footprint')
    ET.SubElement(robot, 'link', name=base)
    control = ET.SubElement(robot, 'ros2_control', name='SwerveMockSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
    ET.SubElement(hardware, 'param', name='calculate_dynamics').text = 'true'
    for i, (wheel, actuator) in enumerate(zip(wheels, actuators)):
        continuous = parameters['steering_continuous'][i]
        initial = 0.0 if continuous else min(max(0.0, parameters['steering_min'][i]),
                                             parameters['steering_max'][i])
        for name, parent, xyz, rpy, kind in (
                (actuator, base, f'{parameters["module_x"][i]} {parameters["module_y"][i]} 0',
                 '0 0 0', 'continuous' if continuous else 'revolute'),
                (wheel, actuator + '_link', f'0 {parameters["wheel_offsets"][i]} 0',
                 '-1.5707963267948966 0 0', 'continuous')):
            ET.SubElement(robot, 'link', name=name + '_link')
            joint = ET.SubElement(robot, 'joint', name=name, type=kind)
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=name + '_link')
            ET.SubElement(joint, 'origin', xyz=xyz, rpy=rpy)
            ET.SubElement(joint, 'axis', xyz='0 0 1')
            if name == actuator:
                limit = {'velocity': str(parameters['steering_velocity_limits'][i]),
                         'effort': '10'}
                if not continuous:
                    limit.update(lower=str(parameters['steering_min'][i]),
                                 upper=str(parameters['steering_max'][i]))
                ET.SubElement(joint, 'limit', **limit)
            interface_joint = ET.SubElement(control, 'joint', name=name)
            interface = ET.SubElement(interface_joint, 'command_interface',
                                      name='velocity' if name == wheel else 'position')
            if name == actuator and not continuous:
                ET.SubElement(interface, 'param', name='min').text = str(
                    parameters['steering_min'][i])
                ET.SubElement(interface, 'param', name='max').text = str(
                    parameters['steering_max'][i])
            position = ET.SubElement(interface_joint, 'state_interface', name='position')
            ET.SubElement(position, 'param', name='initial_value').text = str(
                initial if name == actuator else 0.0)
            ET.SubElement(interface_joint, 'state_interface', name='velocity')
    return ET.tostring(robot, encoding='unicode')


def launch_setup(context):
    """Resolve names and materialize namespaced YAML without modifying the supplied file."""
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = '/' + namespace if namespace else ''
    clock = LaunchConfiguration('use_sim_time').perform(context).lower()
    if clock not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    use_sim_time = clock == 'true'
    with Path(LaunchConfiguration('controller_config').perform(context)).open(
            encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    params = config['swerve_controller']['ros__parameters']
    description = mock_description(params)
    # Fully qualified node keys let the same YAML launch in any namespace.
    namespaced = {}
    for name, section in config.items():
        section.setdefault('ros__parameters', {})['use_sim_time'] = use_sim_time
        namespaced[prefix + '/' + name] = section
    temporary = tempfile.TemporaryDirectory(prefix='swerve_mock_')
    config_path = Path(temporary.name) / 'controllers.yaml'
    with config_path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(namespaced, stream, sort_keys=False)

    def cleanup(_context):
        temporary.cleanup()

    manager = Node(
        package='controller_manager', executable='ros2_control_node', namespace=namespace,
        parameters=[str(config_path)], remappings=[('~/robot_description', 'robot_description')],
        output='screen')
    publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
        parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
        output='screen')
    spawner = Node(
        package='controller_manager', executable='spawner', namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['joint_state_broadcaster', 'swerve_controller',
                   '--controller-manager', prefix + '/controller_manager',
                   '--controller-manager-timeout', '30', '--param-file', str(config_path),
                   '--activate-as-group'], output='screen')

    def checked_exit(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Mock controller activation failed'))]
        return []

    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=checked_exit)),
            publisher, manager, spawner]


def generate_launch_description():
    """Expose configuration, namespace and clock selection for other workspaces."""
    share = Path(get_package_share_directory('swervebot_controllers'))
    config = share / 'config/controllers_mock.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('controller_config', default_value=str(config)),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
