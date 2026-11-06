"""Launch joystick input and manual mode projection with configurable ROS names."""

from pathlib import Path
import subprocess

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_device(device, id_override='', name_override=''):
    """Resolve explicit launch selection before profile selection and legacy defaults."""
    device_id = device.get('id', 0)
    name = device.get('name', '')
    if type(device_id) is not int or device_id < 0:
        raise ValueError('device.id must be a nonnegative integer')
    if not isinstance(name, str):
        raise ValueError('device.name must be a string')
    if id_override:
        device_id = int(id_override)
        if device_id < 0:
            raise ValueError('device_id must be nonnegative')
        name = ''
    if name_override:
        name = name_override
    return device_id, name


def device_rows(output):
    """Extract SDL enumeration rows, excluding its table header and separator."""
    return [line for line in output.splitlines()
            if line.partition(':')[0].strip().isdigit() and ':' in line]


def launch_setup(context):
    """Resolve the input driver and profile once before starting the nodes."""
    driver = LaunchConfiguration('driver_type').perform(context)
    if driver not in ('game_controller', 'joy'):
        raise ValueError('driver_type must be game_controller or joy')
    namespace = LaunchConfiguration('namespace')
    profile = LaunchConfiguration('config_file').perform(context)
    if not profile:
        profile = str(Path(get_package_share_directory('swervebot_teleop')) /
                      'config/game_controller.yaml')
    with Path(profile).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    device = config.get('device', {}) if isinstance(config, dict) else None
    if not isinstance(device, dict):
        raise ValueError('Profile device must be a mapping')
    device_id, device_name = resolve_device(
        device, LaunchConfiguration('device_id').perform(context),
        LaunchConfiguration('device_name').perform(context))
    diagnostics = [LogInfo(msg=f'Joystick profile: {profile}; selection: '
                           f'device_id={device_id}, device_name={device_name!r}')]
    enumerator = Path(get_package_prefix('joy')) / 'lib/joy/joy_enumerate_devices'
    try:
        result = subprocess.run([str(enumerator)], capture_output=True, text=True,
                                timeout=5, check=True)
        diagnostics.append(LogInfo(msg=result.stdout))
        if len(device_rows(result.stdout)) > 1:
            diagnostics.append(LogInfo(msg=(
                'WARNING: Multiple joystick devices detected. Confirm the intended controller '
                f'and set device.id or device.name in {profile}, or override through launch '
                'arguments device_id/device_name.')))
    except (OSError, subprocess.SubprocessError) as error:
        diagnostics.append(LogInfo(msg=f'WARNING: Device enumeration failed: {error}; '
                                   'the joystick driver will attempt the configured selection.'))
    parameters = {
        'autorepeat_rate': float(LaunchConfiguration('autorepeat_rate').perform(context)),
        'deadzone': float(LaunchConfiguration('driver_deadzone').perform(context)),
        'device_id': device_id,
        'device_name': device_name,
    }
    return [
        *diagnostics,
        Node(package='joy', executable='game_controller_node' if driver == 'game_controller'
             else 'joy_node', namespace=namespace, parameters=[parameters]),
        Node(package='swervebot_teleop', executable='teleop_node', namespace=namespace,
             parameters=[{'config_file': LaunchConfiguration('config_file')}],
             remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]),
    ]


def generate_launch_description():
    """Expose the joystick profile and integration settings without editing package files."""
    profile = Path(get_package_share_directory('swervebot_teleop')) / 'config/game_controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('driver_type', default_value='game_controller'),
        DeclareLaunchArgument('config_file', default_value=str(profile)),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('device_id', default_value=''),
        DeclareLaunchArgument('device_name', default_value=''),
        DeclareLaunchArgument('driver_deadzone', default_value='0.05'),
        DeclareLaunchArgument('autorepeat_rate', default_value='50.0'),
        OpaqueFunction(function=launch_setup),
    ])
