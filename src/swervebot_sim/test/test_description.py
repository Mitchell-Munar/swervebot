"""Verify the world/model contract before invoking the physics engine."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import pytest
import xacro


def test_world_and_robot():
    """Check a single ground model, exactly eight joints, and consistent steering limits."""
    share = Path(get_package_share_directory('swervebot_description'))
    robot = ET.fromstring(xacro.process_file(
        str(share / 'description/swervebot.urdf.xacro'),
        mappings={'use_sim': 'true', 'use_mock_hardware': 'false'}).toxml())
    joints = [joint for joint in robot.findall('joint') if joint.attrib['type'] != 'fixed']
    assert len(joints) == 8
    for joint in joints:
        if joint.attrib['name'].startswith('actuator_'):
            assert joint.attrib['type'] == 'revolute'
            assert float(joint.find('limit').attrib['lower']) < 0
            assert float(joint.find('limit').attrib['upper']) > 0
        else:
            assert float(joint.find('origin').attrib['rpy'].split()[0]) < 0
    hardware = robot.findall('ros2_control/hardware/plugin')
    assert len(hardware) == 1
    assert hardware[0].text == 'gz_ros2_control/GazeboSimSystem'
    plugins = [plugin.attrib['name'] for plugin in robot.findall('gazebo/plugin')]
    assert 'gz_ros2_control::GazeboSimROS2ControlPlugin' in plugins
    world_path = Path(get_package_share_directory('swervebot_sim')) / 'worlds/empty.sdf'
    world = ET.parse(world_path).getroot().find('world')
    assert [model.attrib['name'] for model in world.findall('model')] == ['ground_plane']
    assert not world.findall('include')


def test_simulation_rejects_mixed_clocks():
    """The embedded Gazebo manager forces sim time, so wall-time launch must fail early."""
    share = Path(get_package_share_directory('swervebot_sim'))
    launch_path = share / 'launch/simulation.launch.py'
    spec = importlib.util.spec_from_file_location('swerve_sim_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'false'
    with pytest.raises(ValueError, match='forces use_sim_time=true'):
        module._launch_simulation(context)


def test_controller_geometry_matches_robot():
    """Check controller pivot coordinates, offsets, rolling radii and limits against Xacro."""
    import yaml

    description = Path(get_package_share_directory('swervebot_description'))
    controller = Path(get_package_share_directory('swervebot_controllers'))
    robot = ET.fromstring(xacro.process_file(
        str(description / 'description/swervebot.urdf.xacro'),
        mappings={'use_mock_hardware': 'true', 'use_sim': 'false'}).toxml())
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    links = {link.attrib['name']: link for link in robot.findall('link')}
    for filename in ('controllers.yaml', 'controllers_mock.yaml'):
        with (controller / 'config' / filename).open(encoding='utf-8') as stream:
            params = yaml.safe_load(stream)['swerve_controller']['ros__parameters']
        assert params['base_frame_id'] == 'base_footprint'
        for i, (wheel_name, steering_name) in enumerate(zip(
                params['wheel_names'], params['actuator_names'])):
            steering, wheel = joints[steering_name], joints[wheel_name]
            pivot = [float(value) for value in steering.find('origin').attrib['xyz'].split()]
            offset = [float(value) for value in wheel.find('origin').attrib['xyz'].split()]
            assert pivot[:2] == pytest.approx([params['module_x'][i], params['module_y'][i]])
            assert offset[0] == 0.0  # Longitudinal caster offset is outside the controller model.
            assert offset[1] == pytest.approx(params['wheel_offsets'][i])
            link = links[wheel.find('child').attrib['link']]
            radius = float(link.find('collision/geometry/cylinder').attrib['radius'])
            assert radius == pytest.approx(params['wheel_radii'][i])
            assert steering.find('axis').attrib['xyz'] == '0 0 1'
            limits = steering.find('limit').attrib
            assert not params['steering_continuous'][i]
            assert float(limits['lower']) == pytest.approx(params['steering_min'][i])
            assert float(limits['upper']) == pytest.approx(params['steering_max'][i])
            assert float(limits['velocity']) == pytest.approx(
                params['steering_velocity_limits'][i])
