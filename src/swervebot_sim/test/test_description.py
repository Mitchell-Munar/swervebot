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
