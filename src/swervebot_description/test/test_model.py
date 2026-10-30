"""Check model composition and sensor contracts without launching ROS nodes."""

import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


DESCRIPTION = Path(get_package_share_directory('swervebot_description')) / 'description'


def expand(directory=DESCRIPTION, **mappings):
    return ET.fromstring(xacro.process_file(
        str(directory / 'swervebot.urdf.xacro'), mappings=mappings).toxml())


@pytest.mark.parametrize('mode,plugin', [
    ({}, 'my_robot_hardware/MyRobotHardwareInterface'),
    ({'use_mock_hardware': 'true'}, 'mock_components/GenericSystem'),
    ({'use_sim': 'true'}, 'gz_ros2_control/GazeboSimSystem'),
])
def test_composition(mode, plugin):
    robot = expand(**mode)
    assert [p.text for p in robot.findall('ros2_control/hardware/plugin')] == [plugin]
    joints = robot.findall('joint')
    assert len([j for j in joints if j.attrib['type'] != 'fixed']) == 8
    links = {link.attrib['name'] for link in robot.findall('link')}
    children = {joint.find('child').attrib['link'] for joint in joints}
    assert links - children == {'base_footprint'}
    assert len(children) == len(joints) == len(links) - 1
    reached = {'base_footprint'}
    while True:
        next_links = {j.find('child').attrib['link'] for j in joints
                      if j.find('parent').attrib['link'] in reached}
        if next_links <= reached:
            break
        reached |= next_links
    assert reached == links
    assert len(robot.findall('ros2_control/joint')) == 8
    sensor = robot.find('gazebo/sensor')
    assert (sensor is not None) == (mode.get('use_sim') == 'true')
    laser = robot.find("link[@name='laser_frame']")
    assert float(laser.find('inertial/mass').attrib['value']) == 0.1
    assert all(float(laser.find('inertial/inertia').attrib[k]) > 0
               for k in ('ixx', 'iyy', 'izz'))
    assert laser.find('visual/material/color') is not None
    mount = robot.find("joint[@name='laser_joint']/origin")
    assert [float(v) for v in mount.attrib['xyz'].split()] == pytest.approx([0.15, 0, 0.095])
    if sensor is not None:
        assert sensor.findtext('gz_frame_id') == 'laser_frame'
        assert float(sensor.findtext('ray/scan/horizontal/min_angle')) == -math.pi
        assert float(sensor.findtext('ray/scan/horizontal/max_angle')) == math.pi


def test_conflicting_hardware():
    with pytest.raises(xacro.XacroException, match='mutually exclusive'):
        expand(use_sim='true', use_mock_hardware='true')


def test_variant_properties(tmp_path):
    # Copy the assembly so testing alternative robot dimensions never edits user configuration.
    for source in DESCRIPTION.glob('*.xacro'):
        text = source.read_text().replace('$(find swervebot_description)/description/',
                                          str(tmp_path) + '/')
        (tmp_path / source.name).write_text(text)
    properties = tmp_path / 'swervebot_properties.xacro'
    properties.write_text(properties.read_text()
                          .replace('name="lidar_enabled" value="true"',
                                   'name="lidar_enabled" value="false"')
                          .replace('name="wheelbase" value="0.26"',
                                   'name="wheelbase" value="0.4"')
                          .replace('name="wheel_radius" value="0.075"',
                                   'name="wheel_radius" value="0.1"')
                          .replace('name="module_2_x" value="${1 * wheelbase / 2}"',
                                   'name="module_2_x" value="0.23"'))
    robot = expand(tmp_path, use_sim='true')
    assert robot.find("link[@name='laser_frame']") is None
    assert robot.find('gazebo/sensor') is None
    pivot = robot.find("joint[@name='actuator_front_left_joint']/origin")
    assert float(pivot.attrib['xyz'].split()[0]) == 0.2
    wheel = robot.find("link[@name='wheel_front_left']/collision/geometry/cylinder")
    assert float(wheel.attrib['radius']) == 0.1
    custom_pivot = robot.find("joint[@name='actuator_front_right_joint']/origin")
    assert float(custom_pivot.attrib['xyz'].split()[0]) == 0.23


def test_sdf_sensor_pose(tmp_path):
    if shutil.which('gz') is None:
        pytest.skip('Gazebo CLI unavailable')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(ET.tostring(expand(use_sim='true'), encoding='unicode'))
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf)],
                            capture_output=True, text=True, check=True)
    sdf = ET.fromstring(result.stdout)
    sensor = sdf.find('.//sensor')
    assert sensor is not None
    assert sensor.findtext('gz_frame_id') == 'laser_frame'
    assert [float(v) for v in sensor.findtext('pose').split()] == pytest.approx(
        [0.15, 0, 0.295, 0, 0, 0])
