"""Exercise rendered scans, the ROS bridge, and the static sensor transform."""

import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    path = Path(get_package_share_directory('swervebot_sim')) / 'launch/simulation.launch.py'
    # The front face is x=1.5 m, hence 1.35 m from the sensor at x=0.15 m.
    obstacle = """<sdf version="1.12"><model name="lidar_target"><static>true</static>
      <pose>0 0 0 0 0 0</pose><link name="box">
      <visual name="visual"><geometry><box><size>1 2 1</size></box></geometry></visual>
      <collision name="collision"><geometry><box><size>1 2 1</size></box></geometry>
      </collision></link></model></sdf>"""
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                 launch_arguments={'gui': 'false'}.items()),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'swervebot_empty', '-string', obstacle, '-x', '2', '-z', '0.5']),
        launch_testing.actions.ReadyToTest(),
    ])


class TestLidar(unittest.TestCase):

    def test_scan_and_transform(self):
        rclpy.init()
        node = rclpy.create_node('lidar_test')
        scans, transforms = [], {}

        def static_tf(message):
            transforms.update({t.child_frame_id: t for t in message.transforms})

        node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
        node.create_subscription(TFMessage, '/tf_static', static_tf,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(scans) >= 3 and 'laser_frame' in transforms:
                    scan = scans[-1]
                    center = len(scan.ranges) // 2
                    if math.isfinite(scan.ranges[center]):
                        break
            self.assertGreaterEqual(len(scans), 3, 'No advancing ROS lidar scans')
            scan = scans[-1]
            self.assertEqual(scan.header.frame_id, 'laser_frame')
            self.assertEqual(len(scan.ranges), 500)
            self.assertAlmostEqual(scan.angle_min, -math.pi, places=5)
            self.assertAlmostEqual(scan.angle_max, math.pi, places=5)
            self.assertGreater(scan.angle_increment, 0)
            self.assertNotEqual(scans[0].header.stamp, scan.header.stamp)
            self.assertAlmostEqual(scan.ranges[len(scan.ranges) // 2], 1.35, delta=0.06)
            laser = transforms['laser_frame']
            self.assertEqual(laser.header.frame_id, 'base_link')
            self.assertAlmostEqual(laser.transform.translation.z, 0.095)
            self.assertEqual(transforms['base_link'].header.frame_id, 'base_footprint')
        finally:
            node.destroy_node()
            rclpy.shutdown()
