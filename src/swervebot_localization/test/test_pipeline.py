"""Exercise real Madgwick, conditioning and EKF processes with synthetic board data."""
import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from tf2_msgs.msg import TFMessage
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    """Use the shipped estimation launch and wall time to isolate it from Gazebo."""
    share = Path(get_package_share_directory('swervebot_localization'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/localization.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPipeline(unittest.TestCase):
    """Verify sensor flow, signed crab motion, rotation and dropout recovery."""

    def test_holonomic_pipeline(self):
        rclpy.init()
        node = rclpy.create_node('fusion_runtime_test')
        try:
            wheel_pub = node.create_publisher(Odometry, '/wheel/odometry', 10)
            imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
            mag_pub = node.create_publisher(MagneticField, '/mag', qos_profile_sensor_data)
            outputs, imus, transforms, authorities = [], [], [], set()
            node.create_subscription(Odometry, '/odom', outputs.append, 10)
            node.create_subscription(Imu, '/imu/data', imus.append, 10)

            def tf_callback(msg, info):
                for transform in msg.transforms:
                    if (transform.header.frame_id, transform.child_frame_id) == (
                            'odom', 'base_footprint'):
                        transforms.append(transform)
                        authorities.add(repr(info['publisher_gid']))

            node.create_subscription(TFMessage, '/tf', tf_callback, 100)
            broadcaster = StaticTransformBroadcaster(node)
            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.child_frame_id = 'imu_link'
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            yaw = 0.0

            def phase(duration, vy=0.0, omega=0.0, sensors=True):
                nonlocal yaw
                start = time.monotonic()
                previous = start
                while time.monotonic() - start < duration:
                    now = time.monotonic()
                    yaw += omega * (now - previous)
                    previous = now
                    stamp = node.get_clock().now().to_msg()
                    wheel = Odometry()
                    wheel.header.stamp = stamp
                    wheel.header.frame_id = 'odom'
                    wheel.child_frame_id = 'base_footprint'
                    wheel.pose.pose.orientation.w = 1.0
                    wheel.twist.twist.linear.y = vy
                    wheel.twist.twist.angular.z = omega
                    wheel_pub.publish(wheel)
                    if sensors:
                        imu = Imu()
                        imu.header.stamp = stamp
                        imu.header.frame_id = 'imu_link'
                        imu.orientation_covariance[0] = -1.0
                        imu.angular_velocity.z = omega
                        imu.linear_acceleration.z = 9.81
                        imu_pub.publish(imu)
                        mag = MagneticField()
                        mag.header = imu.header
                        # Magnetic north is world +Y in ENU, rotated into the board frame.
                        mag.magnetic_field.x = 2e-5 * math.sin(yaw)
                        mag.magnetic_field.y = 2e-5 * math.cos(yaw)
                        mag.magnetic_field.z = -4.5e-5
                        mag_pub.publish(mag)
                    rclpy.spin_once(node, timeout_sec=0.005)
                    time.sleep(0.015)

            phase(3.0)
            self.assertTrue(outputs, 'EKF did not publish')
            self.assertTrue(imus, 'Madgwick/conditioning did not publish')
            self.assertAlmostEqual(outputs[-1].pose.pose.position.y, 0.0, delta=0.05)
            self.assertAlmostEqual(outputs[-1].pose.pose.orientation.z, 0.0, delta=0.1)
            before = outputs[-1].pose.pose.position.y
            phase(3.0, vy=0.3)
            self.assertGreater(outputs[-1].twist.twist.linear.y, 0.2)
            self.assertGreater(outputs[-1].pose.pose.position.y - before, 0.5)
            before = outputs[-1].pose.pose.position.y
            phase(3.0, vy=-0.3)
            self.assertLess(outputs[-1].twist.twist.linear.y, -0.2)
            self.assertLess(outputs[-1].pose.pose.position.y - before, -0.5)
            phase(2.0, omega=0.3)
            self.assertGreater(outputs[-1].twist.twist.angular.z, 0.2)
            phase(1.0, sensors=False)
            count = len(imus)
            phase(2.0)
            self.assertGreater(len(imus), count, 'IMU failed to recover after dropout')
            self.assertTrue(transforms)
            self.assertEqual(len(authorities), 1, 'Multiple odometry TF authorities')
            self.assertEqual(outputs[-1].child_frame_id, 'base_footprint')
            for imu in imus:
                q = imu.orientation
                self.assertAlmostEqual(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, places=3)
                self.assertGreater(imu.orientation_covariance[0], 0.0)
                self.assertGreater(imu.angular_velocity_covariance[8], 0.0)
            for odom in outputs:
                self.assertTrue(all(math.isfinite(v) for v in odom.pose.covariance))
                self.assertTrue(all(math.isfinite(v) for v in odom.twist.covariance))
        finally:
            node.destroy_node()
            rclpy.shutdown()
