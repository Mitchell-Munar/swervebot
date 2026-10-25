"""Exercise the installed controller against GenericSystem without Gazebo."""

import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, SwitchController
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    """Launch the production mock entry point, including both controller spawners."""
    path = get_package_share_directory('swervebot_controllers')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            path + '/launch/controller_mock_test.launch.py')),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMockConnection(unittest.TestCase):
    """Verify commands reach the mock hardware and lifecycle transitions remain safe."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mock_connection_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail('Timed out waiting for mock controller condition')

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, '/controller_manager/' + name)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        future = client.call_async(request)
        self.wait_for(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def test_command_feedback_and_lifecycle(self):
        wheels = ['wheel_' + corner + '_joint' for corner in
                  ['front_left', 'front_right', 'back_left', 'back_right']]
        actuators = [name.replace('wheel_', 'actuator_') for name in wheels]
        self.feedback = None
        self.odom = None
        self.odom_tf = None
        joint_sub = self.node.create_subscription(
            JointState, '/joint_states', lambda msg: setattr(self, 'feedback', msg), 10)
        odom_sub = self.node.create_subscription(
            Odometry, '/odom', lambda msg: setattr(self, 'odom', msg), 10)

        def receive_tf(msg):
            for transform in msg.transforms:
                if (transform.header.frame_id == 'odom' and
                        transform.child_frame_id == 'base_footprint'):
                    self.odom_tf = transform

        tf_sub = self.node.create_subscription(TFMessage, '/tf', receive_tf, 10)
        publisher = self.node.create_publisher(Twist, '/cmd_vel', 1)

        def active():
            response = self.call(ListControllers, 'list_controllers', ListControllers.Request())
            return any(item.name == 'swerve_controller' and item.state == 'active'
                       for item in response.controller)

        self.wait_for(active, 30.0)
        interfaces = self.call(ListHardwareInterfaces, 'list_hardware_interfaces',
                               ListHardwareInterfaces.Request())
        claimed = {item.name for item in interfaces.command_interfaces if item.is_claimed}
        self.assertEqual(claimed, {name + '/velocity' for name in wheels} |
                         {name + '/position' for name in actuators})
        self.wait_for(lambda: self.feedback is not None and self.odom is not None)
        self.wait_for(lambda: self.odom_tf is not None)
        self.assertEqual(self.odom.header.frame_id, 'odom')
        self.assertEqual(self.odom.child_frame_id, 'base_footprint')
        self.wait_for(lambda: publisher.get_subscription_count() > 0)

        def matches(speed, angle):
            msg = self.feedback
            if msg is None or not set(wheels + actuators).issubset(msg.name):
                return False
            return (all(abs(msg.velocity[msg.name.index(name)] - speed) < 0.01
                        for name in wheels) and
                    all(abs(msg.position[msg.name.index(name)] - angle) < 0.01
                        for name in actuators))

        def drive(vx, vy, speed, angle):
            command = Twist()
            command.linear.x = vx
            command.linear.y = vy

            def arrived():
                publisher.publish(command)
                return matches(speed, angle)

            self.wait_for(arrived)

        drive(0.075, 0.0, 1.0, 0.0)
        drive(0.075, 0.075, 0.0, 0.0)
        drive(0.075, 0.0, 1.0, 0.0)
        # No more publications: the command expires and steering holds its feedback position.
        self.wait_for(lambda: matches(0.0, 0.0), 2.0)
        drive(0.075, 0.075, 0.0, 0.0)
        drive(0.0, 0.0, 0.0, 0.0)
        drive(0.075, 0.0, 1.0, 0.0)

        def switch(activate):
            request = SwitchController.Request()
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = 5
            if activate:
                request.activate_controllers = ['swerve_controller']
            else:
                request.deactivate_controllers = ['swerve_controller']
            self.assertTrue(self.call(SwitchController, 'switch_controller', request).ok)

        switch(False)
        self.wait_for(lambda: matches(0.0, 0.0))
        switch(True)
        # Keep observing for several manager cycles to catch stale-command replay.
        end = time.monotonic() + 0.25
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self.assertTrue(matches(0.0, 0.0))
        drive(0.075, 0.0, 1.0, 0.0)
        drive(float('nan'), 0.0, 0.0, 0.0)
        self.assertTrue(active())
        self.node.destroy_subscription(joint_sub)
        self.node.destroy_subscription(odom_sub)
        self.node.destroy_subscription(tf_sub)
        self.node.destroy_publisher(publisher)
