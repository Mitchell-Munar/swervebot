"""Verify the robot-specific entrypoint connects its full Xacro to the reusable controller."""

from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    """Exercise the installed robot-specific launch, rather than the generic demo."""
    share = Path(get_package_share_directory('swervebot_bringup'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/mock.launch.py'))),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRobotMock(unittest.TestCase):
    """Drive the configured robot through GenericSystem and verify its watchdog."""

    def test_driving_and_timeout(self):
        rclpy.init()
        node = rclpy.create_node('robot_mock_test')
        feedback = None

        def receive(message):
            nonlocal feedback
            feedback = message

        subscription = node.create_subscription(JointState, '/joint_states', receive, 10)
        publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')

        def wait(predicate, timeout=15.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            self.fail('Robot mock bringup did not reach the expected state')

        try:
            self.assertTrue(client.wait_for_service(timeout_sec=30.0))
            future = client.call_async(ListControllers.Request())
            wait(future.done)
            # The spawner may still be switching; poll until both controllers are active.

            def active():
                nonlocal future
                if not future.done():
                    return False
                names = {item.name for item in future.result().controller
                         if item.state == 'active'}
                if names == {
                        'joint_state_broadcaster', 'swerve_controller'}:
                    return True
                future = client.call_async(ListControllers.Request())
                return False

            wait(active)
            wait(lambda: feedback is not None and publisher.get_subscription_count() == 1)
            command = Twist()
            command.linear.x = 0.075

            def matches(speed):
                wheels = [velocity for name, velocity in zip(feedback.name, feedback.velocity)
                          if name.startswith('wheel_')]
                return len(wheels) == 4 and all(abs(value - speed) < 0.01 for value in wheels)

            def moving():
                publisher.publish(command)
                return matches(1.0)

            wait(moving)
            wait(lambda: matches(0.0), timeout=2.0)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_publisher(publisher)
            node.destroy_client(client)
            node.destroy_node()
            rclpy.shutdown()
