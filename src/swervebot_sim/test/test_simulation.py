"""Test physical motion against Gazebo poses, independently of wheel odometry."""

import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    """Run the production launch headlessly; use the same model, plugin, and controllers."""
    path = Path(get_package_share_directory('swervebot_sim')) / 'launch/simulation.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                 launch_arguments={'gui': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimulation(unittest.TestCase):
    """Assert signed physical displacement, steering, clocks, and live command policies."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('simulation_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=40.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        self.fail(f'Timed out: clock={self.sim_time}, truth={self.truth is not None}, '
                  f'odom={self.odom is not None}, joints={self.joints is not None}')

    def call(self, srv_type, path, request):
        client = self.node.create_client(srv_type, path)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), path)
        future = client.call_async(request)
        self.wait_for(future.done)
        result = future.result()
        self.node.destroy_client(client)
        return result

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def pose(self):
        transform = self.truth.transform
        return (transform.translation.x, transform.translation.y,
                self.yaw(transform.rotation))

    def run_for(self, seconds, vx=None, yaw=0.0, lateral=0.0, nonplanar=0.0):
        """Publish at 50 Hz wall time while measuring the interval in simulation time."""
        start = self.sim_time
        deadline = time.monotonic() + max(15.0, 10 * seconds)
        next_publish = 0.0
        while self.sim_time - start < seconds and time.monotonic() < deadline:
            now = time.monotonic()
            if vx is not None and now >= next_publish:
                command = Twist()
                command.linear.z = nonplanar
                command.linear.x = vx
                command.linear.y = lateral
                command.angular.z = yaw
                self.publisher.publish(command)
                next_publish = now + 0.02
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertGreaterEqual(self.sim_time - start, seconds)

    def policy(self, value):
        request = SetParametersAtomically.Request()
        parameter = Parameter('unsupported_command_policy', value=value)
        request.parameters = [parameter.to_parameter_msg()]
        return self.call(
            SetParametersAtomically, '/swerve_drive_controller/set_parameters_atomically',
            request).result.successful

    def test_driving_and_policies(self):
        self.sim_time = 0.0
        self.truth = self.odom = self.joints = None

        def clock(msg):
            self.sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec

        def truth(msg):
            # Only the root model pose should be published, never link transforms or duplicates.
            self.assertEqual(len(msg.transforms), 1)
            self.assertEqual(msg.transforms[0].child_frame_id, 'swervebot')
            self.truth = msg.transforms[0]

        subscriptions = [
            self.node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
            self.node.create_subscription(TFMessage, '/simulation/ground_truth', truth, 10),
            self.node.create_subscription(
                Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10),
            self.node.create_subscription(JointState, '/joint_states',
                                          lambda m: setattr(self, 'joints', m), 10),
        ]
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 1)
        self.wait_for(lambda: self.sim_time > 0 and self.truth is not None and
                      self.odom is not None and self.joints is not None)
        response = self.call(ListControllers, '/controller_manager/list_controllers',
                             ListControllers.Request())
        self.assertEqual({c.name for c in response.controller if c.state == 'active'},
                         {'swerve_controller', 'joint_state_broadcaster'})
        interfaces = self.call(
            ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces',
            ListHardwareInterfaces.Request())
        self.assertEqual(sum(i.is_claimed for i in interfaces.command_interfaces), 8)
        self.wait_for(lambda: self.publisher.get_subscription_count() > 0)
        self.run_for(0.5, 0.0)
        self.assertLess(abs(self.truth.transform.translation.z), 0.05)
        self.assertLess(abs(self.truth.transform.rotation.x), 0.05)
        self.assertLess(abs(self.truth.transform.rotation.y), 0.05)
        self.assertTrue(all(math.isfinite(value) for value in self.joints.position))

        before = self.pose()
        odom_before = self.odom.pose.pose.position.x
        self.run_for(1.5, 0.2)
        after = self.pose()
        displacement = (after[0] - before[0]) * math.cos(before[2]) + \
            (after[1] - before[1]) * math.sin(before[2])
        self.assertGreater(displacement, 0.15)
        self.assertLess(abs(after[1] - before[1]), 0.08)
        self.assertAlmostEqual(self.odom.pose.pose.position.x - odom_before,
                               after[0] - before[0], delta=0.12)
        self.run_for(0.7)  # No commands: the monotonic watchdog must stop the robot.
        stopped = self.pose()
        self.run_for(0.4)
        self.assertLess(math.hypot(self.pose()[0] - stopped[0], self.pose()[1] - stopped[1]), 0.03)

        before = self.pose()
        self.run_for(1.5, -0.2)
        self.assertLess((self.pose()[0] - before[0]) * math.cos(before[2]) +
                        (self.pose()[1] - before[1]) * math.sin(before[2]), -0.15)
        self.run_for(0.4, 0.0)
        for sign in (1.0, -1.0):
            before = self.pose()
            self.run_for(1.8, 0.2, sign * 0.3)
            yaw_delta = math.atan2(math.sin(self.pose()[2] - before[2]),
                                   math.cos(self.pose()[2] - before[2]))
            self.assertGreater(sign * yaw_delta, 0.2)
            angles = {name: self.joints.position[i] for i, name in enumerate(self.joints.name)}
            self.assertGreater(sign * angles['actuator_front_left_joint'], 0.0)
            self.assertLess(sign * angles['actuator_back_left_joint'], 0.0)
            self.run_for(0.4, 0.0)

        # The policy applies only to unsupported nonplanar components; all planar motion is valid.
        before = self.pose()
        self.run_for(0.6, 0.2, nonplanar=0.5)
        self.assertLess(math.hypot(self.pose()[0] - before[0], self.pose()[1] - before[1]), 0.03)
        self.assertTrue(self.policy('ignore'))
        before = self.pose()
        self.run_for(1.5, 0.2, nonplanar=0.5)
        distance = math.hypot(self.pose()[0] - before[0], self.pose()[1] - before[1])
        self.assertGreater(distance, 0.15)
        self.run_for(0.4, 0.0)

        def align(lateral=0.0, yaw=0.0):
            deadline = self.sim_time + 10.0
            while self.sim_time < deadline:
                self.run_for(0.25, 0.0, lateral=lateral, yaw=yaw)
                angles = {name: self.joints.position[i]
                          for i, name in enumerate(self.joints.name)}
                errors = []
                for corner, x, y in (('front_left', 0.13, 0.12),
                                     ('front_right', 0.13, -0.12),
                                     ('back_left', -0.13, 0.12),
                                     ('back_right', -0.13, -0.12)):
                    target = math.atan2(lateral + yaw * x, -yaw * y)
                    if target > math.pi / 2:
                        target -= math.pi
                    elif target < -math.pi / 2:
                        target += math.pi
                    difference = angles['actuator_' + corner + '_joint'] - target
                    errors.append(abs(math.atan2(math.sin(2 * difference),
                                                 math.cos(2 * difference))) / 2)
                if max(errors) < 0.03:
                    return
            self.fail('Steering did not align for constrained motion')

        # Measure steady motion only after measured steering reaches its targets.
        for lateral in (0.15, -0.15):
            align(lateral=lateral)
            before = self.pose()
            self.run_for(1.5, 0.0, lateral=lateral)
            after = self.pose()
            body_y = -(after[0] - before[0]) * math.sin(before[2]) + \
                (after[1] - before[1]) * math.cos(before[2])
            self.assertGreater(math.copysign(1.0, lateral) * body_y, 0.1)
            self.assertAlmostEqual(after[2], before[2], delta=0.1)
        for yaw in (0.3, -0.3):
            align(yaw=yaw)
            before = self.pose()
            self.run_for(1.5, 0.0, yaw=yaw)
            after = self.pose()
            delta = math.atan2(math.sin(after[2] - before[2]),
                               math.cos(after[2] - before[2]))
            self.assertGreater(math.copysign(1.0, yaw) * delta, 0.2)
            self.assertLess(math.hypot(after[0] - before[0], after[1] - before[1]), 0.08)
        self.run_for(0.4, 0.0)
        self.assertFalse(self.policy('invalid'))
        self.assertTrue(self.policy('stop'))
        self.run_for(0.3, float('nan'))
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(self.publisher)
