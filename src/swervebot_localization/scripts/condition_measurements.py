#!/usr/bin/env python3
"""Provide explicit covariance floors for publishers without uncertainty estimates."""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def covariance_floor(covariance, variances):
    """Preserve larger upstream uncertainties; never interpret zero as perfect certainty."""
    for index, variance in variances.items():
        covariance[index * 6 + index] = max(covariance[index * 6 + index], variance)


class Conditioner(Node):
    """Condition odometry outside the real-time controller without changing frames."""

    def __init__(self):
        super().__init__('condition_measurements')
        self.subscriptions_ = []
        self.declare_parameter('gyro_variance_floor', 0.0001)
        gyro_variance = self.get_parameter('gyro_variance_floor').value
        if not math.isfinite(gyro_variance) or gyro_variance <= 0:
            raise ValueError('gyro_variance_floor must be positive and finite')
        imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        def imu_callback(msg):
            values = (msg.orientation.x, msg.orientation.y, msg.orientation.z,
                      msg.orientation.w, msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z, *msg.orientation_covariance)
            if not all(math.isfinite(v) for v in values):
                return
            norm = sum(v * v for v in values[:4])
            if abs(norm - 1.0) > 0.01 or msg.orientation_covariance[0] < 0:
                return
            if msg.angular_velocity_covariance[0] < 0:
                return
            for index in (0, 4, 8):
                value = msg.angular_velocity_covariance[index]
                if not math.isfinite(value):
                    return
                msg.angular_velocity_covariance[index] = max(value, gyro_variance)
            imu_pub.publish(msg)

        self.subscriptions_.append(self.create_subscription(
            Imu, '/imu/data_filtered', imu_callback, qos_profile_sensor_data))
        publisher = self.create_publisher(Odometry, '/wheel/odometry_conditioned', 10)
        self.declare_parameter('wheel_variances', [0.04, 0.04, 0.01])
        variances = self.get_parameter('wheel_variances').value
        if len(variances) != 3 or not all(math.isfinite(v) and v > 0 for v in variances):
            raise ValueError('Wheel variance floors must contain three positive values')

        def wheel_callback(msg):
            covariance = msg.twist.covariance
            values = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.angular.z)
            if not all(math.isfinite(v) for v in (*covariance, *values)):
                return
            if msg.child_frame_id != 'base_footprint' or msg.header.frame_id != 'odom':
                self.get_logger().error('Odometry frame mismatch; dropping measurement',
                                        throttle_duration_sec=5.0)
                return
            covariance_floor(covariance, dict(zip((0, 1, 5), variances)))
            publisher.publish(msg)

        self.subscriptions_.append(self.create_subscription(
            Odometry, '/wheel/odometry', wheel_callback, 10))


def main():
    """Run conditioning with ROS-managed shutdown."""
    rclpy.init()
    node = Conditioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
