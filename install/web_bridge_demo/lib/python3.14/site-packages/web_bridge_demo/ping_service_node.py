"""Provide a simple connectivity check for WebSocket clients."""

import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class PingServiceNode(Node):
    """Respond to ping requests with the robot host name."""

    def __init__(self):
        super().__init__('ping_service_node')
        self._message = f'pong from {socket.gethostname()}'
        self._service = self.create_service(Trigger, '/web_ping', self._ping)

    def _ping(self, request, response):
        response.success = True
        response.message = self._message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PingServiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Release the service before closing the ROS context.
        node.destroy_node()
        rclpy.try_shutdown()
