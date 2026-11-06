"""Verify the public ping service contract through ROS."""

import socket

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from web_bridge_demo.ping_service_node import PingServiceNode


def test_ping():
    rclpy.init()
    server = PingServiceNode()
    client_node = rclpy.create_node('ping_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    try:
        client = client_node.create_client(Trigger, '/web_ping')
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=10)
        assert future.done()
        response = future.result()
        assert response.success
        assert response.message == f'pong from {socket.gethostname()}'
    finally:
        executor.shutdown()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.try_shutdown()
