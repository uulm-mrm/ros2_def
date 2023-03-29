#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from orchestrator_interfaces.msg import Status, SampleMessage
from orchestrator_interfaces.srv import SampleService


class ServiceProviderNode(Node):

    def __init__(self) -> None:
        super().__init__('ServiceProviderNode')  # type: ignore
        self.input_subscription = self.create_subscription(
            SampleMessage, "input", self.sub_callback, 10)
        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.srv = self.create_service(
            SampleService, 'service', self.add_two_ints_callback)
        self.last_caller = ""

    def add_two_ints_callback(self, request: SampleService.Request, response):
        self.get_logger().info(f"Incoming request: {request}")
        if self.last_caller == request.caller:
            self.get_logger().error("TWO REQUESTS FROM SAME CALLER IN A ROW")
        self.last_caller = request.caller
        return response

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def sub_callback(self, msg: SampleMessage):
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    sub_node = ServiceProviderNode()
    try:
        rclpy.spin(sub_node)
    except KeyboardInterrupt:
        pass

    sub_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
