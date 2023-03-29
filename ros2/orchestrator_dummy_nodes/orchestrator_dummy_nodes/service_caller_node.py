#!/usr/bin/env python3

import random
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from orchestrator_interfaces.msg import Status, SampleMessage
from orchestrator_interfaces.srv import SampleService


class ServiceCallerNode(Node):
    """Calls a service within [0,20]ms in topic-input callback"""

    def __init__(self) -> None:
        super().__init__('ServiceCallerNode')  # type: ignore
        self.input_subscription = self.create_subscription(
            SampleMessage, "input", self.sub_callback, 10)
        self.status_publisher = self.create_publisher(Status, "status", 10)
        service_callback_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(
            SampleService, 'service', callback_group=service_callback_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def sub_callback(self, msg: SampleMessage):
        req = SampleService.Request()
        req.caller = self.get_name()
        time.sleep(random.uniform(0, 0.02))
        self.get_logger().info("Calling service")
        res = self.cli.call(req)
        self.get_logger().info(f"Got response: {res}")
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    sub_node = ServiceCallerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(sub_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    sub_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
