#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from orchestrator_interfaces.msg import SampleMessage, Status


class TSubscriber(Node):

    def __init__(self):
        super().__init__('t')  # type: ignore

        self.get_logger().info("TSubscriber starting")

        self.declare_parameter("processing_time", 0.2)
        self.processing_time = self.get_parameter("processing_time").get_parameter_value().double_value

        self.input_subscription_a = self.create_subscription(SampleMessage, "input1", self.input_callback_a, 10)
        self.get_logger().info(f"Subscribed to {self.input_subscription_a.topic_name}")
        self.input_subscription_b = self.create_subscription(SampleMessage, "input2", self.input_callback_b, 10)
        self.get_logger().info(f"Subscribed to {self.input_subscription_b.topic_name}")
        self.status_publisher = self.create_publisher(Status, "status", 10)

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def input_callback_a(self, msg: SampleMessage):
        self.get_logger().info(f"Received message for subscriber A on {self.input_subscription_a.topic_name}: {msg}")
        time.sleep(self.processing_time)
        self.get_logger().info(f"Publishing status")
        self.publish_status()

    def input_callback_b(self, msg: SampleMessage):
        self.get_logger().info(f"Received message for subscriber B on {self.input_subscription_b.topic_name}: {msg}")
        time.sleep(self.processing_time)
        self.get_logger().info(f"Publishing status")
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    node = TSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
