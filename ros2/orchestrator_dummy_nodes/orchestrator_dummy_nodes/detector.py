#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from orchestrator_interfaces.msg import SampleMessage


class Detector(Node):

    def __init__(self):
        super().__init__('Detector')  # type: ignore

        self.get_logger().info("Detector starting")
        self.declare_parameter("processing_time", 0.05)
        self.processing_time = self.get_parameter("processing_time").get_parameter_value().double_value

        self.input_subscription = self.create_subscription(SampleMessage, "input", self.input_callback, 10)
        self.get_logger().info(f"Subscribed to {self.input_subscription.topic_name}")
        self.output_publisher = self.create_publisher(String, "output", 10)

    def input_callback(self, msg: SampleMessage):
        time.sleep(self.processing_time)
        output = String()
        output.data = self.get_name() + ": Detection from input: " + msg.debug_data
        self.get_logger().debug(f"Got input on topic {self.input_subscription.topic_name}, publishing output on topic {self.output_publisher.topic_name}")
        self.output_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    detector = Detector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass

    detector.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
