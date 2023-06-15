#!/usr/bin/env python3

import time
import random
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
        self.declare_parameter("processing_time_range", 0.0)
        self.declare_parameter("queue_size", 10)
        self.processing_time = self.get_parameter("processing_time").get_parameter_value().double_value
        self.processing_time_range = self.get_parameter("processing_time_range").get_parameter_value().double_value
        queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value

        self.input_subscription = self.create_subscription(SampleMessage, "input", self.input_callback, queue_size)
        self.get_logger().info(f"Subscribed to {self.input_subscription.topic_name}")
        self.output_publisher = self.create_publisher(SampleMessage, "output", 10)

    def input_callback(self, msg: SampleMessage):
        output = SampleMessage()
        output.header = msg.header
        output.debug_data = self.get_name() + ": Detection from input: " + msg.debug_data
        now = self.get_clock().now()
        pub = Time.from_msg(msg.header.stamp)
        age = now - pub
        age_ms = age.nanoseconds * 1e-6

        processing_time = self.processing_time
        processing_time += random.uniform(-self.processing_time_range / 2, self.processing_time_range)

        self.get_logger().info(f"Got input on topic {self.input_subscription.topic_name}: {msg.debug_data}, "
                               f"age {age_ms}ms, "
                               f"processing for {processing_time}s")
        time.sleep(processing_time)
        self.get_logger().info(f"publishing output on topic {self.output_publisher.topic_name}")
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
