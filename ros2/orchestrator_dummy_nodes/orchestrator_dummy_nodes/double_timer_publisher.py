#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from orchestrator_interfaces.msg import SampleMessage


class DoubleTimerPublisher(Node):

    def __init__(self):
        super().__init__('DoubleTimerPublisher')  # type: ignore
        self.declare_parameter('timer_a_period_s', 0.1)
        self.declare_parameter('timer_b_period_s', 0.3)

        self.publisher_a = self.create_publisher(SampleMessage, "output_a", 10)  # type: ignore
        self.publisher_b = self.create_publisher(SampleMessage, "output_b", 10)  # type: ignore

        self.timer_a_period_s: float = self.get_parameter(
            'timer_a_period_s').get_parameter_value().double_value  # type: ignore
        self.timer_b_period_s: float = self.get_parameter(
            'timer_b_period_s').get_parameter_value().double_value  # type: ignore
        self.get_logger().info(f"Timer A period: {self.timer_a_period_s}s")
        self.get_logger().info(f"Timer B period: {self.timer_b_period_s}s")

        self.timer_a = self.create_timer(self.timer_a_period_s, self.timer_a_callback)
        self.timer_b = self.create_timer(self.timer_b_period_s, self.timer_b_callback)

    def timer_a_callback(self):
        self.get_logger().info(f"Timer A callback at time {self.get_clock().now()}, "
                               f"Time since last: {self.timer_a.time_since_last_call()}, "
                               f"Time until next: {self.timer_a.time_until_next_call()}")
        msg = SampleMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_a.publish(msg)

    def timer_b_callback(self):
        self.get_logger().info(f"Timer B callback at time {self.get_clock().now()}, "
                               f"Time since last: {self.timer_b.time_since_last_call()}, "
                               f"Time until next: {self.timer_b.time_until_next_call()}")
        msg = SampleMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_b.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    timed_sensor_publisher = DoubleTimerPublisher()

    try:
        rclpy.spin(timed_sensor_publisher)
    except KeyboardInterrupt:
        pass

    timed_sensor_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
