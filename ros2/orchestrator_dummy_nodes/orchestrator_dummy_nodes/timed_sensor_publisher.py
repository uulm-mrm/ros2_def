#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from std_msgs.msg import String
from orchestrator_interfaces.msg import SampleMessage


class TimedSensorPublisher(Node):

    def __init__(self):
        super().__init__('TimedSensorPublisher')  # type: ignore
        self.declare_parameter('timer_period_s', 0.1)
        self.declare_parameter('timer_uncertainty_s', 0.0)

        self.publisher = self.create_publisher(SampleMessage, "output", 10)  # type: ignore

        self.timer_period_s: float = self.get_parameter('timer_period_s').get_parameter_value().double_value  # type: ignore
        self.timer_uncertainty: float = self.get_parameter('timer_uncertainty_s').get_parameter_value().double_value  # type: ignore

        self.get_logger().info(f"Timer period: {self.timer_period_s}±{self.timer_uncertainty}s")

        self.base_timer = self.create_timer(self.timer_period_s, self.base_timer_callback)
        self.publish_timers: list[Timer] = []

    def base_timer_callback(self):
        next_interval = self.timer_period_s + random.uniform(-self.timer_uncertainty, self.timer_uncertainty)
        timer = self.create_timer(next_interval, self.timer_callback)
        self.publish_timers.append(timer)

    def timer_callback(self):
        assert (len(self.publish_timers) > 0)
        timer = self.publish_timers.pop(0)
        timer.destroy()
        msg = SampleMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.debug_data = 'Hello World:'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    timed_sensor_publisher = TimedSensorPublisher()

    rclpy.spin(timed_sensor_publisher)

    timed_sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
