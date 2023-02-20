import rclpy
from rclpy.node import Node

from orchestrator_interfaces.msg import SampleMessage


class SimpleTimerPublisher(Node):

    def __init__(self):
        super().__init__('SimpleTimerPublisher')  # type: ignore
        self.declare_parameter('timer_period_s', 0.1)

        self.publisher = self.create_publisher(SampleMessage, "output", 10)  # type: ignore

        self.timer_period_s: float = self.get_parameter('timer_period_s').get_parameter_value().double_value  # type: ignore
        self.get_logger().info(f"Timer period: {self.timer_period_s}s")

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Timer callback at time {self.get_clock().now()}, "
                               f"Time since last: {self.timer.time_since_last_call()}, "
                               f"Time until next: {self.timer.time_until_next_call()}")
        msg = SampleMessage()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    timed_sensor_publisher = SimpleTimerPublisher()

    try:
        rclpy.spin(timed_sensor_publisher)
    except KeyboardInterrupt:
        pass

    timed_sensor_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
