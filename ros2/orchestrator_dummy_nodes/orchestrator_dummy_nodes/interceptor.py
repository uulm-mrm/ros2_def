from typing import Optional
from rclpy.task import Future
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Interceptor(Node):

    def __init__(self):
        super().__init__('interceptor')
        self.declare_parameter('nr_topics', 6)
        nr_topics = self.get_parameter('nr_topics').get_parameter_value().integer_value

        self.all_subscriptions = [[self.create_subscription(
            String, f"topic_{i}",
            lambda msg, i=i: self.listener_callback(i, msg), 10)
            for i in range(nr_topics)]]
        self.all_publishers = [self.create_publisher(String, f"topic_{i}_intercepted", 10) for i in range(nr_topics)]
        self.status_subscription = self.create_subscription(String, "status", self.on_status, 10)
        self.message_buffer: dict[int, String] = {}
        self.next_publisher: int = 0

    def on_status(self, msg: String):
        self.next_publisher = int(msg.data)+1
        self.get_logger().info(f"{int(msg.data)} finished processing")
        self.callback()

    def listener_callback(self, i: int, msg: String):
        self.message_buffer[i] = msg
        self.get_logger().info(f"got message for {i}")
        self.callback()

    def callback(self):
        if self.next_publisher == 5:
            self.get_logger().info(f"all 5 finished processing, resetting")
            self.next_publisher = 0
            self.message_buffer = {}

        next_publisher = self.next_publisher
        if next_publisher in self.message_buffer:
            self.get_logger().info(f"We have message for {next_publisher}, and it is ready. sending.")
            msg = self.message_buffer[next_publisher]
            del self.message_buffer[next_publisher]
            self.all_publishers[next_publisher].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    interceptor = Interceptor()

    rclpy.spin(interceptor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interceptor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
