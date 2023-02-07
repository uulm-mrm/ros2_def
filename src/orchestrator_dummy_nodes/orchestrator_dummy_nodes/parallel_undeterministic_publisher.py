import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32


class PUPublisher(Node):

    def __init__(self):
        super().__init__('parallel_undeterministic_publisher')  # type: ignore
        self.declare_parameter('nr_publishers', 6)
        nr_publishers = self.get_parameter('nr_publishers').get_parameter_value().integer_value

        self.declare_parameter('in_order', False)
        self.declare_parameter('use_timer', True)

        self.all_publishers = [self.create_publisher(String, f"topic_{i}", 10) for i in range(nr_publishers)]
        if self.get_parameter('use_timer').get_parameter_value().bool_value:
            timer_period = 0.5
            self.timer = self.create_timer(timer_period, self.timer_callback)

        self.trigger_subscriber = self.create_subscription(String, f"trigger_sample", lambda _msg: self.timer_callback(), 10)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        # Publish on all publishers in random order

        if self.get_parameter('in_order').get_parameter_value().bool_value:
            for publisher in self.all_publishers:
                publisher.publish(msg)
        else:
            for publisher in sorted(self.all_publishers, key=lambda _: random.random()):
                publisher.publish(msg)
        self.get_logger().info(f"Published \"{msg.data}\" on {len(self.all_publishers)} publishers")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PUPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
