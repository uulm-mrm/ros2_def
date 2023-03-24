import rclpy
from rclpy.node import Node
from orchestrator_interfaces.msg import SampleMessage
from rclpy.time import Time, Duration
import numpy as np


class ForwardingTimingNode(Node):

    def __init__(self):
        super().__init__('TestNode')  # type: ignore
        self.publisher = self.create_publisher(SampleMessage, "output", 10)
        self.create_subscription(SampleMessage, "input", self.callback, 10)
        self.create_timer(0.02, self.timer_callback)
        self.create_timer(30.0, self.long_timer_callback)
        self.times: list[Duration] = []

    def timer_callback(self):
        msg = SampleMessage()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def long_timer_callback(self):
        times_s = np.array([d.nanoseconds * 1e-9 for d in self.times])
        mean = np.mean(times_s)
        var = np.var(times_s)
        self.get_logger().info(f"Mean: {mean}, Variance: {var}")
        #self.get_logger().info(f"{times_s.tolist()}")
        np.savetxt("times.csv", times_s)

    def callback(self, msg: SampleMessage):
        duration: Duration = self.get_clock().now()-Time.from_msg(msg.stamp)
        self.times.append(duration)
        self.get_logger().debug(f"Duration: {duration}")


def main(args=None):
    rclpy.init(args=args)

    test_node = ForwardingTimingNode()
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
