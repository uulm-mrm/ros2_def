import rclpy
from rclpy.node import Node
from orchestrator_interfaces.msg import SampleMessage


class TestNode(Node):

    def __init__(self):
        super().__init__('ForwardingNode')  # type: ignore
        self.publisher = self.create_publisher(SampleMessage, "output", 10)
        self.create_subscription(SampleMessage, "input", self.callback, 10)

    def callback(self, msg: SampleMessage):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    test_node = TestNode()
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
