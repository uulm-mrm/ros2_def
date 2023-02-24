import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from orchestrator_interfaces.msg import Status
from example_interfaces.srv import AddTwoInts


class ServiceCallerNode(Node):

    def __init__(self) -> None:
        super().__init__('ServiceProviderNode')  # type: ignore
        self.input_subscription = self.create_subscription(String, "input", self.sub_callback, 10)
        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.cli = self.create_client(AddTwoInts, 'service')

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def sub_callback(self, msg: String):
        req = AddTwoInts.Request()
        req.a = 1
        req.b = 2
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    sub_node = ServiceCallerNode()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
