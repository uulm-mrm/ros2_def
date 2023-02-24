import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from orchestrator_interfaces.msg import Status
from example_interfaces.srv import AddTwoInts


class ServiceProviderNode(Node):

    def __init__(self) -> None:
        super().__init__('ServiceProviderNode')  # type: ignore
        self.input_subscription = self.create_subscription(String, "input", self.sub_callback, 10)
        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.srv = self.create_service(AddTwoInts, 'service', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def sub_callback(self, msg: String):
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    sub_node = ServiceProviderNode()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
