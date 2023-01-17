import rclpy
from rclpy.node import Node


class TestNode(Node):

    def __init__(self):
        super().__init__('TestNode')  # type: ignore
        self.timer = self.create_timer(3.0, self.callback)
        self.callback()

    def callback(self):
        try:
            self.get_logger().info(str(self.get_publisher_names_and_types_by_node("detector_camera", "/")))
            self.get_logger().info(str(self.get_publishers_info_by_topic("/detections/camera")[0].__str__))
        except:
            pass


def main(args=None):
    rclpy.init(args=args)

    test_node = TestNode()
    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
