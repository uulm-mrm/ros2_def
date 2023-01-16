import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Detector(Node):

    def __init__(self):
        super().__init__('Detector')  # type: ignore

        self.declare_parameter("processing_time", 0.05)
        self.processing_time = self.get_parameter("processing_time").get_parameter_value().double_value

        self.input_subscription = self.create_subscription(String, "input", self.input_callback, 10)
        self.output_publisher = self.create_publisher(String, "output", 10)

    def input_callback(self, msg: String):
        time.sleep(self.processing_time)
        output = String()
        output.data = "Detection from input: "+msg.data
        self.output_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    detector = Detector()
    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
