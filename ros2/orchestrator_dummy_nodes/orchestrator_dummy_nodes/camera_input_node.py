import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from orchestrator_interfaces.msg import SampleMessage, Status
import message_filters


class CameraInputNode(Node):

    def __init__(self):
        super().__init__('CameraInputNode')  # type: ignore

        self.declare_parameter("processing_time", 0.05)
        self.processing_time = self.get_parameter(
            "processing_time").get_parameter_value().double_value

        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.camera_info_sub = message_filters.Subscriber(
            self, SampleMessage, "camera_info")
        self.image_sub = message_filters.Subscriber(
            self, SampleMessage, "image")
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.camera_info_sub, self.image_sub],
            4,
            0.1)
        self.time_sync.registerCallback(self.input_callback)
        self.camera_info_sub.registerCallback(self.send_status_callback)
        self.image_sub.registerCallback(self.send_status_callback)

        self.output_publisher = self.create_publisher(
            SampleMessage, "output", 10)

    def input_callback(self, camera_info_msg: SampleMessage, image_msg: SampleMessage):
        time.sleep(self.processing_time)
        output = SampleMessage()
        output.header = image_msg.header
        output.debug_data = f"{self.get_name()}: Output from inputs at: " + \
            f"{Time.from_msg(camera_info_msg.header.stamp)} and {Time.from_msg(image_msg.header.stamp)}"
        self.get_logger().info("Got input, publishing output")
        self.output_publisher.publish(output)

    def send_status_callback(self, msg):
        msg = Status()
        msg.node_name = self.get_name()
        self.get_logger().info("publishing status")
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    detector = CameraInputNode()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass

    detector.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
