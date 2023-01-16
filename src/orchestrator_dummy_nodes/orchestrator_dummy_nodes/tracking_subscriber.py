import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from orchestrator_interfaces.msg import Status


class TrackingSubscriber(Node):

    def __init__(self):
        super().__init__('TrackingSubscriber')  # type: ignore
        self.camera_subscription = self.create_subscription(String, "input_camera", self.camera_callback, 10)
        self.lidar_subscription = self.create_subscription(String, "input_lidar", self.lidar_callback, 10)
        self.radar_subscription = self.create_subscription(String, "input_radar", self.radar_callback, 10)
        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.tracks_publisher = self.create_publisher(String, "tracks", 10)

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def camera_callback(self, msg: String):
        self.publish_status()

    def lidar_callback(self, msg: String):
        self.publish_status()

    def radar_callback(self, msg: String):
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    tracking_subscriber = TrackingSubscriber()
    rclpy.spin(tracking_subscriber)

    tracking_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
