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
        self.cached_camera = None
        self.cached_radar = None

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.get_name()
        self.status_publisher.publish(status_msg)

    def lidar_callback(self, msg: String):
        if self.cached_camera is None:
            self.get_logger().warn("Received lidar data without camera!")
            self.publish_status()
            return
        camera = self.cached_camera
        self.cached_camera = None

        if self.cached_radar is None:
            self.get_logger().warn("Received lidar data without radar!")
            self.publish_status()
            return
        radar = self.cached_radar
        self.cached_radar = None

        tracks = String()
        tracks.data = f"Tracks from inputs: [{camera.data}, {radar.data}, {msg.data}]"
        self.get_logger().info("Publishing tracks!")
        self.tracks_publisher.publish(tracks)

    def camera_callback(self, msg: String):
        if self.cached_camera is not None:
            self.get_logger().warn("Overwriting saved camera detection!")
        self.cached_camera = msg
        self.publish_status()

    def radar_callback(self, msg: String):
        if self.cached_radar is not None:
            self.get_logger().warn("Overwriting saved radar detection!")
        self.cached_radar = msg
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)

    tracking_subscriber = TrackingSubscriber()
    rclpy.spin(tracking_subscriber)

    tracking_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
