#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlausibilityNode(Node):

    def __init__(self):
        super().__init__("plausibility")  # type: ignore
        self.track_pub = self.create_publisher(String, "tracks_out", 10)
        self.gridmap_tracks_pub = self.create_publisher(
            String, "tracks_out_gridmap", 10)
        self.gridmap_sub = self.create_subscription(
            String, "gridmap", self.gridmap_callback, 10)
        self.tracks_sub = self.create_subscription(
            String, "tracks_in", self.tracks_callback, 10)

    def gridmap_callback(self, msg: String):
        output = String()
        self.get_logger().debug("Got gridmap input, publishing tracks")
        output.data = f"Tracks extracted from gridmap input: {msg.data}"
        self.gridmap_tracks_pub.publish(output)

    def tracks_callback(self, msg):
        output = String()
        self.get_logger().debug("Got tracks input, publishing plausible tracks")
        output.data = f"Plausible tracks from input: {msg.data}"
        self.track_pub.publish(output)


def main():
    rclpy.init()
    node = PlausibilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
