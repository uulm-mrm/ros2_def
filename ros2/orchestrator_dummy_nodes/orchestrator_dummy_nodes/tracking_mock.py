#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from orchestrator_interfaces.msg import Status
from aduulm_messages.msg import DetectionList, ObjectList


class TrackingSubscriber(Node):

    def __init__(self):
        super().__init__('TrackingSubscriber')  # type: ignore
        self.status_publisher = self.create_publisher(Status, "status", 10)
        self.tracks_publisher = self.create_publisher(ObjectList, "tracks", 10)
        self.subs = []
        for (i, j) in [(0, 2), (1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (4, 1), (4, 2), (5, 1), (6, 1), (6, 2)]:
            sub = self.create_subscription(DetectionList, f"amqp/detections{i}_{j}", self.omit_tracks_cb, 10)
            self.subs.append(sub)

        self.subs.append(self.create_subscription(DetectionList, "amqp/detections0_1", self.output_tracks_cb, 10))

    def output_tracks_cb(self, msg):
        self.get_logger().info("Received input, publishing tracks")
        tracks_msg = ObjectList()
        tracks_msg.header.frame_id = "mec_fusion"
        self.tracks_publisher.publish(tracks_msg)

    def omit_tracks_cb(self, msg):
        self.get_logger().info("Received input, not publishing tracks")
        status_msg = Status()
        status_msg.node_name = self.get_name()
        status_msg.omitted_outputs = [self.tracks_publisher.topic_name]
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    tracking_subscriber = TrackingSubscriber()
    try:
        rclpy.spin(tracking_subscriber)
    except KeyboardInterrupt:
        pass

    tracking_subscriber.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
