#!/usr/bin/env python3
import hashlib

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
        self.hash_state = hashlib.md5()

        self.declare_parameter("inputs")
        input_topics = self.get_parameter("inputs").get_parameter_value().string_array_value

        for it in input_topics:
            self.subs.append(self.create_subscription(
                DetectionList, it,
                lambda msg, topic_name=it: self.omit_tracks_cb(msg, topic_name),
                10))

    def output_tracks_cb(self, msg):
        self.get_logger().info("Received input, publishing tracks")
        tracks_msg = ObjectList()
        tracks_msg.header.frame_id = "mec_fusion"
        self.tracks_publisher.publish(tracks_msg)

    def omit_tracks_cb(self, msg, topic):
        self.get_logger().info(f"Received input on {topic}, not publishing tracks. Message: {msg}")
        self.hash_state.update(str.encode(topic))
        self.hash_state.update(str.encode(f"{msg}"))

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

    tracking_subscriber.get_logger().fatal(
        f"Final state of node {tracking_subscriber.get_name()}: {tracking_subscriber.hash_state.hexdigest()}")

    tracking_subscriber.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
