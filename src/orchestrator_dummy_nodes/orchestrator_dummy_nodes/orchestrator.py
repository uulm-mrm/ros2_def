import datetime
import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time, Duration

from .orchestrator_lib.orchestrator import Orchestrator

from orchestrator_dummy_nodes.tracking_example_configuration import \
    external_input_topics as external_input_topics_config, \
    output_topics

from orchestrator.model_loader import *

from orchestrator_interfaces.msg import SampleMessage
from rosgraph_msgs.msg import Clock


def l(msg):
    return get_logger("l").info(msg)


def spin_for(node, duration):
    start = datetime.datetime.now()
    while datetime.datetime.now() - start < duration:
        remaining = datetime.datetime.now() - start
        rclpy.spin_once(node, timeout_sec=remaining.total_seconds())


class BagPlayer(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        l(f"Orchestrator Node Starting!")

        self.t = Time(seconds=0, nanoseconds=0)

        launch_config = load_launch_config(
            "orchestrator_dummy_nodes",
            "tracking_example_launch_config.json",
            load_launch_config_schema())
        node_config = load_models(launch_config, load_node_config_schema())

        self.radar_publisher = self.create_publisher(SampleMessage, "meas/radar", 10)
        self.camera_publisher = self.create_publisher(SampleMessage, "meas/camera", 10)
        self.lidar_publisher = self.create_publisher(SampleMessage, "meas/lidar", 10)

        self.clock_publisher = self.create_publisher(Clock, "clock", 10)

        self.orchestrator = Orchestrator(
            self,
            node_config,
            external_input_topics_config,
            output_topics,
            logger=get_logger("l"))
        time.sleep(3)
        self.orchestrator.initialize_ros_communication()

    def publish_lidar(self):
        msg = SampleMessage()
        self.lidar_publisher.publish(msg)

    def publish_radar(self):
        msg = SampleMessage()
        self.radar_publisher.publish(msg)

    def publish_camera(self):
        msg = SampleMessage()
        self.camera_publisher.publish(msg)

    def publish_time(self):
        msg = Clock()
        msg.clock = self.t.to_msg()
        self.clock_publisher.publish(msg)

    def timestep(self):

        f = self.orchestrator.wait_until_time_publish_allowed(self.t)
        rclpy.spin_until_future_complete(self, f)
        self.get_logger().info(f"Timestep {self.t}!")
        self.publish_time()

        if self.t.nanoseconds % 10**9 == 0:
            # Publish sensors once per second
            f = self.orchestrator.wait_until_publish_allowed("meas/lidar")
            rclpy.spin_until_future_complete(self, f)
            self.publish_lidar()

            f = self.orchestrator.wait_until_publish_allowed("meas/radar")
            rclpy.spin_until_future_complete(self, f)
            self.publish_radar()

            f = self.orchestrator.wait_until_publish_allowed("meas/camera")
            rclpy.spin_until_future_complete(self, f)
            self.publish_camera()

        spin_for(self, datetime.timedelta(seconds=3.0))
        self.t += Duration(seconds=0, nanoseconds=100_000_000)


def main():
    rclpy.init()
    orchestrator = BagPlayer()

    try:
        while True:
            orchestrator.timestep()
    except KeyboardInterrupt:
        pass

    orchestrator.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
