#!/usr/bin/env python3

import datetime
import time
from typing import Literal

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time, Duration

from orchestrator.orchestrator_lib.orchestrator import Orchestrator
from orchestrator.orchestrator_lib.model_loader import *
from orchestrator.orchestrator_lib.ros_utils.spin import spin_for

from orchestrator_interfaces.msg import SampleMessage
from rosgraph_msgs.msg import Clock


def l(msg):
    return get_logger("l").info(msg)


class BagPlayer(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        self.get_logger().info(f"Orchestrator Node Starting!")

        self.t = Time(seconds=0, nanoseconds=0)

        self.declare_parameter('mode', '')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        self.mode: Literal["tracking", "service", "time_sync"]
        if mode == "tracking":
            self.mode = "tracking"
            launch_config = load_launch_config(
                "orchestrator_dummy_nodes",
                "tracking_example_launch_config.json",
                load_launch_config_schema())

            self.radar_publisher = self.create_publisher(
                SampleMessage, "meas/radar", 10)
            self.camera_publisher = self.create_publisher(
                SampleMessage, "meas/camera", 10)
            self.lidar_publisher = self.create_publisher(
                SampleMessage, "meas/lidar", 10)
        elif mode == "service":
            self.mode = "service"
            self.input_publisher = self.create_publisher(
                SampleMessage, "i", 10)
            launch_config = load_launch_config(
                "orchestrator_dummy_nodes",
                "service_test_launch_config.json",
                load_launch_config_schema())
        elif mode == "time_sync":
            self.mode = "time_sync"
            self.camera_info_publisher = self.create_publisher(
                SampleMessage, "camera_info", 10)
            self.image_publisher = self.create_publisher(
                SampleMessage, "image", 10)
            launch_config = load_launch_config(
                "orchestrator_dummy_nodes",
                "time_sync_test_launch_config.json",
                load_launch_config_schema()
            )
        else:
            self.get_logger().fatal(f"Unknown mode: {mode}")
            exit(1)

        self.get_logger().info(f"Mode is {self.mode}")

        self.clock_publisher = self.create_publisher(Clock, "clock", 10)

        node_config = load_models(launch_config, load_node_config_schema())

        self.orchestrator = Orchestrator(
            self,
            node_config,
            logger=get_logger("l"))
        self.orchestrator.initialize_ros_communication()

    def publish_lidar(self):
        msg = SampleMessage()
        msg.debug_data = "lidar measurement"
        self.lidar_publisher.publish(msg)

    def publish_radar(self):
        msg = SampleMessage()
        msg.debug_data = "radar measurement"
        self.radar_publisher.publish(msg)

    def publish_camera(self):
        msg = SampleMessage()
        msg.debug_data = "camera measurement"
        self.camera_publisher.publish(msg)

    def publish_time(self):
        msg = Clock()
        msg.clock = self.t.to_msg()
        self.clock_publisher.publish(msg)

    def timestep_service(self):

        f = self.orchestrator.wait_until_time_publish_allowed(self.t)
        rclpy.spin_until_future_complete(self, f)
        self.get_logger().info(f"Timestep {self.t}!")
        self.publish_time()

        f = self.orchestrator.wait_until_publish_allowed("i")
        rclpy.spin_until_future_complete(self, f)
        msg = SampleMessage()
        self.input_publisher.publish(msg)

        spin_for(self, datetime.timedelta(seconds=0.5))
        self.t += Duration(seconds=3, nanoseconds=0)

    def timestep_tracking(self):
        f = self.orchestrator.wait_until_time_publish_allowed(self.t)
        rclpy.spin_until_future_complete(self, f)
        self.get_logger().info(f"Timestep {self.t}!")
        self.publish_time()

        if self.t.nanoseconds % 10 ** 9 == 0:
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

        spin_for(self, datetime.timedelta(seconds=0.1))
        self.t += Duration(seconds=0, nanoseconds=100_000_000)

    def timestep_time_sync(self):
        f = self.orchestrator.wait_until_time_publish_allowed(self.t)
        rclpy.spin_until_future_complete(self, f)
        self.publish_time()

        ci_msg = SampleMessage()
        ci_msg.header.stamp = self.t.to_msg()
        f = self.orchestrator.wait_until_publish_allowed("camera_info")
        rclpy.spin_until_future_complete(self, f)
        self.camera_info_publisher.publish(ci_msg)

        if self.t.nanoseconds % 10 ** 9 == 0:
            image_msg = SampleMessage()
            image_msg.header.stamp = self.t.to_msg()
            f = self.orchestrator.wait_until_publish_allowed("image")
            rclpy.spin_until_future_complete(self, f)
            self.image_publisher.publish(image_msg)

        spin_for(self, datetime.timedelta(seconds=0.3))
        self.t += Duration(seconds=0, nanoseconds=100_000_000)

    def timestep(self):
        if self.mode == "tracking":
            self.timestep_tracking()
        elif self.mode == "service":
            self.timestep_service()
        elif self.mode == "time_sync":
            self.timestep_time_sync()


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
