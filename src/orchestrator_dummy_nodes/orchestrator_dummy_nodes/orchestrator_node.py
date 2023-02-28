import datetime
import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.time import Time, Duration
    
from orchestrator.orchestrator_lib.orchestrator import Orchestrator
from orchestrator.orchestrator_lib.model_loader import *
from orchestrator.orchestrator_lib.ros_utils.spin import spin_for

from orchestrator_interfaces.msg import SampleMessage
from rosgraph_msgs.msg import Clock

from std_msgs.msg import String


def l(msg):
    return get_logger("l").info(msg)


class BagPlayer(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        self.get_logger().info(f"Orchestrator Node Starting!")

        self.t = Time(seconds=0, nanoseconds=0)

        launch_config = load_launch_config(
            "orchestrator_dummy_nodes",
            "service_test_launch_config.json",
            load_launch_config_schema())
        node_config = load_models(launch_config, load_node_config_schema())

        self.radar_publisher = self.create_publisher(SampleMessage, "meas/radar", 10)
        self.camera_publisher = self.create_publisher(SampleMessage, "meas/camera", 10)
        self.lidar_publisher = self.create_publisher(SampleMessage, "meas/lidar", 10)

        self.input_publisher = self.create_publisher(String, "i",10)

        self.clock_publisher = self.create_publisher(Clock, "clock", 10)

        self.orchestrator = Orchestrator(
            self,
            node_config,
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

        f = self.orchestrator.wait_until_publish_allowed("i")
        rclpy.spin_until_future_complete(self, f)
        msg = String()
        self.input_publisher.publish(msg)


        #spin_for(self, datetime.timedelta(seconds=1))
        self.t += Duration(seconds=5, nanoseconds=0)


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
