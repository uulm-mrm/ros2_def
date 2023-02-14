import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from .orchestrator_lib.orchestrator import Orchestrator

from orchestrator_dummy_nodes.tracking_example_configuration import \
    external_input_topics as external_input_topics_config, \
    output_topics

from orchestrator.model_loader import *


def l(msg):
    return get_logger("l").info(msg)


class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        l(f"Orchestrator Node Starting!")

        launch_config = load_launch_config(
            "orchestrator_dummy_nodes",
            "tracking_example_launch_config.json",
            load_launch_config_schema())
        node_config = load_models(launch_config, load_node_config_schema())

        self.orchestrator = Orchestrator(
            self,
            node_config,
            external_input_topics_config,
            output_topics,
            logger=get_logger("l"))
        time.sleep(3)
        self.orchestrator.initialize_ros_communication()


def main():
    rclpy.init()
    orchestrator = OrchestratorNode()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
