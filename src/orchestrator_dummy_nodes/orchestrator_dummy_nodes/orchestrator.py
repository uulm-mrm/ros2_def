# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


import time

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from .orchestrator_lib.orchestrator import Orchestrator

# from orchestrator_dummy_nodes.tracking_example_configuration import
from orchestrator_dummy_nodes.tracking_example_configuration import \
    external_input_topics as external_input_topics_config, \
    nodes as node_config, \
    output_topics


def l(msg):
    return get_logger("l").info(msg)


class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        l(f"Orchestrator Node Starting!")
        self.orchestrator = Orchestrator(self, node_config, external_input_topics_config, output_topics, logger=get_logger("l"))
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
