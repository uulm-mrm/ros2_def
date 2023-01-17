# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


from orchestrator_dummy_nodes.topic_remapping import collect_intercepted_topics
from orchestrator_dummy_nodes.tracking_example_configuration import nodes as node_config
from rclpy.node import Node
import rclpy
from rclpy.logging import get_logger

# Buffered message = multiple causes (one per subscription/node)


def l(msg):
    return get_logger("l").info(msg)


class Orchestrator(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        for node, canonical_name, intercepted_name, type in collect_intercepted_topics(self.get_topic_names_and_types()):
            l(f"Intercepted input \"{canonical_name}\" of type {type.__name__} from node \"{node}\" as \"{intercepted_name}\"")
            # TODO: create subscription, publisher


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
