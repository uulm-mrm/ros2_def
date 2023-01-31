# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


from dataclasses import dataclass
from enum import Enum
import time
from typing import Any, Callable, TypeAlias
import networkx as nx

from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish
from orchestrator_dummy_nodes.topic_remapping import collect_intercepted_topics, type_from_string
from orchestrator_dummy_nodes.tracking_example_configuration import nodes as node_config
from orchestrator_dummy_nodes.tracking_example_configuration import external_input_topics as external_input_topics_config

from rclpy.node import Node, Subscription, Publisher
import rclpy
from rclpy.logging import get_logger

from orchestrator_interfaces.msg import Status


TopicName: TypeAlias = str
NodeName: TypeAlias = str


class ActionState(Enum):
    WAITING = 1
    READY = 2
    RUNNING = 3
    DONE = 4


@dataclass
class RxAction:
    state: ActionState
    node: str
    topic: str
    data: Any | None = None


def next_id() -> int:
    n = next_id.value
    next_id.value += 1
    return n


next_id.value = 0


def l(msg):
    return get_logger("l").info(msg)


def lc(msg):
    return get_logger("l").info('\033[96m' + msg + '\033[0m')


def d(msg):
    return get_logger("l").debug(msg)


class Orchestrator(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        l(f"Starting!")
        time.sleep(3)

        # For now, the node models are hardcoded.
        # In the future, these could for example be loaded dynamically
        # for each node which is connected to an intercepted topic.
        self.node_models: list[NodeModel] = node_config

        self.add_input()

        # Subscription for each topic (topic_name -> sub)
        self.interception_subs: dict[TopicName, Subscription] = {}
        # Publisher for each node (node_name -> topic_name -> pub)
        self.interception_pubs: dict[NodeName, dict[TopicName, Publisher]] = {}

        self.external_input_topics = external_input_topics_config
        # self.external_input_subs: list[Subscription] = []

        # Subscriptions for outputs which we do not need to buffer,
        #  but we need to inform the node models that they happened
        self.modeled_node_output_subs: dict[TopicName, Subscription] = {}

        self.graph: nx.DiGraph = nx.DiGraph()

        for node, canonical_name, intercepted_name, type in collect_intercepted_topics(self.get_topic_names_and_types()):
            l(f"Intercepted input \"{canonical_name}\" of type {type.__name__} from node \"{node}\" as \"{intercepted_name}\"")
            # Subscribe to the input topic
            if canonical_name not in self.interception_subs:
                l(f" Subscribing to {canonical_name}")
                subscription = self.create_subscription(
                    type, canonical_name,
                    lambda msg, topic_name=canonical_name: self.interception_subscription_callback(topic_name, msg),
                    10)
                self.interception_subs[canonical_name] = subscription
            else:
                l(" Subscription already exists")
            # Create separate publisher for each node
            l(f" Creating publisher for {intercepted_name}")
            publisher = self.create_publisher(type, intercepted_name, 10)
            if node not in self.interception_pubs:
                self.interception_pubs[node] = {}
            self.interception_pubs[node][canonical_name] = publisher

        self.status_subscription = self.create_subscription(Status, "status", self.status_callback, 10)

    def add_input(self):
        camera_input = TopicInput("meas/camera")
        lidar_input = TopicInput("meas/lidar")
        radar_input = TopicInput("meas/radar")

        expected_rx_actions = []

        for node in self.node_models:
            for input in [camera_input, lidar_input, radar_input]:
                if input in node.get_possible_inputs():
                    expected_rx_actions.append(RxAction(ActionState.WAITING, node.get_name(), input.input_topic))
        l(f"{expected_rx_actions}")

        for action in expected_rx_actions:
            self.add_action_and_effects(action)

    def add_action_and_effects(self, action: RxAction, parent: int | None = None):
        node_id = next_id()
        self.graph.add_node(node_id, action=action)

        # Sibling connections: Complete all actions at this node before the to-be-added action
        for node, node_data in self.graph.nodes(data=True):
            other_action: RxAction = node_data["data"]  # type: ignore
            if other_action.node == action.node and node != node_id:
                self.graph.add_edge(node_id, node)

        if parent is not None:
            self.graph.add_edge(node_id, parent)

        topic_input_cause = TopicInput(action.topic)
        node_model = self.node_model_by_name(action.node)
        assert topic_input_cause in node_model.get_possible_inputs()
        effects = node_model.effects_for_input(topic_input_cause)

        # Recursively add action nodes for publish events in the current action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                resulting_input = TopicInput(effect.output_topic)
                for node in self.node_models:
                    if resulting_input in node.get_possible_inputs():
                        self.add_action_and_effects(RxAction(ActionState.WAITING, node.get_name(), resulting_input.input_topic), node_id)

    def interception_subscription_callback(self, topic_name: TopicName, msg: Any):
        lc(f"Received message on intercepted topic {topic_name}")

        # For expected/arriving input, create receiving actions (waiting) for each initial subscriber
        # Recursively, for each added action, add successor actions. Add parent as dependency
        # If actions for the same node already exist, add as dependencies (this ensures sequence-determinism at each node)
        #
        # To advance processing, execute all actions which are ready and have no dependencies (setting it to running)
        #
        # For incoming messages, set actions requiring it to ready, and remove the (running) action causing it

    def node_model_by_name(self, name) -> NodeModel:
        for node in self.node_models:
            if node.get_name() == name:
                return node

        raise KeyError(f"No model for node \"{name}\"")

    def status_callback(self, msg: Status):
        lc(f"Received status message from node {msg.node_name}")


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
