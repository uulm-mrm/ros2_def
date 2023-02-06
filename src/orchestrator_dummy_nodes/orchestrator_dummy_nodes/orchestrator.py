# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


from dataclasses import dataclass
from enum import Enum
import time
from typing import Any, Callable, TypeAlias, cast
import networkx as nx
import matplotlib.pyplot as plt
import netgraph


from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish
from orchestrator_dummy_nodes.topic_remapping import collect_intercepted_topics, type_from_string
# from orchestrator_dummy_nodes.tracking_example_configuration import
from orchestrator_dummy_nodes.tracking_example_configuration import external_input_topics as external_input_topics_config, nodes as node_config, output_topics

from rclpy.node import Node, Subscription, Publisher
import rclpy
from rclpy.logging import get_logger

from orchestrator_interfaces.msg import Status


TopicName: TypeAlias = str
NodeName: TypeAlias = str


class ActionState(Enum):
    WAITING = 1  # Waiting for data, no message buffered
    READY = 2  # Data is buffered, ready to execute once allowed by ordering constraints
    RUNNING = 3  # Running, expecting output as specified by model


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


class ActionNotFoundError(Exception):
    pass


class Orchestrator(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore
        l(f"Starting!")
        time.sleep(3)

        # For now, the node models are hardcoded.
        # In the future, these could for example be loaded dynamically
        # for each node which is connected to an intercepted topic.
        self.node_models: list[NodeModel] = node_config

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

        self.timestep: int = 0
        self.add_input(self.timestep)
        for _i in range(11):
            self.timestep += 1
            self.add_input(self.timestep)

        labels = {}
        for node, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            labels[node] = f"{d.node}: rx {d.topic}"

        color_map = {
            "same-node": "tab:green",
            "same-topic": "tab:orange",
            "causality": "tab:blue"
        }
        edge_colors = {}
        for u, v, edge_data in self.graph.edges(data=True):  # type: ignore
            edge_type = edge_data["edge_type"]  # type: ignore
            edge_colors[(u, v)] = color_map[edge_type]

        edge_proxy_artists = []
        for name, color in color_map.items():
            proxy = plt.Line2D(  # type: ignore
                [], [],
                linestyle='-',
                color=color,
                label=name
            )
            edge_proxy_artists.append(proxy)

        fig = plt.figure()
        ax: plt.Axes = fig.subplots()  # type: ignore
        edge_legend = ax.legend(handles=edge_proxy_artists, loc='upper right', title='Edges')
        ax.add_artist(edge_legend)

        node_to_community = {}
        for node, node_data in self.graph.nodes(data=True):
            timestep: int = node_data["timestep"]  # type: ignore
            node_to_community[node] = timestep

        plot_instance = netgraph.InteractiveGraph(self.graph,
                                                  node_layout='community',
                                                  node_layout_kwargs=dict(node_to_community=node_to_community),
                                                  annotations=labels,
                                                  arrows=True,
                                                  edge_color=edge_colors,
                                                  ax=ax)
        for artist in plot_instance.artist_to_annotation:
            placement = plot_instance._get_annotation_placement(artist)
            plot_instance._add_annotation(artist, *placement)

        # plt.show()

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

        # Subscribe to outputs (tracking example: plausibility node publishers)
        self.output_subs: list[Subscription] = []
        for MessageType, topic_name in output_topics:
            l(f"Subscribing to output topic: {topic_name}")
            sub = self.create_subscription(
                MessageType,
                topic_name,
                lambda msg, topic_name=topic_name: self.interception_subscription_callback(topic_name, msg),
                10)
            self.output_subs.append(sub)

        self.status_subscription = self.create_subscription(Status, "status", self.status_callback, 10)

    def add_input(self, timestep):
        inputs = [TopicInput(top) for (_type, top) in external_input_topics_config]
        lc(f"Adding inputs on {len(inputs)} topics")

        expected_rx_actions = []

        for node in self.node_models:
            for input in inputs:
                if input in node.get_possible_inputs():
                    expected_rx_actions.append(RxAction(ActionState.WAITING, node.get_name(), input.input_topic))

        l(f" Those inputs cause {len(expected_rx_actions)} rx actions")
        for action in expected_rx_actions:
            self.add_action_and_effects(action, timestep)

    def add_action_and_effects(self, action: RxAction, timestep: int,  parent: int | None = None):
        # Parent: Node ID of the action causing this topic-publish. Should only be None for inputs
        node_id = next_id()
        self.graph.add_node(node_id, data=action, timestep=timestep)

        # Sibling connections: Complete all actions at this node before the to-be-added action
        for node, node_data in self.graph.nodes(data=True):
            other_action: RxAction = node_data["data"]  # type: ignore
            if other_action.node == action.node and node != node_id:
                self.graph.add_edge(node_id, node, edge_type="same-node")

        if parent is not None:
            self.graph.add_edge(node_id, parent, edge_type="causality")

        topic_input_cause = TopicInput(action.topic)
        node_model = self.node_model_by_name(action.node)
        assert topic_input_cause in node_model.get_possible_inputs()
        effects = node_model.effects_for_input(topic_input_cause)

        # Multi-Publisher Connections:
        # Add edge to all RxActions for the topics that are published by this action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                # Add edge to all RxActions for this topic.
                # Not doing this would allow concurrent publishing on the same topic, which results in undeterministic receive order
                for node, node_data in self.graph.nodes(data=True):
                    other_action: RxAction = node_data["data"]  # type: ignore
                    if other_action.topic == effect.output_topic:
                        self.graph.add_edge(node_id, node, edge_type="same-topic")

        # Recursively add action nodes for publish events in the current action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                resulting_input = TopicInput(effect.output_topic)
                for node in self.node_models:
                    if resulting_input in node.get_possible_inputs():
                        self.add_action_and_effects(RxAction(ActionState.WAITING, node.get_name(), resulting_input.input_topic), timestep, node_id)

    def process(self):
        lc(f"Processing Graph with {self.graph.number_of_nodes()} nodes")
        repeat: bool = True
        while repeat:
            repeat = False
            for graph_node_id, node_data in self.graph.nodes(data=True):
                d(f" Node {graph_node_id}: {node_data}")
                # Skip actions that still have ordering constraints
                if cast(int, self.graph.out_degree(graph_node_id)) > 0:
                    continue
                data: RxAction = node_data["data"]  # type: ignore
                # Skip actions which are still missing their data
                if data.state != ActionState.READY:
                    continue
                l(f"  Action is ready and has no constraints: RX of {data.topic} ({type(data.data).__name__}) at node {data.node}. Publishing data...")
                repeat = True
                data.state = ActionState.RUNNING
                self.interception_pubs[data.node][data.topic].publish(data.data)
        l(" Done processing!")

    def find_running_action(self, published_topic_name: TopicName) -> int:
        for node_id, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING:
                node_model = self.node_model_by_name(d.node)
                outputs = node_model.effects_for_input(TopicInput(d.topic))
                for output in outputs:
                    if output == TopicPublish(published_topic_name):
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action which should have published a message on {published_topic_name}")

    def find_running_action_status(self, node_name: NodeName) -> int:
        for node_id, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING and d.node == node_name:
                node_model = self.node_model_by_name(d.node)
                outputs = node_model.effects_for_input(TopicInput(d.topic))
                for output in outputs:
                    if output == StatusPublish():
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action for node {node_name} which should have published a status message")

    def interception_subscription_callback(self, topic_name: TopicName, msg: Any):
        lc(f"Received message on intercepted topic {topic_name}")

        # Complete the action which sent this message
        try:
            cause_action_id = self.find_running_action(topic_name)
            causing_action: RxAction = self.graph.nodes[cause_action_id]["data"]  # type: ignore
            l(f"  This completes the {causing_action.topic} callback of {causing_action.node}! Removing node...")
            self.graph.remove_node(cause_action_id)
        except ActionNotFoundError:
            if topic_name in [name for (_type, name) in external_input_topics_config]:
                l("  This is an external input.")
                pass
            else:
                l("  This is not an external input!")
                raise

        # Buffer this data for next actions
        i: int = 0
        for _node, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            if d.state == ActionState.WAITING and d.topic == topic_name:
                i += 1
                d.state = ActionState.READY
                d.data = msg
        l(f"  {i} actions are now ready to forward data")

        self.process()

    def node_model_by_name(self, name) -> NodeModel:
        for node in self.node_models:
            if node.get_name() == name:
                return node

        raise KeyError(f"No model for node \"{name}\"")

    def status_callback(self, msg: Status):
        lc(f"Received status message from {msg.node_name}")
        # Complete the action which sent this message
        cause_action_id = self.find_running_action_status(msg.node_name)
        causing_action: RxAction = self.graph.nodes[cause_action_id]["data"]  # type: ignore
        l(f"  This completes the {causing_action.topic} callback of {causing_action.node}! Removing node...")
        self.graph.remove_node(cause_action_id)


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
