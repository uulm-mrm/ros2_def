from typing import Any, cast

from .node_model import NodeModel, StatusPublish, TopicInput, TopicPublish
from .name_utils import NodeName, TopicName, collect_intercepted_topics
from .action import ActionNotFoundError, ActionState, EdgeType, RxAction

from orchestrator_dummy_nodes.ros_utils.logger import lc

from orchestrator_interfaces.msg import Status

from rclpy.node import Node as RosNode
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

import networkx as nx
import matplotlib.pyplot as plt
import netgraph


def _next_id() -> int:
    n = _next_id.value
    _next_id.value += 1
    return n


_next_id.value = 0


class Orchestrator:
    def __init__(self, ros_node: RosNode, node_config, external_input_topics_config, output_topics, logger: RcutilsLogger | None = None) -> None:
        self.ros_node = ros_node
        self.l = logger or ros_node.get_logger()
        self.output_topics = output_topics
        self.external_input_topics_config = external_input_topics_config

        self.node_models: list[NodeModel] = node_config

        # Subscription for each topic (topic_name -> sub)
        self.interception_subs: dict[TopicName, Subscription] = {}
        # Publisher for each node (node_name -> topic_name -> pub)
        self.interception_pubs: dict[NodeName, dict[TopicName, Publisher]] = {}

        self.external_input_topics = external_input_topics_config
        # self.external_input_subs: list[Subscription] = []

        # Subscriptions for outputs which we do not need to buffer,
        #  but we need to inform the node models that they happened
        self.output_subs: list[Subscription] = []

        self.graph: nx.DiGraph = nx.DiGraph()

        # TODO: Add new timesteps automatically
        self.timestep: int = 0
        self.__add_input(self.timestep)
        for _i in range(11):
            self.timestep += 1
            self.__add_input(self.timestep)

    def initialize_ros_communication(self):
        for node, canonical_name, intercepted_name, type in collect_intercepted_topics(self.ros_node.get_topic_names_and_types()):
            lc(self.l, f"Intercepted input \"{canonical_name}\" of type {type.__name__} from node \"{node}\" as \"{intercepted_name}\"")
            # Subscribe to the input topic
            if canonical_name not in self.interception_subs:
                self.l.info(f" Subscribing to {canonical_name}")
                subscription = self.ros_node.create_subscription(
                    type, canonical_name,
                    lambda msg, topic_name=canonical_name: self.__interception_subscription_callback(topic_name, msg),
                    10)
                self.interception_subs[canonical_name] = subscription
            else:
                self.l.info(" Subscription already exists")
            # Create separate publisher for each node
            self.l.info(f" Creating publisher for {intercepted_name}")
            publisher = self.ros_node.create_publisher(type, intercepted_name, 10)
            if node not in self.interception_pubs:
                self.interception_pubs[node] = {}
            self.interception_pubs[node][canonical_name] = publisher

        # Subscribe to outputs (tracking example: plausibility node publishers)
        for MessageType, topic_name in self.output_topics:
            self.l.info(f"Subscribing to output topic: {topic_name}")
            sub = self.ros_node.create_subscription(
                MessageType,
                topic_name,
                lambda msg, topic_name=topic_name: self.__interception_subscription_callback(topic_name, msg),
                10)
            self.output_subs.append(sub)

        self.status_subscription = self.ros_node.create_subscription(Status, "status", self.__status_callback, 10)

    def __add_input(self, timestep):
        inputs = [TopicInput(top) for (_type, top) in self.external_input_topics_config]
        lc(self.l, f"Adding inputs on {len(inputs)} topics")

        expected_rx_actions = []

        for node in self.node_models:
            for input in inputs:
                if input in node.get_possible_inputs():
                    expected_rx_actions.append(RxAction(ActionState.WAITING, node.get_name(), input.input_topic))

        self.l.info(f" Those inputs cause {len(expected_rx_actions)} rx actions")
        for action in expected_rx_actions:
            self.__add_action_and_effects(action, timestep)

    def __add_action_and_effects(self, action: RxAction, timestep: int,  parent: int | None = None):
        # Parent: Node ID of the action causing this topic-publish. Should only be None for inputs
        node_id = _next_id()
        self.graph.add_node(node_id, data=action, timestep=timestep)

        # Sibling connections: Complete all actions at this node before the to-be-added action
        for node, node_data in self.graph.nodes(data=True):
            other_action: RxAction = node_data["data"]  # type: ignore
            if other_action.node == action.node and node != node_id:
                self.graph.add_edge(node_id, node, edge_type=EdgeType.SAME_NODE)

        if parent is not None:
            self.graph.add_edge(node_id, parent, edge_type=EdgeType.CAUSALITY)

        topic_input_cause = TopicInput(action.topic)
        node_model = self.__node_model_by_name(action.node)
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
                        # TODO: I think this occurs in other, unneeded cases?
                        self.graph.add_edge(node_id, node, edge_type=EdgeType.SAME_TOPIC)

        # Recursively add action nodes for publish events in the current action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                resulting_input = TopicInput(effect.output_topic)
                for node in self.node_models:
                    if resulting_input in node.get_possible_inputs():
                        self.__add_action_and_effects(RxAction(ActionState.WAITING, node.get_name(), resulting_input.input_topic), timestep, node_id)

    def __process(self):
        lc(self.l, f"Processing Graph with {self.graph.number_of_nodes()} nodes")
        repeat: bool = True
        while repeat:
            repeat = False
            for graph_node_id, node_data in self.graph.nodes(data=True):
                # Skip actions that still have ordering constraints
                if cast(int, self.graph.out_degree(graph_node_id)) > 0:
                    continue
                data: RxAction = node_data["data"]  # type: ignore
                # Skip actions which are still missing their data
                if data.state != ActionState.READY:
                    continue
                self.l.info(
                    f"  Action is ready and has no constraints: RX of {data.topic} ({type(data.data).__name__}) at node {data.node}. Publishing data...")
                repeat = True
                data.state = ActionState.RUNNING
                self.interception_pubs[data.node][data.topic].publish(data.data)
        self.l.info(" Done processing!")

    def __find_running_action(self, published_topic_name: TopicName) -> int:
        for node_id, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING:
                node_model = self.__node_model_by_name(d.node)
                outputs = node_model.effects_for_input(TopicInput(d.topic))
                for output in outputs:
                    if output == TopicPublish(published_topic_name):
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action which should have published a message on {published_topic_name}")

    def __find_running_action_status(self, node_name: NodeName) -> int:
        for node_id, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING and d.node == node_name:
                node_model = self.__node_model_by_name(d.node)
                outputs = node_model.effects_for_input(TopicInput(d.topic))
                for output in outputs:
                    if output == StatusPublish():
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action for node {node_name} which should have published a status message")

    def plot_graph(self):
        annotations = {}
        for node, node_data in self.graph.nodes(data=True):
            d: RxAction = node_data["data"]  # type: ignore
            annotations[node] = f"{d.node}: rx {d.topic}"

        color_map = {
            EdgeType.SAME_NODE: "tab:green",
            EdgeType.SAME_TOPIC: "tab:orange",
            EdgeType.CAUSALITY: "tab:blue"
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
                label=name.name
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
                                                  annotations=annotations,
                                                  node_labels=True,
                                                  arrows=True,
                                                  edge_color=edge_colors,
                                                  ax=ax)
        for artist in plot_instance.artist_to_annotation:
            placement = plot_instance._get_annotation_placement(artist)
            plot_instance._add_annotation(artist, *placement)

        plt.show()

    def __interception_subscription_callback(self, topic_name: TopicName, msg: Any):
        lc(self.l, f"Received message on intercepted topic {topic_name}")

        # Complete the action which sent this message
        cause_action_id = None
        input_timestep = None

        # TODO: This input-timestep handling should be resolved using same-node edges! External inputs should only be buffered by
        #  Root nodes!(?)
        # are all actions requiring external input root nodes?
        # all actions directly caused by external input have no causality edges.
        #  future exception: action with multiple input topics, when one of them is not an external input...
        # an action directly caused by external input *can* have a same-node connection to another action
        #  (e.g. 2 external-input callbacks on the same node)
        # Maybe each external input should be a graph node, such that the actions for that timestep are directly connected?
        # -> Set external input to "running" when time is advanced
        # -> On rx, buffer at each action with causality edge to this running external-input
        # -> Remove external-input

        try:
            cause_action_id = self.__find_running_action(topic_name)
            causing_action: RxAction = self.graph.nodes[cause_action_id]["data"]  # type: ignore
            self.l.info(f"  This completes the {causing_action.topic} callback of {causing_action.node}! Removing node...")
            # Removing node happens below, since we still need it to find actions caused by this...
        except ActionNotFoundError:
            if topic_name in [name for (_type, name) in self.external_input_topics_config]:
                # Find the timestep of the earliest matching waiting action, such that we don't accidentally
                #  buffer this data for later inputs below.
                for action_node_id, node_data in self.graph.nodes(data=True):
                    rxdata: RxAction = node_data["data"]  # type: ignore
                    if rxdata.topic != topic_name:
                        continue
                    if rxdata.state != ActionState.WAITING:
                        continue
                    t = cast(int, node_data["timestep"])  # type: ignore
                    if input_timestep is None or t < input_timestep:
                        input_timestep = t
                self.l.info(f"  This is an external input for timestep {input_timestep}.")
            else:
                self.l.info("  This is not an external input!")
                raise

        # Buffer this data for next actions
        i: int = 0
        for action_node_id, node_data in self.graph.nodes(data=True):
            rxdata: RxAction = node_data["data"]  # type: ignore
            if rxdata.topic != topic_name:
                continue
            if rxdata.state != ActionState.WAITING:
                continue

            # If we caused this publish, only advance the actions directly caused by this ons
            if cause_action_id is not None:
                # Skip if this is action is not caused by cause_action_id
                #  or edge type is not "causality"
                if not self.graph.has_successor(action_node_id, cause_action_id) \
                        or not self.graph.edges[action_node_id, cause_action_id]["edge_type"] == EdgeType.CAUSALITY:
                    continue
            else:
                # This is an input, so we only buffer the data for the earliest timestep.
                assert input_timestep is not None
                if node_data["timestep"] != input_timestep:  # type: ignore
                    continue

            self.l.debug(f" Buffering data and readying action {action_node_id}: {node_data}")
            i += 1
            rxdata.state = ActionState.READY
            assert rxdata.data is None
            rxdata.data = msg

        self.l.info(f"  Buffered data for {i} actions")

        if cause_action_id != None:
            self.graph.remove_node(cause_action_id)

        self.__process()

    def __node_model_by_name(self, name) -> NodeModel:
        for node in self.node_models:
            if node.get_name() == name:
                return node

        raise KeyError(f"No model for node \"{name}\"")

    def __status_callback(self, msg: Status):
        lc(self.l, f"Received status message from {msg.node_name}")
        # Complete the action which sent this message
        cause_action_id = self.__find_running_action_status(msg.node_name)
        causing_action: RxAction = self.graph.nodes[cause_action_id]["data"]  # type: ignore
        self.l.info(f"  This completes the {causing_action.topic} callback of {causing_action.node}! Removing node...")
        self.graph.remove_node(cause_action_id)
        self.__process()
