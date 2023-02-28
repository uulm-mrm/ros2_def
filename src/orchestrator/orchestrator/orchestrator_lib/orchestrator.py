from dataclasses import dataclass
import datetime
from time import sleep
from typing import Any, Type, cast, TypeAlias

from orchestrator.orchestrator_lib.node_model import Cause, NodeModel, ServiceCall, StatusPublish, TimerInput, TopicInput, TopicPublish
from orchestrator.orchestrator_lib.name_utils import NodeName, TopicName, collect_intercepted_topics, type_from_string
from orchestrator.orchestrator_lib.action import ActionNotFoundError, ActionState, EdgeType, RxAction, TimerCallbackAction, Action

from orchestrator.orchestrator_lib.ros_utils.logger import lc
from orchestrator.orchestrator_lib.ros_utils.spin import spin_for

from orchestrator_interfaces.msg import Status

import rclpy
from rclpy import Future
from rclpy.node import Node as RosNode
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time
from rclpy.clock import ClockType

import networkx as nx
import matplotlib.pyplot as plt
import netgraph

import rosgraph_msgs.msg

GraphNodeId: TypeAlias = int


def _next_id() -> GraphNodeId:
    n = _next_id.value
    _next_id.value += 1
    return n


_next_id.value = 0


# Inputs always correspond to current simulator time
@dataclass
class FutureInput:
    topic: TopicName
    future: Future


@dataclass
class FutureTimestep:
    time: Time
    future: Future


class Orchestrator:
    def __init__(self, ros_node: RosNode, node_config, logger: RcutilsLogger | None = None) -> None:
        self.ros_node = ros_node
        self.l = logger or ros_node.get_logger()

        self.node_models: list[NodeModel] = node_config

        # Subscription for each topic (topic_name -> sub)
        self.interception_subs: dict[TopicName, Subscription] = {}
        # Publisher for each node (node_name -> topic_name -> pub)
        self.interception_pubs: dict[NodeName, dict[TopicName, Publisher]] = {}

        # Subscriptions for outputs which we do not need to buffer,
        #  but we need to inform the node models that they happened
        self.output_subs: list[Subscription] = []

        # Offered input by the data source, which can be requested by completing the contained future
        self.next_input: FutureInput | FutureTimestep | None = None

        # The current time of the data source. This should be set once all actions for this time have been added,
        #  altough they might not be done yet.
        #  When setting this, the data source is allowed to publish the time on /clock, and the corresponding next_input
        #  shall be requested.
        self.simulator_time: Time | None = None
        self.ignore_inputs = False
        self.discard_next_clock = True
        self.expected_external_inputs: list[TopicName] = []

        self.graph: nx.DiGraph = nx.DiGraph()

    def initialize_ros_communication(self):
        # TODO: get this info from node models/check missing topics
        topic_names_and_types = self.ros_node.get_topic_names_and_types()
        self.l.debug(f"Known topics: {topic_names_and_types}")
        for node, canonical_name, intercepted_name, type in collect_intercepted_topics(topic_names_and_types):
            lc(self.l, f"Intercepted input \"{canonical_name}\" of type {type.__name__} from node \"{node}\" as \"{intercepted_name}\"")
            # Subscribe to the input topic
            if canonical_name not in self.interception_subs:
                self.l.info(f"  Subscribing to {canonical_name}")
                subscription = self.ros_node.create_subscription(
                    type, canonical_name,
                    lambda msg, topic_name=canonical_name: self.__interception_subscription_callback(topic_name, msg),
                    10)
                self.interception_subs[canonical_name] = subscription
            else:
                self.l.info("  Subscription already exists")
            # Create separate publisher for each node
            self.l.info(f"  Creating publisher for {intercepted_name}")
            publisher = self.ros_node.create_publisher(type, intercepted_name, 10)
            if node not in self.interception_pubs:
                self.interception_pubs[node] = {}
            self.interception_pubs[node][canonical_name] = publisher

        def get_type(topic: str) -> Type:
            if not topic.startswith("/"):
                topic = "/" + topic
            for n, ts in topic_names_and_types:
                if n == topic:
                    assert len(ts) == 1
                    return type_from_string(ts[0])
            raise RuntimeError(f"Could not find type for input-topic {topic}.")

        # Subscribe to outputs (tracking example: plausibility node publishers)
        for node_model in self.node_models:
            for cause in node_model.get_possible_inputs():
                for effect in node_model.effects_for_input(cause):
                    match effect:
                        case TopicPublish():
                            if effect.output_topic not in self.interception_subs:
                                self.l.info(f"Subscribing to output topic: {effect.output_topic}")
                                sub = self.ros_node.create_subscription(
                                    get_type(effect.output_topic),
                                    effect.output_topic,
                                    lambda msg, topic_name=effect.output_topic: self.__interception_subscription_callback(topic_name, msg),
                                    10)
                                self.output_subs.append(sub)
                        case StatusPublish():
                            pass
                        case ServiceCall():
                            pass

        self.status_subscription = self.ros_node.create_subscription(Status, "status", self.__status_callback, 10)

    def wait_until_publish_allowed(self, topic: TopicName) -> Future:
        """
        Get a future that will be complete once the specified topic can be published.
        The caller should spin the executor of the RosNode while waiting, otherwise the future
        may never be done (and processing might not continue).
        """

        if self.next_input is not None:
            raise RuntimeError("Data source wants to provide next input before waiting on the last one!")

        if self.simulator_time is None:
            raise RuntimeError("Data source has to provide time before first data")

        lc(self.l, f"Data source offers input on topic \"{topic}\" for current time {self.simulator_time}")

        future = Future()
        self.next_input = FutureInput(topic, future)

        if not self.__graph_is_busy():
            self.l.info("  There are no running actions, immediately requesting data...")
            self.__request_next_input()

        return future

    def __initialize_sim_time(self, initial_time: Time):
        lc(self.l, f"Initializing sim time at {initial_time}")

        self.ignore_inputs = True

        msg = rosgraph_msgs.msg.Clock()
        msg.clock = initial_time.to_msg()

        for node_model in self.node_models:
            node_name = node_model.get_name()
            fastest_timer_period = None
            for input_cause in node_model.get_possible_inputs():
                if isinstance(input_cause, TimerInput):
                    if fastest_timer_period is None or fastest_timer_period > input_cause.period:
                        fastest_timer_period = input_cause.period

            if fastest_timer_period is not None:
                if initial_time.nanoseconds < fastest_timer_period:
                    self.l.info(f"  Initializing sim time at node {node_name}, expecting 1 callback")
                    # TODO: expect no callback
                    pass
                elif fastest_timer_period <= initial_time.nanoseconds < 2*fastest_timer_period:
                    self.l.info(f"  Initializing sim time at node {node_name}, expecting 1 callback")
                    # TODO: expect 1 callback
                    pass
                else:
                    self.l.info(f"  Initializing sim time at node {node_name}, expecting 2 callbacks")
                    # TODO: expect 2 callbacks
                    pass

                pub = self.interception_pubs[node_name]["clock"]
                pub.publish(msg)

        delay = datetime.timedelta(seconds=5)
        self.l.info(f"  Spinning for {delay} to let (duplicate) initial-timer-callbacks complete")
        spin_for(self.ros_node, delay)
        self.ignore_inputs = False
        lc(self.l, "Done initializing sim time!")

    def wait_until_time_publish_allowed(self, t: Time) -> Future:

        if self.simulator_time is None:
            self.__initialize_sim_time(t)
            self.simulator_time = t
            self.discard_next_clock = True
            f = Future()
            f.set_result(None)
            return f

        if self.next_input is not None:
            raise RuntimeError("Data source wants to provide next timestep before waiting on the last one!")

        lc(self.l, f"Data source offers clock input for time {t}")

        f = Future()
        self.next_input = FutureTimestep(t, f)

        if not self.__graph_is_busy():
            self.l.info("  There are no running actions, immediately requesting time...")
            self.__request_next_input()

        return f

    def __add_pending_timers_until(self, t: Time):
        """
        Add all remaining timer events.

        At time of calling, simulator_time should be the last time for which all timers have already
        been added.
        """
        # When receiving time T, timers <=T fire
        # A timer doesnt fire at 0
        # For time jumps > 2*dt, timer always fires exactly twice (at most one missed callback is executed)
        # It seems that with use_sim_time, timers always start at 0, even if time has been published before.

        lc(self.l, f"Adding pending timers until {t}")

        if self.simulator_time is None:
            raise RuntimeError("simulator_time has to be initialized before adding timer events")

        last_time: int = self.simulator_time.nanoseconds  # time until which timer actions have been added already

        @dataclass
        class Timer:
            node: str
            cause: TimerInput

        timers: list[Timer] = []
        for node_model in self.node_models:
            for input_cause in node_model.get_possible_inputs():
                if isinstance(input_cause, TimerInput):
                    timer = Timer(node_model.get_name(), input_cause)
                    timers.append(timer)

        expected_timer_actions: list[TimerCallbackAction] = []

        for timer in timers:
            nr_fires = last_time // timer.cause.period
            last_fire = nr_fires * timer.cause.period
            next_fire = Time(nanoseconds=last_fire + timer.cause.period)

            dt: int = t.nanoseconds - last_time
            if dt > timer.cause.period:
                raise RuntimeError(f"Requested timestep too large! Stepping time from {last_time} to {t} ({dt}) "
                                   "would require firing the timer multiple times within the same timestep. This is probably unintended!")
            if next_fire <= t:
                action = TimerCallbackAction(ActionState.WAITING, timer.node, t, timer.cause, timer.cause.period)
                expected_timer_actions.append(action)

        for action in expected_timer_actions:
            self.__add_action_and_effects(action)

    def __add_topic_input(self, t: Time, topic: TopicName):
        lc(self.l, f"Adding input on topic \"{topic}\"")

        self.expected_external_inputs.append(topic)

        input = TopicInput(topic)
        expected_rx_actions = []

        for node in self.node_models:
            if input in node.get_possible_inputs():
                expected_rx_actions.append(RxAction(ActionState.WAITING, node.get_name(), t, TopicInput(input.input_topic), input.input_topic))

        self.l.info(f"  This input causes {len(expected_rx_actions)} rx actions")

        for action in expected_rx_actions:
            self.__add_action_and_effects(action)

    def __request_next_input(self):
        """
        Request publishing of next input.
        Requires the data provider to already be waiting to publish (called wait_until_publish_allowed before).
        """

        match self.next_input:
            case FutureInput():
                next = self.next_input
                self.next_input = None
                self.l.info(f"Requesting data on topic {next.topic} for current time {self.simulator_time}")
                assert self.simulator_time is not None
                self.__add_topic_input(self.simulator_time, next.topic)
                next.future.set_result(None)
            case FutureTimestep():
                next = self.next_input
                self.next_input = None
                self.l.info(f"Requesting clock input for time {next.time}")
                self.__add_pending_timers_until(next.time)
                self.simulator_time = next.time
                next.future.set_result(None)
            case None:
                raise RuntimeError("No unused input available. Check if next_input exists before calling __request_next_input.")

    def __add_action_and_effects(self, action: Action, parent: int | None = None):

        # Parent: Node ID of the action causing this topic-publish. Should only be None for inputs
        cause_node_id = _next_id()
        self.l.debug(f"  Adding graph node for action {action}")
        self.graph.add_node(cause_node_id, data=action)

        # Sibling connections: Complete all actions at this node before the to-be-added action
        for node, node_data in self.graph.nodes(data=True):
            other_action = cast(Action, node_data["data"])  # type: ignore
            if other_action.node == action.node and node != cause_node_id:
                self.graph.add_edge(cause_node_id, node, edge_type=EdgeType.SAME_NODE)

        if parent is not None:
            self.l.debug(f"   Adding edge to parent: {parent}")
            self.graph.add_edge(cause_node_id, parent, edge_type=EdgeType.CAUSALITY)

        cause: Cause = action.cause

        node_model = self.__node_model_by_name(action.node)
        assert cause in node_model.get_possible_inputs()
        effects = node_model.effects_for_input(cause)

        assert len(effects) > 0

        # Multi-Publisher Connections:
        # Add edge to all RxActions for the topics that are published by this action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                # Add edge to all RxActions for this topic.
                # Not doing this would allow concurrent publishing on the same topic, which results in undeterministic receive order
                for node, node_data in self.graph.nodes(data=True):
                    other_action: Action = node_data["data"]  # type: ignore
                    if isinstance(other_action, RxAction) and other_action.topic == effect.output_topic:
                        self.l.debug(f"   Adding edge to node {node}, because it publishes on same topic")
                        # TODO: I think this occurs in other, unneeded cases?
                        self.graph.add_edge(cause_node_id, node, edge_type=EdgeType.SAME_TOPIC)

        # Collect services relating to this action
        services = set(node_model.get_provided_services())
        for effect in effects:
            if isinstance(effect, ServiceCall):
                services.add(effect.service_name)
        self.l.debug(f"   This action interactis with services: {services}")

        # Add connection to service group
        for service in services:
            for id in self.__service_group(service):
                if id == cause_node_id:
                    continue
                self.l.debug(f"   Adding edge to action in service group: {id}")
                self.graph.add_edge(cause_node_id, id, edge_type=EdgeType.SERVICE_GROUP)

        # Recursively add action nodes for publish events in the current action
        for effect in effects:
            if isinstance(effect, TopicPublish):
                resulting_input = TopicInput(effect.output_topic)
                for node in self.node_models:
                    if resulting_input in node.get_possible_inputs():
                        self.__add_action_and_effects(
                            RxAction(ActionState.WAITING, node.get_name(), action.timestamp, resulting_input, resulting_input.input_topic), cause_node_id)

    def __find_service_provider_node(self, service: str) -> NodeModel:
        for node_model in self.node_models:
            if service in node_model.get_provided_services():
                return node_model
        raise RuntimeError(f"Could not find node providing service \"{service}\"")

    def __service_group(self, service: str) -> set[GraphNodeId]:
        """Find all actions which relate to a service"""
        l: set[GraphNodeId] = set()

        service_call = ServiceCall(service_name=service)
        service_provider_node = self.__find_service_provider_node(service).get_name()

        for graph_node_id, node_data in self.graph.nodes(data=True):
            data: Action = node_data["data"]  # type: ignore

            # Add actions at node provider
            if data.node == service_provider_node:
                l.add(graph_node_id)
                continue

            # Add actions which call the service
            node_model = self.__node_model_by_name(data.node)
            if service_call in node_model.effects_for_input(data.cause):
                l.add(graph_node_id)
                continue

        return l

    def __process(self):
        lc(self.l, f"Processing Graph with {self.graph.number_of_nodes()} nodes")
        repeat: bool = True
        while repeat:
            repeat = False
            for graph_node_id, node_data in self.graph.nodes(data=True):
                self.l.debug(f"Node {graph_node_id}: {node_data}")
                # Skip actions that still have ordering constraints
                if cast(int, self.graph.out_degree(graph_node_id)) > 0:
                    self.l.debug("Has dependencies")
                    continue
                data: Action = node_data["data"]  # type: ignore
                # Skip actions which are still missing their data
                if data.state != ActionState.READY:
                    self.l.debug("Is not ready")
                    continue
                match data:
                    case RxAction():
                        assert data.data is not None
                        self.l.info(
                            f"    Action is ready and has no constraints: RX of {data.topic} ({type(data.data).__name__}) at node {data.node}. Publishing data...")
                        data.state = ActionState.RUNNING
                        self.interception_pubs[data.node][data.topic].publish(data.data)
                    case TimerCallbackAction():
                        self.l.info(
                            f"    Action is ready and has no constraints: Timer callback with period "
                            f"{data.cause.period} at time {data.timestamp} of node {data.node}. Publishing clock...")
                        data.state = ActionState.RUNNING
                        time_msg = rosgraph_msgs.msg.Clock()
                        time_msg.clock = data.timestamp.to_msg()
                        self.interception_pubs[data.node]["clock"].publish(time_msg)
                repeat = True
        self.l.info("  Done processing! Checking if next input can be requested...")

        if self.next_input is None:
            self.l.info("  Next input can not be requested yet, no input data has been offered.")
        elif not self.__ready_for_next_input():
            self.l.info("  Next input is available, but we are not ready for it yet.")
        else:
            self.l.info("  We are now ready for the next input, requesting...")
            self.__request_next_input()

    def __ready_for_next_input(self) -> bool:
        """
        Check if we are ready for the next input.

        This is the case for data-input iff there are no nodes which are currently waiting
        for this topic or have already buffered this topic, which both means
        that the input has already been requested (or even received).

        For time-inputs, this is the case if there are no actions left which are waiting to
        receive an earlier clock input.
        """

        match self.next_input:
            case None:
                raise RuntimeError("There is no next input!")
            case FutureTimestep():
                # We are ready for time input as soon as there are no actions left waiting for another (earlier)
                # clock input.
                for _graph_node_id, node_data in self.graph.nodes(data=True):
                    data: Action = node_data["data"]  # type: ignore
                    if not isinstance(data, TimerCallbackAction):
                        continue
                    if data.state != ActionState.WAITING:
                        continue
                    if data.timestamp == self.next_input.time:
                        continue
                    self.l.debug("Not ready for next clock input since some actions are waiting for another clock input")
                    return False
                return True
            case FutureInput():
                for _graph_node_id, node_data in self.graph.nodes(data=True):
                    data: Action = node_data["data"]  # type: ignore
                    if data.state == ActionState.READY or data.state == ActionState.WAITING:
                        if isinstance(data, RxAction) and data.topic == self.next_input.topic:
                            return False
                return True

    def __graph_is_busy(self) -> bool:
        has_running = False
        for graph_node_id, node_data in self.graph.nodes(data=True):
            data: Action = node_data["data"]  # type: ignore
            if data.state == ActionState.RUNNING or data.state == ActionState.WAITING:
                has_running = True
                break
        return has_running

    def __find_running_action(self, published_topic_name: TopicName) -> int:
        """Find running action which published the message on the specified topic"""
        for node_id, node_data in self.graph.nodes(data=True):
            d: Action = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING:
                node_model = self.__node_model_by_name(d.node)
                outputs = node_model.effects_for_input(d.cause)
                for output in outputs:
                    if output == TopicPublish(published_topic_name):
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action which should have published a message on topic \"{published_topic_name}\"")

    def __find_running_action_status(self, node_name: NodeName) -> int:
        for node_id, node_data in self.graph.nodes(data=True):
            d: Action = node_data["data"]  # type: ignore
            if d.state == ActionState.RUNNING and d.node == node_name:
                node_model = self.__node_model_by_name(d.node)
                outputs = node_model.effects_for_input(d.cause)
                for output in outputs:
                    if output == StatusPublish():
                        return node_id
        raise ActionNotFoundError(f"There is no currently running action for node {node_name} which should have published a status message")

    def plot_graph(self):
        annotations = {}
        for node, node_data in self.graph.nodes(data=True):
            d: Action = node_data["data"]  # type: ignore
            match d:
                case RxAction():
                    annotations[node] = f"{d.node}: rx {d.topic}"
                case TimerCallbackAction():
                    annotations[node] = f"{d.node}: timer @{d.timestamp}"

        color_map = {
            EdgeType.SAME_NODE: "tab:green",
            EdgeType.SAME_TOPIC: "tab:orange",
            EdgeType.CAUSALITY: "tab:blue",
            EdgeType.SERVICE_GROUP: "tab:red"
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
            data: Action = node_data["data"]  # type: ignore
            node_to_community[node] = data.timestamp

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

        if self.ignore_inputs:
            self.l.info("  Currently ignoring all inputs...")
            return

        if topic_name == "clock" and self.discard_next_clock:
            self.l.info("  Discarding this clock input")
            self.discard_next_clock = False
            return

        if topic_name == "clock":

            clock_msg = cast(rosgraph_msgs.msg.Clock, msg)
            time_input = Time.from_msg(clock_msg.clock, clock_type=ClockType.SYSTEM_TIME)  # ROS_TIME

            self.l.debug(f"Time input: {time_input}")
            self.l.debug(f"Simulator time: {self.simulator_time}")  # SYSTEM_TIME

            if time_input != self.simulator_time:
                # "Advancing time" means setting self.simulator_time, and allowing publishing of the corresponding value.
                # If no actions wait for a specific timestep, this might happen before the timestep is actually received.
                self.l.debug(
                    f"  This clock input is for time {time_input}, but we already advanced time to {self.simulator_time}. Nothing should happen now...")

            for action_node_id, node_data in self.graph.nodes(data=True):
                rxdata: Action = node_data["data"]  # type: ignore
                if not isinstance(rxdata, TimerCallbackAction):
                    continue
                self.l.debug(f"node {action_node_id}: {rxdata}")
                if rxdata.timestamp != time_input:
                    continue
                if rxdata.state != ActionState.WAITING:
                    continue
                self.l.info(f"  Setting action to ready: {rxdata}")
                rxdata.state = ActionState.READY

        else:

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

            # TODO: Maybe this could be simplified by only advancing time once all data for this timestep is buffered
            # (no waiting actions with current timestep left) (does this imply no waiting actions left at all? -> yes, but only for time-input)

            try:
                cause_action_id = self.__find_running_action(topic_name)
                causing_action: Action = self.graph.nodes[cause_action_id]["data"]  # type: ignore
                self.l.info(f"  This completes the {causing_action.cause} callback of {causing_action.node}! Removing node...")
                # TODO: it does not. maybe the callback publishes more data?
                # I guess it can be removed once all actions with a causality node to this have buffered data?
                # Removing node happens below, since we still need it to find actions caused by this...
            except ActionNotFoundError:
                if topic_name in self.expected_external_inputs:
                    # This is an external input...
                    self.l.info("  This is an external input")
                    self.expected_external_inputs.remove(topic_name)
                    # Find the timestep of the earliest matching waiting action, such that we don't accidentally
                    #  buffer this data for later inputs below.
                    for action_node_id, node_data in self.graph.nodes(data=True):
                        rxdata: Action = node_data["data"]  # type: ignore
                        if not isinstance(rxdata, RxAction):
                            continue
                        if rxdata.topic != topic_name:
                            continue
                        if rxdata.state != ActionState.WAITING:
                            continue
                        t = rxdata.timestamp
                        if input_timestep is None or t < input_timestep:
                            input_timestep = t
                    self.l.info(f"  for timestep {input_timestep}.")
                else:
                    self.l.error(" This is not an external input!")
                    raise

            # Buffer this data for next actions
            i: int = 0
            for action_node_id, node_data in self.graph.nodes(data=True):
                rxdata: Action = node_data["data"]  # type: ignore
                if not isinstance(rxdata, RxAction):
                    continue
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
                    if rxdata.timestamp != input_timestep:
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
        if self.ignore_inputs:
            self.l.info("  Currently ignoring all inputs...")
            return
        # Complete the action which sent this message
        cause_action_id = self.__find_running_action_status(msg.node_name)
        causing_action: Action = self.graph.nodes[cause_action_id]["data"]  # type: ignore
        self.l.info(f"  This completes the {causing_action.cause} callback of {causing_action.node}! Removing node...")
        self.graph.remove_node(cause_action_id)
        self.__process()
