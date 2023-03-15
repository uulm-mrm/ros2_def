from dataclasses import dataclass
from typing import Any, Generator, Tuple, Type, cast, TypeAlias

from orchestrator.orchestrator_lib.node_model import Cause, NodeModel, ServiceCall, StatusPublish, TimeSyncInfo, TimerInput, TopicInput, TopicPublish
from orchestrator.orchestrator_lib.name_utils import NodeName, TopicName, intercepted_name
from orchestrator.orchestrator_lib.action import ActionNotFoundError, ActionState, DataProviderInputAction, EdgeType, RxAction, TimerCallbackAction, Action, OrchestratorBufferAction, OrchestratorStatusAction, CallbackAction
from orchestrator.orchestrator_lib.ros_utils.logger import lc
from orchestrator.orchestrator_lib.ros_utils.message_filter import ApproximateTimeSynchronizerTracker
from orchestrator.orchestrator_lib.ros_utils.pubsub import wait_for_node_pub, wait_for_node_sub, wait_for_topic

from orchestrator_interfaces.msg import Status

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
        self.time_sync_models: dict[NodeName, dict[TimeSyncInfo, ApproximateTimeSynchronizerTracker]] = {}

        # Offered input by the data source, which can be requested by completing the contained future
        self.next_input: FutureInput | FutureTimestep | None = None

        # The current time of the data source. This should be set once all actions for this time have been added,
        #  altough they might not be done yet.
        #  When setting this, the data source is allowed to publish the time on /clock, and the corresponding next_input
        #  shall be requested.
        self.simulator_time: Time | None = None

        self.graph: nx.DiGraph = nx.DiGraph()

    def initialize_ros_communication(self):
        """
        Subscribe to all required ros topics as per launch-config.

        If a topic is not found, or a subscriber/publisher does not exist,
        this will wait for it and spin the node while waiting.
        """

        self.status_subscription = self.ros_node.create_subscription(Status, "status", self.__status_callback, 10)

        def intercept_topic(canonical_name: TopicName, node: NodeModel):
            intercepted_topic_name = intercepted_name(node.get_name(), canonical_name)
            lc(self.l, f"Intercepted input \"{canonical_name}\" "
               f" from node \"{node.get_name()}\" as \"{intercepted_topic_name}\"")
            TopicType = wait_for_topic(canonical_name, self.l, self.ros_node)
            # Subscribe to the input topic
            if canonical_name not in self.interception_subs:
                self.l.info(f"  Subscribing to {canonical_name}")

                subscription = self.ros_node.create_subscription(
                    TopicType, canonical_name,
                    lambda msg, topic_name=canonical_name: self.__interception_subscription_callback(topic_name, msg),
                    10)
                self.interception_subs[canonical_name] = subscription
                self.l.info(f"Added interception_subs[{canonical_name}] = {subscription.topic}")
            else:
                self.l.info("  Subscription already exists")

            # Create separate publisher for each node
            self.l.info(f"  Creating publisher for {intercepted_topic_name}")
            publisher = self.ros_node.create_publisher(TopicType, intercepted_topic_name, 10)
            if node.get_name() not in self.interception_pubs:
                self.interception_pubs[node.get_name()] = {}
            self.interception_pubs[node.get_name()][canonical_name] = publisher
            wait_for_node_sub(intercepted_topic_name, node.get_name(), self.l, self.ros_node)

        for node_model in self.node_models:
            for input_cause in node_model.get_possible_inputs():
                match input_cause:
                    case TopicInput():
                        intercept_topic(input_cause.input_topic, node_model)
                    case TimerInput():
                        intercept_topic("clock", node_model)

                for effect in node_model.effects_for_input(input_cause):
                    match effect:
                        case TopicPublish():
                            if effect.output_topic not in self.interception_subs:
                                self.l.info(f" Subscribing to output topic: {effect.output_topic}")
                                sub = self.ros_node.create_subscription(
                                    wait_for_topic(effect.output_topic, self.l, self.ros_node),
                                    effect.output_topic,
                                    lambda msg, topic_name=effect.output_topic: self.__interception_subscription_callback(topic_name, msg),
                                    10)
                                self.interception_subs[effect.output_topic] = sub
                                wait_for_node_pub(effect.output_topic, node_model.get_name(), self.l, self.ros_node)
                        case StatusPublish():
                            wait_for_node_pub("status", node_model.get_name(), self.l, self.ros_node)
                            pass
                        case ServiceCall():
                            pass
            for tsi in node_model.time_sync_infos():
                assert tsi not in self.time_sync_models
                if node_model.get_name() not in self.time_sync_models:
                    self.time_sync_models[node_model.get_name()] = {}
                self.time_sync_models[node_model.get_name()][tsi] = ApproximateTimeSynchronizerTracker(
                    list(tsi.input_topics), tsi.queue_size, tsi.slop)

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

        if self.__graph_is_empty():
            self.l.info("  Graph is empty, immediately requesting data...")
            self.__request_next_input()

        return future

    def __initialize_sim_time(self, initial_time: Time):
        lc(self.l, f"Initializing sim time at {initial_time}")

        for node_model in self.node_models:
            node_name = node_model.get_name()
            fastest_timer_cause = None
            for input_cause in node_model.get_possible_inputs():
                if isinstance(input_cause, TimerInput):
                    if fastest_timer_cause is None or input_cause.period < fastest_timer_cause.period:
                        fastest_timer_cause = input_cause

            if fastest_timer_cause is None:
                # Node has no timers
                continue

            def add_buffer(topic: TopicName, parent: GraphNodeId):
                buffer_node_id = _next_id()
                self.graph.add_node(buffer_node_id, data=OrchestratorBufferAction(TopicInput(topic)))
                self.graph.add_edge(buffer_node_id, parent, edge_type=EdgeType.CAUSALITY)

            def add_status(parent: GraphNodeId):
                status_node_id = _next_id()
                self.graph.add_node(status_node_id, data=OrchestratorStatusAction())
                self.graph.add_edge(status_node_id, parent, edge_type=EdgeType.CAUSALITY)

            if initial_time.nanoseconds < fastest_timer_cause.period:
                self.l.info(f"  Initializing sim time at node {node_name}, expecting no callback")
                self.l.warn("Node has timers, but no callback for the initial time-input, which means we can not be sure it received the initial time")
                pass
            elif fastest_timer_cause.period <= initial_time.nanoseconds < 2*fastest_timer_cause.period \
                    or (initial_time.nanoseconds % fastest_timer_cause.period) != 0:
                self.l.info(f"  Initializing sim time at node {node_name}, expecting 1 callback")

                causing_action = TimerCallbackAction(ActionState.WAITING, node_name, initial_time,
                                                     fastest_timer_cause, fastest_timer_cause.period)
                causing_action_id = _next_id()
                self.graph.add_node(causing_action_id, data=causing_action)

                for effect in node_model.effects_for_input(fastest_timer_cause):
                    match effect:
                        case TopicPublish():
                            add_buffer(effect.output_topic, causing_action_id)
                        case StatusPublish():
                            add_status(causing_action_id)
                        case ServiceCall():
                            pass

            else:
                self.l.info(f"  Initializing sim time at node {node_name}, expecting 2 callbacks")

                causing_action = TimerCallbackAction(ActionState.WAITING, node_name, initial_time,
                                                     fastest_timer_cause, fastest_timer_cause.period)
                causing_action_id = _next_id()
                self.graph.add_node(causing_action_id, data=causing_action)

                for effect in node_model.effects_for_input(fastest_timer_cause):
                    match effect:
                        case TopicPublish():
                            add_buffer(effect.output_topic, causing_action_id)
                            add_buffer(effect.output_topic, causing_action_id)
                        case StatusPublish():
                            add_status(causing_action_id)
                            add_status(causing_action_id)
                        case ServiceCall():
                            pass

        self.l.info(" Created all callback actions, proceeding as usual.")

    def wait_until_time_publish_allowed(self, t: Time) -> Future:
        lc(self.l, f"Data source offers clock input for time {t}")

        if self.next_input is not None:
            raise RuntimeError("Data source wants to provide next timestep before waiting on the last one!")

        if self.simulator_time is None:
            self.__initialize_sim_time(t)
            self.simulator_time = t
            f = Future()
            f.set_result(None)
            self.l.info("  This is the first time input, immediately requesting...")
            return f

        f = Future()
        self.next_input = FutureTimestep(t, f)

        if self.__graph_is_empty():
            self.l.info("  Graph is empty, immediately requesting time...")
            self.__request_next_input()
        else:
            self.l.info("  Graph is not empty, requesting time at later state.")

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

        self.l.info(f" Adding actions for {len(expected_timer_actions)} timer callbacks.")
        for action in expected_timer_actions:
            self.__add_action_and_effects(action)

    def __add_topic_input(self, t: Time, topic: TopicName):
        lc(self.l, f"Adding input on topic \"{topic}\"")

        input_action = DataProviderInputAction(ActionState.WAITING, topic)
        input_action_id = _next_id()
        self.graph.add_node(input_action_id, data=input_action)
        self.l.debug(f"Adding input action to graph: {input_action}")

        input = TopicInput(topic)

        buffer_action = OrchestratorBufferAction(input)
        buffer_action_id = _next_id()
        self.graph.add_node(buffer_action_id, data=buffer_action)
        self.graph.add_edge(buffer_action_id, input_action_id, edge_type=EdgeType.CAUSALITY)

        expected_rx_actions = []

        for node in self.node_models:
            if input in node.get_possible_inputs():
                time_sync = False
                if node.time_sync_info(input.input_topic) is not None:
                    time_sync = True
                expected_rx_actions.append(
                    RxAction(ActionState.WAITING,
                             node.get_name(),
                             t,
                             TopicInput(input.input_topic), input.input_topic, is_approximate_time_synced=time_sync))

        self.l.info(f"  This input causes {len(expected_rx_actions)} rx actions")

        for action in expected_rx_actions:
            self.__add_action_and_effects(action, parent=buffer_action_id)

        return input_action_id

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
                input_node_id = self.__add_topic_input(self.simulator_time, next.topic)
                input_node_data: DataProviderInputAction = self.graph.nodes[input_node_id]["data"]
                self.l.debug(" Setting input action state running")
                input_node_data.state = ActionState.RUNNING
                self.l.debug(" Requesting publish")
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

    def __callback_nodes_with_data(self) -> Generator[Tuple[GraphNodeId, CallbackAction], None, None]:
        for id, data in self.graph.nodes(data=True):
            action: Action = data["data"]  # type: ignore
            if isinstance(action, CallbackAction):
                yield (id, action)

    def __buffer_nodes_with_data(self) -> Generator[Tuple[GraphNodeId, OrchestratorBufferAction], None, None]:
        for id, data in self.graph.nodes(data=True):
            action: Action = data["data"]  # type: ignore
            if isinstance(action, OrchestratorBufferAction):
                yield (id, action)

    def __buffer_childs_of_parent(self, parent: GraphNodeId) -> Generator[Tuple[GraphNodeId, OrchestratorBufferAction], None, None]:
        for id, data in self.__buffer_nodes_with_data():
            if self.graph.has_edge(id, parent):
                yield id, data

    def __causality_childs_of(self, buffer_id: GraphNodeId) -> Generator[GraphNodeId, None, None]:
        for node in self.graph.predecessors(buffer_id):
            edge_type: EdgeType = self.graph.edges[node, buffer_id]["edge_type"]
            if edge_type == EdgeType.CAUSALITY:
                yield node

    def __add_action_and_effects(self, action: CallbackAction, parent: int | None = None):

        # Parent: Node ID of the action causing this topic-publish. Should only be None for inputs
        cause_node_id = _next_id()
        self.l.debug(f"  Adding graph node for action {action}")
        self.graph.add_node(cause_node_id, data=action)

        # Sibling connections: Complete all actions at this node before the to-be-added action
        for node, other_action in self.__callback_nodes_with_data():
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
                # Add edge to all orchestrator buffer actions of the same topic.
                # Why all? They are all in the same or past timestep by definition (insertion order).
                # Not doing this would allow concurrent publishing on the same topic, which results in undeterministic receive order
                for node, node_data in self.graph.nodes(data=True):
                    other_action: Action = node_data["data"]  # type: ignore
                    if isinstance(other_action, OrchestratorBufferAction) and other_action.cause.input_topic == effect.output_topic:
                        self.l.debug(f"   Adding edge to node {node}, because it ")
                        self.graph.add_edge(cause_node_id, node, edge_type=EdgeType.SAME_TOPIC)

        # Collect services relating to this action
        services = set(node_model.get_provided_services())
        for effect in effects:
            if isinstance(effect, ServiceCall):
                services.add(effect.service_name)
        self.l.debug(f"   This action interacts with services: {services}")

        # Add connection to service group
        for service in services:
            for id in self.__service_group(service):
                if id == cause_node_id:
                    continue
                self.l.debug(f"   Adding edge to action in service group: {id}")
                self.graph.add_edge(cause_node_id, id, edge_type=EdgeType.SERVICE_GROUP)

        # Recursively add action nodes for publish events in the current action
        for effect in effects:
            match effect:
                case TopicPublish():
                    resulting_input = TopicInput(effect.output_topic)

                    # Add buffer node
                    buffer_node_id = _next_id()
                    self.graph.add_node(buffer_node_id, data=OrchestratorBufferAction(resulting_input))

                    # Link buffer node upwards
                    self.graph.add_edge(buffer_node_id, cause_node_id, edge_type=EdgeType.CAUSALITY)

                    # Recursively add RX actions as children of buffer node
                    for node in self.node_models:
                        if resulting_input in node.get_possible_inputs():
                            time_sync = False
                            if node.time_sync_info(resulting_input.input_topic) is not None:
                                time_sync = True
                            self.__add_action_and_effects(
                                RxAction(
                                    ActionState.WAITING,
                                    node.get_name(),
                                    action.timestamp,
                                    resulting_input,
                                    resulting_input.input_topic, is_approximate_time_synced=time_sync
                                ), buffer_node_id)
                case StatusPublish():
                    status_node_id = _next_id()
                    self.graph.add_node(status_node_id, data=OrchestratorStatusAction())
                    self.graph.add_edge(status_node_id, cause_node_id, edge_type=EdgeType.CAUSALITY)
                case ServiceCall():
                    pass

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

        for graph_node_id, data in self.__callback_nodes_with_data():
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

    def __remove_node_recursive(self, graph_node: GraphNodeId):
        for child in self.__causality_childs_of(graph_node):
            self.__remove_node_recursive(child)
        self.graph.remove_node(graph_node)

    def __process(self):
        lc(self.l, f"Processing Graph with {self.graph.number_of_nodes()} nodes")
        repeat: bool = True
        while repeat:
            repeat = False
            for graph_node_id, data in list(self.__callback_nodes_with_data()):
                self.l.debug(f"Node {graph_node_id}: {data}")
                # Skip actions that still have ordering constraints
                if cast(int, self.graph.out_degree(graph_node_id)) > 0:
                    self.l.debug("Has dependencies")
                    continue
                # Skip actions which are still missing their data
                if data.state != ActionState.READY:
                    self.l.debug("Is not ready")
                    continue
                match data:
                    case RxAction(is_approximate_time_synced=True):
                        assert isinstance(data, RxAction)
                        # Search time synchronizer
                        time_sync_tracker: None | ApproximateTimeSynchronizerTracker = None
                        for tsi, ts in self.time_sync_models[data.node].items():
                            if data.cause.input_topic in tsi.input_topics:
                                time_sync_tracker = ts
                                break
                        assert time_sync_tracker is not None
                        callback_occurs = time_sync_tracker.test_input(data.cause.input_topic, data.data)
                        if callback_occurs:
                            self.l.info(
                                f"    Action is ready and has no constraints, and (multi-input-)callback will occur: RX of {data.topic} ({type(data.data).__name__}) at node {data.node}. Publishing data...")
                        else:
                            self.l.info(
                                f"    Action is ready and has no constraints, but (multi-input-)callback will not occur: RX of {data.topic} ({type(data.data).__name__}) at node {data.node}. Publishing data and removing effects...")
                            for child in list(self.__causality_childs_of(graph_node_id)):
                                child_data: Action = self.graph.nodes[child]["data"]
                                if not isinstance(child_data, OrchestratorStatusAction):
                                    self.l.debug(f"Removing effect {child_data}")
                                    self.__remove_node_recursive(child)
                        data.state = ActionState.RUNNING
                        self.interception_pubs[data.node][data.topic].publish(data.data)
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
                # We are ready for data input as soon as there are no actions left waiting for topic
                # Note: We *could* already accept the input if the parents of all those buffer nodes are not
                # currently running. This might enable running more nodes from different timesteps in parallel.
                # This is not currently implemented, and the benefit might not be substantial.
                for _graph_node_id, data in self.__buffer_nodes_with_data():
                    if data.cause.input_topic == self.next_input.topic:
                        return False
                return True

    def __graph_is_empty(self) -> bool:
        return self.graph.number_of_nodes() == 0

    def __find_running_action(self, published_topic_name: TopicName) -> int:
        """Find running action which published the message on the specified topic"""
        for node_id, rd in self.graph.nodes(data=True):
            d: Action = rd["data"]  # type: ignore
            match d:
                case TimerCallbackAction() | RxAction():
                    if d.state == ActionState.RUNNING:
                        node_model = self.__node_model_by_name(d.node)
                        outputs = node_model.effects_for_input(d.cause)
                        for output in outputs:
                            if output == TopicPublish(published_topic_name):
                                return node_id
                case DataProviderInputAction():
                    if d.published_topic == published_topic_name and d.state == ActionState.RUNNING:
                        return node_id
                case OrchestratorBufferAction() | OrchestratorStatusAction():
                    pass

        raise ActionNotFoundError(f"There is no currently running action which should have published a message on topic \"{published_topic_name}\"")

    def __find_running_action_status(self, node_name: NodeName) -> GraphNodeId:
        for node_id, node_data in self.graph.nodes(data=True):

            d = cast(Action, node_data["data"])  # type: ignore
            match d:
                case OrchestratorStatusAction():
                    parent = self.__parent_node(node_id)
                    assert parent is not None
                    parent_data = cast(CallbackAction, self.graph.nodes[parent]["data"])
                    if parent_data.node == node_name:
                        return node_id
                case _:
                    pass
        raise ActionNotFoundError(f"There is no currently running action for node {node_name} which should have published a status message")

    def plot_graph(self):
        annotations = {}
        for node, node_data in self.graph.nodes(data=True):
            d: Action = node_data["data"]  # type: ignore
            match d:
                case RxAction():
                    annotations[node] = f"{d.node}: rx {d.cause.input_topic}"
                case TimerCallbackAction():
                    annotations[node] = f"{d.node}: timer @{d.timestamp}"
                case OrchestratorBufferAction():
                    annotations[node] = f"buffer {d.cause.input_topic}"
                case OrchestratorStatusAction():
                    annotations[node] = "rx status"
                case DataProviderInputAction():
                    annotations[node] = f"input {d.published_topic}"

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

        plot_instance = netgraph.InteractiveGraph(self.graph,
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

        if topic_name == "clock":

            clock_msg = cast(rosgraph_msgs.msg.Clock, msg)
            time_input = Time.from_msg(clock_msg.clock, clock_type=ClockType.SYSTEM_TIME)

            self.l.debug(f"Time input: {time_input}")
            self.l.debug(f"Simulator time: {self.simulator_time}")

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
            cause_action_id: None | GraphNodeId = None
            causing_action: None | CallbackAction | DataProviderInputAction = None

            cause_action_id = self.__find_running_action(topic_name)
            causing_action = cast(CallbackAction | DataProviderInputAction, self.graph.nodes[cause_action_id]["data"])

            # Buffer this data for next actions

            # list creation to allow graph modification while iterating
            # Iterate over all children which are buffers
            for buffer_id, buffer_data in list(self.__buffer_childs_of_parent(cause_action_id)):
                if buffer_data.cause.input_topic == topic_name:
                    i = 0
                    # Iterate over all callbacks below this buffer
                    for child_id in list(self.__causality_childs_of(buffer_id)):
                        action: Action = self.graph.nodes[child_id]["data"]
                        match action:
                            case RxAction():
                                assert (action.state == ActionState.WAITING)
                                assert (action.data is None)
                                self.l.debug(f" Buffering data and readying action and removing edge to parent: {child_id}: {action}")
                                action.data = msg
                                action.state = ActionState.READY
                                i += 1
                            case _:
                                raise RuntimeError(f"Action {action} ({child_id}) is a child of a buffer action but not an RxAction!")
                    self.l.info(f" Buffered data at all {i} subsequent actions, deleting buffer node")
                    self.graph.remove_node(buffer_id)
                    break  # Only handle first buffer node

            if self.__in_degree_by_type(cause_action_id, EdgeType.CAUSALITY) == 0:
                # Only remove the node if no other callbacks require input from it.
                # Otherwise, a callback publishing multiple topics would be removed too early!
                assert causing_action is not None
                self.l.info(f"  This completes the {type(causing_action).__name__} action! Removing...")
                self.graph.remove_node(cause_action_id)

        self.__process()

    def __in_degree_by_type(self, node: GraphNodeId, edge_type: EdgeType) -> int:
        """ Returns the in-degree of an edge filtered by edge type """
        degree = 0
        for u, v, edge_data in self.graph.in_edges(node, data=True):  # type: ignore
            type: EdgeType = edge_data["edge_type"]  # type: ignore
            if type == edge_type:
                degree += 1
        return degree

    def __parent_node(self, node: GraphNodeId) -> GraphNodeId | None:
        parent = None
        for u, v, edge_data in self.graph.out_edges(node, data=True):  # type: ignore
            type: EdgeType = edge_data["edge_type"]  # type: ignore
            if type == EdgeType.CAUSALITY:
                if parent is not None:
                    raise RuntimeError(f"Mutliple parent nodes of node {node}: {parent} and {v}")
                parent = v
        return parent

    def __node_model_by_name(self, name) -> NodeModel:
        for node in self.node_models:
            if node.get_name() == name:
                return node

        raise KeyError(f"No model for node \"{name}\"")

    def __status_callback(self, msg: Status):
        lc(self.l, f"Received status message from {msg.node_name}")

        status_node_id = self.__find_running_action_status(msg.node_name)

        cause_action_id = self.__parent_node(status_node_id)
        assert cause_action_id is not None
        causing_action = self.graph.nodes[cause_action_id]["data"]

        self.graph.remove_node(status_node_id)

        in_degree = self.__in_degree_by_type(cause_action_id, EdgeType.CAUSALITY)
        self.l.debug(f"cause action is node {cause_action_id}: {causing_action} with in-degree {in_degree}")

        # Complete the action which sent this message
        if in_degree == 0:
            self.l.info(f"  This completes the {type(causing_action).__name__} callback! Removing...")
            self.graph.remove_node(cause_action_id)

        self.__process()
