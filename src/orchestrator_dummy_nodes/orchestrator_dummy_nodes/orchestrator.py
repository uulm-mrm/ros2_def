# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


import time
from typing import Any, TypeAlias
from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish
from orchestrator_dummy_nodes.topic_remapping import collect_intercepted_topics, type_from_string
from orchestrator_dummy_nodes.tracking_example_configuration import nodes as node_config
from orchestrator_dummy_nodes.tracking_example_configuration import external_input_topics as external_input_topics_config

from rclpy.node import Node, Subscription, Publisher
import rclpy
from rclpy.logging import get_logger

from orchestrator_interfaces.msg import Status

# Buffered message = multiple causes (one per subscription/node)


TopicName: TypeAlias = str
NodeName: TypeAlias = str


def l(msg):
    return get_logger("l").info(msg)


def lc(msg):
    return get_logger("l").info('\033[96m' + msg + '\033[0m')


def d(msg):
    return get_logger("l").debug(msg)


class Orchestrator(Node):
    def __init__(self) -> None:
        super().__init__("orchestrator")  # type: ignore

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
        #self.external_input_subs: list[Subscription] = []

        # Subscriptions for outputs which we do not need to buffer,
        #  but we need to inform the node models that they happened
        self.modeled_node_output_subs: dict[TopicName, Subscription] = {}

        # Waiting for occurence of those effects from nodes
        self.expected_effect_queue: list[tuple[str, Effect]] = []

        # Yet to be released causes for nodes
        # Tuples (node, cause, data) ("detector", TopicInput("input"), MeasurementMessage(...))
        self.cause_queue: list[tuple[str, Cause, Any]] = []

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

        # Outputs which are not intercepted (should probably only be the very last nodes in the graph)
        for node in self.node_models:
            # TODO: Skip topics which do not correspond to effects? Do those exist?
            for internal_name, topic_name in node.output_remappings:
                if topic_name in self.interception_subs:
                    continue
                if topic_name in self.modeled_node_output_subs:
                    continue
                if topic_name == "status":
                    continue
                topic_infos = self.get_publishers_info_by_topic(topic_name)
                assert len(topic_infos) > 0
                type = type_from_string(topic_infos[0].topic_type)
                l(f"Subscribing to {topic_name} ({type.__name__}) because it is an output of a node model")
                subscription = self.create_subscription(
                    type, topic_name,
                    lambda msg, topic_name=topic_name, node_model=node: self.non_intercepted_output_callback(topic_name, node_model, msg),
                    10)
                self.modeled_node_output_subs[topic_name] = subscription

    def subscriptions_match_model(self) -> bool:
        # TODO: Check if all inputs are intercepted? -> One publisher for each node model input, One subscription for topic
        # TODO: Check if all intercepted subs have corresponding models?
        model_inputs = []
        intercepted_subs = []
        interception_pubs = []

        for node in self.node_models:
            node.input_remappings

        raise NotImplementedError()

    def non_intercepted_output_callback(self, topic_name: TopicName, node: NodeModel, msg: Any):
        lc(f"Received message on output topic {topic_name}")
        node.handle_event(TopicPublish(topic_name))

    def process_queue(self):
        lc(f"Processing queue of length {len(self.cause_queue)}")
        changed_anything: bool = False
        processed_tasks = []
        for (node_name, cause, data) in self.cause_queue:
            d(f" Considering {cause} for {node_name}...")
            node = self.node_model_by_name(node_name)
            if node.ready_for_input(cause):
                d(f"  Node is ready for input")
                if isinstance(cause, TopicInput):
                    publisher = self.interception_pubs[node_name][cause.input_topic]
                    node_effects = node.effects_for_input(cause)
                    d(f"  Expected effects: {node_effects}")
                    self.expected_effect_queue += [(node_name, effect) for effect in node_effects]
                    d(f"  Informing node model about publish")
                    node.process_input(cause)
                    d(f"  Publishing message for {node_name} at {publisher.topic_name}")
                    publisher.publish(data)
                    processed_tasks.append((node_name, cause, data))
                    changed_anything = True
                else:
                    l(f"  Cause {cause} can not be handled!")
            else:
                d(f"  Node is not ready for input")

        for processed in processed_tasks:
            self.cause_queue.remove(processed)

        if changed_anything:
            self.process_queue()

    def interception_subscription_callback(self, topic_name: TopicName, msg: Any):
        lc(f"Received message on intercepted topic {topic_name}")

        if topic_name in [name for (_type, name) in self.external_input_topics]:
            l(" This is an external input, not triggered by orchestrator.")
        else:
            node_name = None
            for node, effect in self.expected_effect_queue:
                if effect == TopicPublish(topic_name):
                    node_name = node
                    break
            if node_name is None:
                raise RuntimeError(f"Topic publish for {topic_name} was not expected by any node!")
            l(" Informing node model about event.")
            self.node_model_by_name(node_name).handle_event(TopicPublish(topic_name))
            l(" Removing event from queue of expected events.")
            self.expected_effect_queue.remove((node_name, TopicPublish(topic_name)))

            # For expected/arriving input, create receiving actions (waiting) for each initial subscriber
            # Recursively, for each added action, add successor actions. Add parent as dependency
            # If actions for the same node already exist, add as dependencies (this ensures sequence-determinism at each node)
            # 
            # To advance processing, execute all actions which are ready and have no dependencies (setting it to running)
            # 
            # For incoming messages, set actions requiring it to ready, and remove the (running) action causing it


        cause = TopicInput(topic_name)
        effects: dict[str, list[Effect]] = {}
        for node in self.node_models:
            if cause in node.get_possible_inputs():
                effects[node.get_name()] = node.effects_for_input(cause)

        l(f" This input has the following effects: {effects}")

        # Send this to all nodes which can receive it
        causes = [(node, cause, msg) for node in effects.keys()]
        self.cause_queue += causes
        l(f" Queued sending this to all receiving nodes (queue length now {len(self.cause_queue)})")

        self.process_queue()

    def node_model_by_name(self, name) -> NodeModel:
        for node in self.node_models:
            if node.get_name() == name:
                return node

        raise KeyError(f"No model for node \"{name}\"")

    def status_callback(self, msg: Status):
        lc(f"Received status message from node {msg.node_name}")
        l(" Removing expected effect from queue")
        self.expected_effect_queue.remove((msg.node_name, StatusPublish()))
        l(" Notifying node model of event")
        self.node_model_by_name(msg.node_name).handle_event(StatusPublish())
        self.process_queue()


def main():
    rclpy.init()
    orchestrator = Orchestrator()
    rclpy.spin(orchestrator)
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
