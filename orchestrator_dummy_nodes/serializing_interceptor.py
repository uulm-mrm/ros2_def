from orchestrator_dummy_nodes.topic_remapping import initial_name_from_intercepted
from rclpy.task import Future
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
import rclpy.type_support
import rclpy.logging

from orchestrator_interfaces.msg import Status

import std_msgs.msg
import importlib


def l():
    return rclpy.logging.get_logger("interceptor")


def type_from_string(typestring: str):
    parts = typestring.split("/")
    module = importlib.import_module(parts[0]+"."+parts[1])
    return getattr(module, parts[2])


class GraphNode:
    def __init__(self):
        pass

    def can_accept(self, topic: str, message: any) -> bool:
        """
        Indicates if the node has completed all prerequisites for processing the specified input
        """
        pass

    def process_status(self, status_message: std_msgs.msg.String):
        pass


class MultiSubscriberNode(GraphNode):
    def __init__(self):
        super().__init__()
        self.busy = False
        self.next_topic = 0

    def can_accept(self, topic: str, input: any) -> bool:
        if not isinstance(input, std_msgs.msg.String):
            l().debug("Can only accept string messages")
            return False
        if not topic.startswith("topic_"):
            l().debug("Can only accept topics named topic_*")
            return False
        if self.busy:
            l().debug("Is busy")
            return False
        input = int(topic.split("_")[1])

        if input != self.next_topic:
            l().debug(f"Was expecting topic {self.next_topic} next")
            return False
        return True

    def process_status(self, status_message: Status):
        self.next_topic += 1
        self.next_topic %= 5
        self.busy = False


class BufferedMsg:
    def __init__(self, data: any, destination_topic: str, receiver_nodes: set[str]):
        self.data: any = data
        self.destination_topic: str = destination_topic
        self.remaining_receiver_nodes: set[str] = receiver_nodes

    def done(self):
        return len(self.remaining_receiver_nodes) == 0

    def __str__(self) -> str:
        return f"BufferedMsg{{to: {self.destination_topic}, nodes: {self.remaining_receiver_nodes}, data: {self.data}}}"


class SInterceptor(Node):
    def __init__(self):
        super().__init__('interceptor')
        self.interception_subs: dict[str, Subscription] = {}  # Subscribe to topic directly
        # TODO: Multiple subscribtions for same topic in same node?
        #                            Topic     Node
        self.interception_pubs: dict[str, dict[str, Publisher]] = {}  # Republish on separate topic for each sub

        self.buffered_msgs: list[BufferedMsg] = []

        self.request_pub = self.create_publisher(std_msgs.msg.String, "trigger_sample", 10)

        self.nodes: dict[str, GraphNode] = {"subscriber": MultiSubscriberNode()}

        self.status_subscription = self.create_subscription(Status, "status", self.callback_status, 10)

        for topic, types in self.get_topic_names_and_types():
            if topic.startswith("/intercepted"):
                for type in types:
                    type = type_from_string(type)
                    node_name, real_topic_name = initial_name_from_intercepted(topic)

                    self.get_logger().info(f"Found topic {topic} as subscription by {node_name}, real name was {real_topic_name}. Type: {type}")
                    if real_topic_name not in self.interception_subs:
                        self.interception_subs[real_topic_name] = \
                            self.create_subscription(type, real_topic_name,
                                                     lambda msg, real_topic_name=real_topic_name: self.callback(msg, real_topic_name), 10)
                    self.interception_pubs.setdefault(real_topic_name, {})[node_name] = self.create_publisher(type, topic, 10)

        self.process_buffers()

    def request_new_sample(self):
        self.get_logger().info("Requesting new sample")
        msg = std_msgs.msg.String()
        self.request_pub.publish(msg)

    def process_buffers(self):
        self.get_logger().info(f"Processing {len(self.buffered_msgs)} buffers")

        changed = True
        while changed:
            changed = False
            for buffer in self.buffered_msgs:
                self.get_logger().debug(f"  Processing buffer {buffer}")
                to_remove = []
                for receiver in buffer.remaining_receiver_nodes:
                    self.get_logger().debug(f"    Buffer needs to be sent to {receiver}")
                    if self.nodes[receiver].can_accept(buffer.destination_topic, buffer.data):
                        changed = True
                        self.interception_pubs[buffer.destination_topic][receiver].publish(buffer.data)
                        self.nodes[receiver].busy = True
                        to_remove.append(receiver)
                        self.get_logger().info(f"    Published message on topic {buffer.destination_topic} for node {receiver}")
                    else:
                        self.get_logger().debug(f"    Receiver is not ready.")
                for r in to_remove:
                    buffer.remaining_receiver_nodes.remove(r)
            self.buffered_msgs = [m for m in self.buffered_msgs if not m.done()]

        all_free = True
        for _name, node in self.nodes.items():
            if node.busy:
                all_free = False
                break
        if all_free and len(self.buffered_msgs) == 0:
            self.request_new_sample()

    def callback(self, msg: any, real_topic_name: str):
        self.get_logger().info(f"Got message {msg} on topic {real_topic_name}")
        destination_nodes = set(self.interception_pubs[real_topic_name].keys())
        self.get_logger().debug(f"  This message is intended for nodes: {destination_nodes}. Buffering...")
        self.buffered_msgs.append(BufferedMsg(msg, real_topic_name, destination_nodes))
        self.process_buffers()

    def callback_status(self, msg: Status):
        self.get_logger().debug(f"Got status callback: {msg}")
        if msg.node_name not in self.nodes:
            l().warning(f"Received status for unknown node \"{msg.node_name}\"")
            return
        self.nodes[msg.node_name].process_status(msg)
        self.process_buffers()


def main(args=None):
    rclpy.init(args=args)

    interceptor = SInterceptor()
    rclpy.spin(interceptor)
    interceptor.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
