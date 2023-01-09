from typing import Optional
from rclpy.task import Future
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
import rclpy.type_support

#from std_msgs.msg import String
import std_msgs.msg
import importlib

import time


def type_from_string(typestring: str):
    parts = typestring.split("/")
    module = importlib.import_module(parts[0]+"."+parts[1])
    return getattr(module, parts[2])


def todo(msg):
    raise NotImplementedError(msg)


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
            return False
        if not topic.startswith("topic_"):
            return False
        if self.busy:
            return False
        input = int(topic.split("_")[1])
        return input == self.next_topic

    def process_status(self, status_message: std_msgs.msg.String):
        next_topic = int(status_message.data)+1
        next_topic %= 5
        self.next_topic = next_topic
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

        self.nodes: dict[str, GraphNode] = {"minimal_subscriber": MultiSubscriberNode()}

        self.status_subscription = self.create_subscription(std_msgs.msg.String, "status", self.callback_status, 10)

        for topic, types in self.get_topic_names_and_types():
            # Syntax: intercepted__sub__NODE_NAME__TOPIC for subscriptions
            if topic.startswith("/intercepted__"):
                for type in types:
                    type = type_from_string(type)
                    name_parts = topic.split("__")
                    if name_parts[1] == "sub":
                        node_name = name_parts[2]
                        real_topic_name = name_parts[3]
                        print(f"Found topic {topic} as subscription by {node_name}, real name was {real_topic_name}. Type: {type}")
                        if real_topic_name not in self.interception_subs:
                            self.interception_subs[real_topic_name] = \
                                self.create_subscription(type, real_topic_name,
                                                         lambda msg, real_topic_name=real_topic_name: self.callback(msg, real_topic_name), 10)
                        self.interception_pubs.setdefault(real_topic_name, {})[node_name] = self.create_publisher(type, topic, 10)
                    else:
                        raise RuntimeError(f"Topic name {topic} does not follow syntax "
                                           "\"intercepted__sub__NODE_NAME__TOPIC\"")
        self.process_buffers()

    def request_new_sample(self):
        print("Requesting new sample")
        msg = std_msgs.msg.String()
        self.request_pub.publish(msg)

    def process_buffers(self):
        print("Processing Buffers")
        published_something = False

        changed = True
        while changed:
            changed = False
            for buffer in self.buffered_msgs:
                time.sleep(0.5)
                print(f"  Processing buffer {buffer}")
                to_remove = []
                for receiver in buffer.remaining_receiver_nodes:
                    print(f"    Buffer needs to be sent to {receiver}")
                    if self.nodes[receiver].can_accept(buffer.destination_topic, buffer.data):
                        published_something = True
                        changed = True
                        self.interception_pubs[buffer.destination_topic][receiver].publish(buffer.data)
                        self.nodes[receiver].busy = True
                        to_remove.append(receiver)
                        print(f"    Published message on topic {buffer.destination_topic} for node {receiver}")
                    else:
                        print(f"    Receiver is not ready.")
                for r in to_remove:
                    buffer.remaining_receiver_nodes.remove(r)
            self.buffered_msgs = [m for m in self.buffered_msgs if not m.done()]

        if not published_something:
            # TODO: This requests way too often. Find out when to request new sample.
            time.sleep(0.5)
            self.request_new_sample()

    def callback(self, msg: any, real_topic_name: str):
        print(f"Got message {msg} on topic {real_topic_name}")
        destination_nodes = set(self.interception_pubs[real_topic_name].keys())
        print(f"This message is intended for nodes: {destination_nodes}. Buffering...")
        self.buffered_msgs.append(BufferedMsg(msg, real_topic_name, destination_nodes))
        self.process_buffers()

    def callback_status(self, msg: std_msgs.msg.String):
        # TODO: Get node name, etc...
        # assume this status is from multi subscriber 'minimal_subscriber'
        print(f"Got status callback: {msg}")
        for name, node in self.nodes.items():
            node.process_status(msg)
        self.process_buffers()


def main(args=None):
    rclpy.init(args=args)

    interceptor = SInterceptor()

    rclpy.spin(interceptor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interceptor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
