from rclpy.lifecycle import Publisher
from rclpy.node import Node, MsgType
from rclpy.subscription import Subscription
from typing import Callable, Any, Union, List

from orchestrator_interfaces.msg import Status


class OrchestratorWrapperNode:
    def __init__(self, node: Node, topics : Union[List[str], None] = None):
    """
    :param topics: List of topics for which status messages should be published.
                   If None, status message is published for every topic input.
    """
        self.node = node
        self.topics = topics
        self.callbacks: dict[str, Callable[[MsgType], None]] = {}
        self.orchestrator_status_pub: Publisher = self.node.create_publisher(Status, "/status", 10)

    def publish_status(self) -> None:
        status_msg = Status()
        status_msg.node_name = self.node.get_name()
        self.orchestrator_status_pub.publish(status_msg)

    def create_subscription(self, topic_type: type, topic: str, callback: Callable[[MsgType], None], *args: Any,
                            **kwargs: Any) -> Subscription:
        self.callbacks[topic] = callback
        callback = (lambda msg, topic=topic: self.handle(msg, topic)) if self.topics is None or topic in self.topics else callback
        return self.node.create_subscription(topic_type, topic, callback, *args,
                                             **kwargs)

    def destroy_subscription(self, subscription: Subscription) -> bool:
        return self.node.destroy_subscription(subscription)

    def handle(self, msg: MsgType, topic: str) -> None:
        self.callbacks[topic](msg)
        self.publish_status()
