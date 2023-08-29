from rclpy.node import Node

from orchestrator_interfaces.msg import Status


class OrchestratorWrapperNode:
    def __init__(self, node: Node):
        self.node = node
        self.callbacks = {}
        self.orchestrator_status_pub = self.node.create_publisher(Status, "/status", 10)

    def publish_status(self):
        status_msg = Status()
        status_msg.node_name = self.node.get_name()
        self.orchestrator_status_pub.publish(status_msg)

    def create_subscription(self, topic_type, topic, callback, *args, **kwargs):
        self.callbacks[topic] = callback
        return self.node.create_subscription(topic_type, topic, lambda msg, topic=topic: self.handle(msg, topic), *args,
                                             **kwargs)

    def destroy_subscription(self, subscription):
        self.node.destroy_subscription(subscription)

    def handle(self, msg, topic):
        self.callbacks[topic](msg)
        self.publish_status()
