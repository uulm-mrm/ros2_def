from typing import Type

import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger

from orchestrator.orchestrator_lib.name_utils import TopicName, type_from_string


def wait_for_topic(name: TopicName, logger: RcutilsLogger, node: Node) -> Type:
    name = node.resolve_topic_name(name)

    def find_type():
        for topic_name, msgtypes in node.get_topic_names_and_types():
            if topic_name == name:
                assert len(msgtypes) == 1
                return type_from_string(msgtypes[0])
        return None

    msgtype = find_type()
    if msgtype is None:
        logger.info(f"  Waiting for topic \"{name}\"")
    while msgtype is None:
        if node.executor is not None:
            node.executor.spin_until_future_complete(Future(), 0.1)
        else:
            rclpy.spin_until_future_complete(node, Future(), timeout_sec=0.1)
        msgtype = find_type()

    return msgtype


def wait_for_node_sub(topic_name: str, node_name: str, logger: RcutilsLogger, node: Node) -> Type:
    topic_name = node.resolve_topic_name(topic_name)

    def try_get_type() -> Type | None:
        for info in node.get_subscriptions_info_by_topic(topic_name):
            if info.node_name == node_name:
                return type_from_string(info.topic_type)
        return None

    topic_type = try_get_type()

    if topic_type:
        logger.info(f"  Node \"{node_name}\" is already subscribed to \"{topic_name}\"")
    else:
        logger.info(f"  Waiting for node \"{node_name}\" to subscribe to \"{topic_name}\"")

    while not topic_type:
        if node.executor is not None:
            node.executor.spin_until_future_complete(Future(), 0.1)
        else:
            rclpy.spin_until_future_complete(node, Future(), timeout_sec=0.1)
        topic_type = try_get_type()

    return topic_type


def wait_for_node_pub(topic_name: str, node_name: str, logger: RcutilsLogger, node: Node):
    topic_name = node.resolve_topic_name(topic_name)

    def node_has_pub():
        for info in node.get_publishers_info_by_topic(topic_name):
            if info.node_name == node_name:
                return True
        return False

    if node_has_pub():
        logger.info(f"  Node \"{node_name}\" already has a publisher for \"{topic_name}\"")
    else:
        logger.info(f"  Waiting for node \"{node_name}\" to create a publisher for \"{topic_name}\"")

    while not node_has_pub():
        if node.executor is not None:
            node.executor.spin_until_future_complete(Future(), 0.1)
        else:
            rclpy.spin_until_future_complete(node, Future(), timeout_sec=0.1)
