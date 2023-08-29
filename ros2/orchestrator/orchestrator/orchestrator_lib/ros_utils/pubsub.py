# pyright: strict

from typing import Any, Type, Optional

from rclpy.task import Future
from rclpy.executors import Executor
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger

from orchestrator.orchestrator_lib.name_utils import TopicName, type_from_string


def wait_for_topic(name: TopicName, logger: RcutilsLogger, node: Node, executor: Executor) -> Type[Any]:
    name = node.resolve_topic_name(name)

    def find_type():
        for topic_name, msgtypes in node.get_topic_names_and_types():
            if topic_name == name:
                if len(msgtypes) != 1:
                    raise RuntimeError(f"Message type for topic {topic_name} is not unique: {msgtypes}")
                return type_from_string(msgtypes[0])
        return None

    msgtype = find_type()
    if msgtype is None:
        logger.info(f"  Waiting for topic \"{name}\" to exist")
    while msgtype is None:
        executor.spin_until_future_complete(Future(), 0.1)
        msgtype = find_type()

    return msgtype


def wait_for_node_sub(topic_name: str, node_name: str, logger: RcutilsLogger, node: Node, executor: Executor) -> Type[
    Any]:
    topic_name = node.resolve_topic_name(topic_name)

    def try_get_type() -> Optional[Type[Any]]:
        for info in node.get_subscriptions_info_by_topic(topic_name):
            if info.node_name == node_name:
                return type_from_string(info.topic_type)
        return None

    topic_type = try_get_type()

    if topic_type:
        logger.info(
            f"  Node \"{node_name}\" is already subscribed to \"{topic_name}\"")
    else:
        logger.info(
            f"  Waiting for node \"{node_name}\" to subscribe to \"{topic_name}\"")

    while not topic_type:
        executor.spin_until_future_complete(Future(), 0.1)
        topic_type = try_get_type()

    return topic_type


def wait_for_node_pub(topic_name: str, node_name: str, logger: RcutilsLogger, node: Node, executor: Executor) -> None:
    topic_name = node.resolve_topic_name(topic_name, only_expand=True)

    def node_has_pub():
        by_node = node.get_publisher_names_and_types_by_node(node_name, node.get_namespace())
        for topic, _types in by_node:
            if topic == topic_name:
                return True
        return False

    if node_has_pub():
        logger.info(
            f"  Node \"{node_name}\" already has a publisher for \"{topic_name}\"")
    else:
        logger.info(
            f"  Waiting for node \"{node_name}\" to create a publisher for \"{topic_name}\"")

    while not node_has_pub():
        executor.spin_until_future_complete(Future(), 1.0)
