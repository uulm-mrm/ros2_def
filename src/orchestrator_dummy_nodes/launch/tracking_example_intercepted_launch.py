from launch import LaunchDescription
from launch_ros.actions import Node
from orchestrator_dummy_nodes.topic_remapping import intercepted_name
from orchestrator_dummy_nodes.tracking_example_launchutil import get_tracking_nodes


def generate_launch_description():
    return get_tracking_nodes(lambda node_name, topic: intercepted_name(node_name, topic))
