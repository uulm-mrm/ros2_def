from launch import LaunchDescription
from orchestrator_dummy_nodes.tracking_example_launchutil import get_tracking_nodes


def generate_launch_description():
    return LaunchDescription(get_tracking_nodes(lambda _node_name, topic: topic))
