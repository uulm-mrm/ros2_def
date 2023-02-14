from launch import LaunchDescription
from orchestrator_dummy_nodes.tracking_example_launchutil import get_tracking_nodes
from orchestrator.remapping_generation import generate_remappings_from_config


def generate_launch_description():
    return LaunchDescription([
        *generate_remappings_from_config("orchestrator_dummy_nodes", "tracking_example_launch_config.json"),
        *get_tracking_nodes(lambda _node_name, topic: topic)
    ])
