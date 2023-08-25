import sys

from typing import List, Dict

from orchestrator.orchestrator_lib.node_model import TopicInput, TimerInput
from orchestrator.orchestrator_lib.name_utils import intercepted_name, normalize_topic_name
from .model_loader import *

from launch_ros.actions import SetRemap
from launch.substitutions import TextSubstitution


def _find_node_model(name: str, models: List[NodeModel]) -> NodeModel:
    for model in models:
        if name == model.get_name():
            return model

    raise RuntimeError(f"No model for node with name {name}")


def _contains_timer_events(node: NodeModel):
    for input in node.get_possible_inputs():
        if isinstance(input, TimerInput):
            return True
    return False


def generate_remappings_from_config_file(package_name: str, launch_config_file: str) -> List[SetRemap]:
    """
    Generate remappings for topic interception by orchestrator.

    Generates interception remapping for each input topic.
    Additionally intercepts /clock if time-triggered callbacks exist.
    """
    print(f"Using launch config {package_name} {launch_config_file}")
    launch_config = load_launch_config(
        package_name, launch_config_file, load_launch_config_schema())
    return generate_remappings_from_config(launch_config)


def generate_remappings_from_config(launch_config: any) -> List[SetRemap]:
    """
    Generate remappings for topic interception by orchestrator.

    Generates interception remapping for each input topic.
    Additionally intercepts /clock if time-triggered callbacks exist.

    :param launch_config: Deserialized launch config as dict
    """
    node_models = load_models(launch_config, load_node_config_schema())

    remap_actions = []

    for node_name, node in launch_config["nodes"].items():
        remappings: Dict[str, str] = node.get("remappings", {})
        model = _find_node_model(node_name, node_models)
        for input in model.get_possible_inputs():
            if isinstance(input, TopicInput):
                # Add identity remapping for input topics if no explicit remapping exists.
                # This ensures that interception remapping is generated below.
                # TODO: This results in some useless remappings, from non-internal names...
                remappings.setdefault(input.input_topic, input.input_topic)

        for internal_name, ros_name in remappings.items():
            ros_name = normalize_topic_name(ros_name)
            if TopicInput(ros_name) not in model.get_possible_inputs():
                continue

            remap_src = f"{node_name}:{internal_name}"
            remap_dst = intercepted_name(node_name, ros_name)
            print(f"Remapping for node \"{node_name}\": \"{internal_name}\" to \"{remap_dst}\"")
            remap_actions.append(
                SetRemap(src=remap_src, dst=remap_dst)
            )
        if _contains_timer_events(model):
            remap_dst = intercepted_name(node_name, "clock")
            print(f"Remapping for node \"{node_name}\": \"clock\" to \"{remap_dst}\"")
            remap_actions.append(
                SetRemap(src=f"{node_name}:clock", dst=remap_dst))
    return remap_actions


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <package_name> <launch_config_file_name>")
        exit(1)

    package_name = sys.argv[1]
    config_name = sys.argv[2]

    remappings = generate_remappings_from_config_file(package_name, config_name)
    for r in remappings:
        if len(r.src) == 0 or not isinstance(r.src[0], TextSubstitution):
            raise RuntimeError(
                f"Source is not a TextSubstitution, no idea how to print that nicely: {r.src}")
        if len(r.dst) == 0 or not isinstance(r.dst[0], TextSubstitution):
            raise RuntimeError(
                f"Destination is not a TextSubstitution, no idea how to print that nicely: {r.src}")
        print(f"Remap {r.src[0].text} to {r.dst[0].text}")


if __name__ == '__main__':
    main()
