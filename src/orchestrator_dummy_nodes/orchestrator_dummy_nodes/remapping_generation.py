import json
import sys

from .orchestrator_lib.name_utils import intercepted_name


def main():
    filename = sys.argv[1]
    launch_config = None
    with open(filename) as f:
        launch_config = json.load(f)
    # TODO: Validate json
    for node_name, node in launch_config["nodes"].items():
        for internal_name, ros_name in node["inputs"].items():
            print(f"Remap input \"{internal_name}\" of node \"{node_name}\" to \"{intercepted_name(node_name, ros_name)}\"")  # type: ignore


if __name__ == '__main__':
    main()
