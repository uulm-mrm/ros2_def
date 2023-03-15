from .node_model_from_file import ConfigFileNodeModel

from orchestrator.orchestrator_lib.node_model import NodeModel

from ament_index_python.packages import get_package_share_path

import json
from jsonschema import validate


def load_launch_config_schema():
    orchestrator_package_path = get_package_share_path("orchestrator")
    with open(orchestrator_package_path/"schemas"/"launch_config_schema.json") as f:
        launch_config_schema = json.load(f)
    return launch_config_schema


def load_node_config_schema():
    orchestrator_package_path = get_package_share_path("orchestrator")
    with open(orchestrator_package_path/"schemas"/"node_config_schema.json") as f:
        node_config_schema = json.load(f)
    return node_config_schema


def load_node_config(package: str, name: str, schema):
    path = get_package_share_path(package)/"configs"/name
    with open(path) as f:
        node_config = json.load(f)
    validate(instance=node_config, schema=schema)
    return node_config


def load_launch_config(package, name, schema):
    launch_config_package_path = get_package_share_path(package)
    launch_config_path = launch_config_package_path / "configs" / name
    with open(launch_config_path) as f:
        launch_config = json.load(f)
    validate(instance=launch_config, schema=schema)
    return launch_config


def load_models(launch_config, node_config_schema) -> list[NodeModel]:
    models = []
    for name, node in launch_config["nodes"].items():
        remappings: dict[str, str] = node.get("remappings", {})
        package, filename = node["config_file"]
        config = load_node_config(package, filename, node_config_schema)
        model = ConfigFileNodeModel(config, name, remappings)
        models.append(model)

    return models
