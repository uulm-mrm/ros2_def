from pathlib import Path
from typing import List, Dict

from .node_model_from_file import ConfigFileNodeModel

from orchestrator.orchestrator_lib.node_model import NodeModel

from ament_index_python.packages import get_package_share_path, PackageNotFoundError

import json
from jsonschema import validate


def load_launch_config_schema():
    orchestrator_package_path = get_package_share_path("orchestrator")
    with open(orchestrator_package_path / "schemas" / "launch_config_schema.json") as f:
        launch_config_schema = json.load(f)
    return launch_config_schema


def load_node_config_schema():
    orchestrator_package_path = get_package_share_path("orchestrator")
    with open(orchestrator_package_path / "schemas" / "node_config_schema.json") as f:
        node_config_schema = json.load(f)
    return node_config_schema


def _get_config_path(package: str, name: str) -> Path:
    """Resolve path to config file, supporting deprecated "configs" paths with warnings"""
    package_share_path = get_package_share_path(package)
    configs_path = package_share_path / "configs" / name
    config_path = package_share_path / "config" / name
    if config_path.exists() and not configs_path.exists():
        # Only correct path exists
        return config_path
    elif not config_path.exists() and configs_path.exists():
        # Only old path exists
        print(f"Orchestrator Warning: Loading configuration from deprecated location at {configs_path}."
              " Please use the \"config\" directory instead of \"configs\"")
        return configs_path
    elif config_path.exists() and configs_path.exists():
        # Both exist
        raise RuntimeError(f"Requested config file exists at {config_path} and {configs_path}."
                           " Please remove the file in the \"configs\" directory to prevent ambiguity.")
    else:
        # None exist
        raise RuntimeError(f"Config file {name} from package {package} not found at expected location {config_path}.")


def load_node_config(package: str, name: str, schema):
    try:
        path = _get_config_path(package, name)
    except PackageNotFoundError:
        raise RuntimeError(f"Could not load node config {name}, because package {package} was not found!")
    with open(path) as f:
        node_config = json.load(f)
    validate(instance=node_config, schema=schema)
    return node_config


def load_launch_config(package, name, schema):
    launch_config_path = _get_config_path(package, name)
    with open(launch_config_path) as f:
        launch_config = json.load(f)
    validate(instance=launch_config, schema=schema)
    return launch_config


def load_models(launch_config, node_config_schema) -> List[NodeModel]:
    models = []
    for name, node in launch_config["nodes"].items():
        remappings: Dict[str, str] = node.get("remappings", {})
        package, filename = node["config_file"]
        state_sequence = node.get("state_sequence", None)
        try:
            config = load_node_config(package, filename, node_config_schema)
        except Exception as e:
            raise RuntimeError(
                f"Error while trying to load the node config for \"{name}\" from {filename} in package {package}: {e}")

        model = ConfigFileNodeModel(config, name, remappings, state_sequence)
        models.append(model)

    return models
