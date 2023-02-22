from orchestrator.orchestrator_lib.model_loader import load_launch_config_schema

import json
from jsonschema import validate


def test_schema_load():
    load_launch_config_schema()


def test_schema_validation():
    schema = load_launch_config_schema()
    testdata: str = """
    {
        "inputs": [
            {
                "name": "meas/radar",
                "type": "orchestrator_interfaces/msg/SampleMessage"
            }
        ],
        "nodes": {
            "detector_radar": {
                "config_file": [
                    "orchestrator_dummy_nodes",
                    "detector_node_config.json"
                ],
                "remappings": {
                    "input": "meas/radar",
                    "output": "detections/radar"
                }
            }
        }
    }
    """
    config = json.loads(testdata)
    validate(instance=config, schema=schema)



def test_node_config_load():
    schema = load_launch_config_schema()
    testdata: str = """
    {
        "inputs": [],
        "nodes": {
            "detector_radar": {
                "config_file": [
                    "orchestrator_dummy_nodes",
                    "detector_node_config.json"
                ],
                "remappings": { }
            }
        }
    }
    """
    config = json.loads(testdata)
    validate(instance=config, schema=schema)
