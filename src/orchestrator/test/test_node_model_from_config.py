from orchestrator.node_model_from_file import ConfigFileNodeModel
from orchestrator.model_loader import load_node_config, load_node_config_schema, load_launch_config, load_launch_config_schema
from orchestrator_dummy_nodes.orchestrator_lib.node_model import NodeModel, StatusPublish, TopicInput, TopicPublish


def assert_tracking_model_correct(model: NodeModel):
    assert set(model.get_possible_inputs()) == set([
        TopicInput("detections/radar"),
        TopicInput("detections/camera"),
        TopicInput("detections/lidar")])

    assert model.effects_for_input(TopicInput("detections/radar")) == [StatusPublish()]
    assert model.effects_for_input(TopicInput("detections/camera")) == [StatusPublish()]
    assert model.effects_for_input(TopicInput("detections/lidar")) == [TopicPublish("tracks")]


def test_model_load():
    test_package = "orchestrator_dummy_nodes"
    test_node_config = "tracking_node_config.json"

    node_schema = load_node_config_schema()
    node_config = load_node_config(
        test_package,
        test_node_config,
        node_schema)
    launch_schema = load_launch_config_schema()
    launch_config = load_launch_config(
        test_package,
        "tracking_example_launch_config.json",
        launch_schema)

    node_name = "tracking"
    instance_config = launch_config["nodes"][node_name]
    assert instance_config["config_file"] == [test_package, test_node_config]

    remappings = instance_config["remappings"]

    model = ConfigFileNodeModel(node_config, node_name, remappings)
    assert_tracking_model_correct(model)
