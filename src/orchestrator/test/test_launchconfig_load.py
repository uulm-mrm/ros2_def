from test_node_model_from_config import assert_tracking_model_correct

from orchestrator.orchestrator_lib.model_loader import load_launch_config, load_launch_config_schema, load_models, load_node_config_schema


def test_load_tracking_example():
    test_package = "orchestrator_dummy_nodes"

    launch_schema = load_launch_config_schema()
    launch_config = load_launch_config(
        test_package,
        "tracking_example_launch_config.json",
        launch_schema)

    model_config_schema = load_node_config_schema()
    models = load_models(launch_config, model_config_schema)

    assert len(models) == 7

    tracking_index = None
    for i, model in enumerate(models):
        if model.get_name() == "tracking":
            assert tracking_index is None
            tracking_index = i

    assert tracking_index is not None
    assert_tracking_model_correct(models[tracking_index])
