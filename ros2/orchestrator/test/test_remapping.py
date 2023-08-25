import pytest
import rclpy.node
from launch import LaunchContext
from launch_ros.actions import SetRemap

from orchestrator.orchestrator_lib.name_utils import initial_name_from_intercepted, intercepted_name
from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


def test_trivial():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic_a") == ("node_a", "topic_a")


def test_slash_in_node():
    # This is not supported by remapping, but the topic-interception should work...
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic_a") == ("node/a", "topic_a")


def test_slash_in_topic():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic/a") == ("node_a", "topic/a")


def test_slash_in_both():
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic/a") == ("node/a", "topic/a")


def test_preexisting_remapping():
    """
    This test verifies interception remappings when the topic would usually already be remapped.
    In this case, the interception remapping replaces/overrides this, since remappings are not applied transitively.
    """
    node_name = "node_1"
    topic_inside_node = "some_relative/topic_name"
    topic_outside = "/the_namespace/the_real_topic"
    intercepted = intercepted_name(node_name, topic_outside)
    # TODO: Write test that the interception remapping is actually applied before the existing remapping
    remap_args = [
        "--ros-args", "--remap", f"{node_name}:{topic_inside_node}:={intercepted}",
        "--ros-args", "--remap", f"{node_name}:{topic_inside_node}:={topic_outside}"]
    print(remap_args)
    rclpy.init()
    node = rclpy.node.Node(node_name, use_global_arguments=False, cli_args=remap_args)
    topic_resolved = node.resolve_topic_name(topic_inside_node)
    assert topic_resolved == "/intercepted/node_1/sub/the_namespace/the_real_topic"


def test_remapping_transitivity():
    # This verifies that remappings are not applied transitively, i.e. that a:=b and b:=c do not imply a:=c
    node_name = "node_1"
    remap_args = ["--ros-args", "--remap", f"a:=b",
                  "--ros-args", "--remap", f"b:=c"]
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.node.Node(node_name, use_global_arguments=False, cli_args=remap_args)
    assert node.resolve_topic_name("a") == "/b"

    remap_args_reversed = ["--ros-args", "--remap", "b:=c",
                           "--ros-args", "--remap", "a:=b"]
    node_2 = rclpy.node.Node(node_name, use_global_arguments=False, cli_args=remap_args_reversed)
    assert node_2.resolve_topic_name("a") == "/b"


def test_remap_ambiguity():
    # This verifies that if multiple remap rules match, the first is applied
    node_name = "node_1"
    remap_args = ["--ros-args", "--remap", f"a:=b",
                  "--ros-args", "--remap", f"a:=c"]
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.node.Node(node_name, use_global_arguments=False, cli_args=remap_args)
    assert node.resolve_topic_name("a") == "/b"

    remap_args_reversed = ["--ros-args", "--remap", "a:=c",
                           "--ros-args", "--remap", "a:=b"]
    node_2 = rclpy.node.Node(node_name, use_global_arguments=False, cli_args=remap_args_reversed)
    assert node_2.resolve_topic_name("a") == "/c"


@pytest.mark.parametrize("node_name,topic_name", [
    ("asdfghj", "ghjkl"),
    ("lsfd/ds_jnf/öhgflk", "dsfmodsf/dskj_fn/ab"),
    ("abc", "def"),
    ("abc", "def/"),
])
def test_round_trip(node_name, topic_name):
    assert initial_name_from_intercepted(intercepted_name(node_name, topic_name)) == (node_name, topic_name)


@pytest.mark.parametrize("node_name,topic_name", [
    ("asdfghj", "/ghjkl"),
    ("lsfdds_jnföhgflk", "/dsfmodsf/dskj_fn/ab"),
    ("abc", "/def"),
    ("abc", "/def/"),
])
def test_leading_slash_topic(node_name, topic_name):
    assert (initial_name_from_intercepted(intercepted_name(node_name, topic_name)) ==
            (node_name, topic_name.removeprefix("/")))


def remap_eq(a: SetRemap, b: SetRemap) -> bool:
    c1 = LaunchContext()
    a.execute(c1)
    c2 = LaunchContext()
    b.execute(c2)
    print(c1.launch_configurations, c2.launch_configurations)
    return c1.launch_configurations == c2.launch_configurations


def remap_contains(item: SetRemap, list_of_remaps: list[SetRemap]):
    for i in list_of_remaps:
        if remap_eq(i, item):
            return True
    else:
        return False


def test_generated_remappings_1():
    remappings = generate_remappings_from_config_file("orchestrator", "unittest_remapping_launch_config.json")

    assert remap_contains(SetRemap(src="t2t:tracks_1", dst="/intercepted/t2t/sub/tracks"), remappings)
    assert remap_contains(SetRemap(src="t2t:dynamic_tracks", dst="/intercepted/t2t/sub/dynamic_tracks"), remappings)
    assert not remap_contains(SetRemap(src="t2t:fused_tracks", dst="/intercepted/t2t/sub/fused_tracks"), remappings)


def test_launchfile_fqn_nodename():
    with pytest.raises(RuntimeError, match=r"Node namespaces are not supported.*"):
        generate_remappings_from_config_file("orchestrator", "unittest_fqn_launch_config.json")
