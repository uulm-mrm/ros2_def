import pytest
from launch import LaunchContext
from launch_ros.actions import SetRemap

from orchestrator.orchestrator_lib.name_utils import initial_name_from_intercepted, intercepted_name
from orchestrator.orchestrator_lib.remapping_generation import generate_remappings_from_config_file


def test_trivial():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic_a") == ("node_a", "topic_a")


def test_slash_in_node():
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic_a") == ("node/a", "topic_a")


def test_slash_in_topic():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic/a") == ("node_a", "topic/a")


def test_slash_in_both():
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic/a") == ("node/a", "topic/a")


@pytest.mark.parametrize("node_name,topic_name", [
    ("asdfghj", "ghjkl"),
    ("lsfd/ds_jnf/öhgflk", "dsfmodsf/dskj_fn/ab"),
    ("abc", "def"),
    ("abc", "def/"),
    ("abc/", "def"),
    ("abc/", "def/"),
    ("/abc", "def"),
    ("/abc", "def/"),
    ("/abc/", "def"),
    ("/abc/", "def/"),
])
def test_round_trip(node_name, topic_name):
    assert initial_name_from_intercepted(intercepted_name(node_name, topic_name)) == (node_name, topic_name)


@pytest.mark.parametrize("node_name,topic_name", [
    ("asdfghj", "/ghjkl"),
    ("lsfd/ds_jnf/öhgflk", "/dsfmodsf/dskj_fn/ab"),
    ("abc", "/def"),
    ("abc", "/def/"),
    ("abc/", "/def"),
    ("abc/", "/def/"),
    ("/abc", "/def"),
    ("/abc", "/def/"),
    ("/abc/", "/def"),
    ("/abc/", "/def/"),
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
