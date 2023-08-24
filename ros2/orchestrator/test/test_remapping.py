from orchestrator.orchestrator_lib.name_utils import initial_name_from_intercepted, intercepted_name
import pytest


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
