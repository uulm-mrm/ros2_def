from orchestrator_dummy_nodes.topic_remapping import initial_name_from_intercepted, intercepted_name


def test_trivial():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic_a") == ("node_a", "topic_a")


def test_slash_in_node():
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic_a") == ("node/a", "topic_a")


def test_slash_in_topic():
    assert initial_name_from_intercepted("/intercepted/node_a/sub/topic/a") == ("node_a", "topic/a")


def test_slash_in_both():
    assert initial_name_from_intercepted("/intercepted/node/a/sub/topic/a") == ("node/a", "topic/a")


def test_round_trip():
    testcases = [
        ("asdfghj", "ghjkl"),
        ("lsfd/ds_jnf/öhgflk", "dsfmodsf/dskj_fn/ab"),
        ("abc/", "/def"),
        ("/abc/", "/def/")
    ]
    for node, topic in testcases:
        assert initial_name_from_intercepted(intercepted_name(node, topic)) == (node, topic)
