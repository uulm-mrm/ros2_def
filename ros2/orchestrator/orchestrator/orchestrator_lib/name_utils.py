# pyright: strict

from __future__ import annotations

from typing_extensions import TypeAlias
import rosidl_runtime_py.utilities

TopicName: TypeAlias = str
NodeName: TypeAlias = str


def normalize_topic_name(name: str) -> str:
    if name[0] != "/":
        return "/" + name
    return name


def remove_prefix(text: str, prefix: str) -> str:
    if text.startswith(prefix):
        return text[len(prefix):]
    return text


def intercepted_name(node_name: str, topic_name: str) -> str:
    if node_name.startswith("/"):
        node_name = node_name[1:]
    if topic_name.startswith("/"):
        topic_name = topic_name[1:]
    return f"/intercepted/{node_name}/sub/{topic_name}"


def initial_name_from_intercepted(intercepted_name: str) -> tuple[str, str]:
    """
    @return (node_name, topic_name)
    """
    intercepted_name = remove_prefix(intercepted_name, "/")
    parts = intercepted_name.split("/")
    if len(parts) < 4:
        raise RuntimeError(
            f"Invalid intercepted name: {intercepted_name}: At least 4 components required.")
    if parts[0] != "intercepted":
        raise RuntimeError()
    sub_index = parts.index("sub")
    node_name = "/".join(parts[1:sub_index])
    topic_name = "/".join(parts[sub_index + 1:])
    return node_name, topic_name


def type_from_string(typestring: str) -> type:
    return rosidl_runtime_py.utilities.get_message(typestring)
