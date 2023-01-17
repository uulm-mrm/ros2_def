import importlib
from typing import Generator, Type


def remove_prefix(text, prefix):
    if text.startswith(prefix):
        return text[len(prefix):]
    return text


def intercepted_name(node_name: str, topic_name: str) -> str:
    return f"/intercepted/{node_name}/sub/{topic_name}"


def initial_name_from_intercepted(intercepted_name: str) -> tuple[str, str]:
    """
    @return (node_name, topic_name)
    """
    intercepted_name = remove_prefix(intercepted_name, "/")
    parts = intercepted_name.split("/")
    if len(parts) < 4:
        raise RuntimeError(f"Invalid intercepted name: {intercepted_name}: At least 4 components required.")
    if parts[0] != "intercepted":
        raise RuntimeError()
    sub_index = parts.index("sub")
    node_name = "/".join(parts[1:sub_index])
    topic_name = "/".join(parts[sub_index+1:])
    return node_name, topic_name


def type_from_string(typestring: str):
    parts = typestring.split("/")
    module = importlib.import_module(parts[0]+"."+parts[1])
    return getattr(module, parts[2])


def collect_intercepted_topics(topic_names_and_types) -> Generator[tuple[str, str, str, Type], None, None]:
    for intercepted_name, types in topic_names_and_types:
        if intercepted_name.startswith("/intercepted"):
            for type in types:
                type = type_from_string(type)
                node, canonical_name = initial_name_from_intercepted(intercepted_name)
                yield node, canonical_name, intercepted_name, type
