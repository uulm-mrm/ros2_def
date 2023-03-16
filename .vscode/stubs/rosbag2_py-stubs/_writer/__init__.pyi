"""Python wrapper of the rosbag2_cpp writer API"""
from __future__ import annotations
import rosbag2_py._writer
import typing
import rosbag2_py._storage

__all__ = [
    "SequentialCompressionWriter",
    "SequentialWriter",
    "get_registered_compressors",
    "get_registered_serializers",
    "get_registered_writers"
]


class SequentialCompressionWriter():
    def __init__(self) -> None: ...
    def create_topic(self, arg0: rosbag2_py._storage.TopicMetadata) -> None: ...
    def open(self, arg0: rosbag2_py._storage.StorageOptions, arg1: rosbag2_py._storage.ConverterOptions) -> None: ...
    def remove_topic(self, arg0: rosbag2_py._storage.TopicMetadata) -> None: ...
    def write(self, arg0: str, arg1: str, arg2: int) -> None: ...
    pass
class SequentialWriter():
    def __init__(self) -> None: ...
    def create_topic(self, arg0: rosbag2_py._storage.TopicMetadata) -> None: ...
    def open(self, arg0: rosbag2_py._storage.StorageOptions, arg1: rosbag2_py._storage.ConverterOptions) -> None: ...
    def remove_topic(self, arg0: rosbag2_py._storage.TopicMetadata) -> None: ...
    def write(self, arg0: str, arg1: str, arg2: int) -> None: ...
    pass
def get_registered_compressors() -> typing.Set[str]:
    """
    Returns list of compression plugins available for rosbag2 recording
    """
def get_registered_serializers() -> typing.Set[str]:
    """
    Returns list of serialization format plugins available for rosbag2 recording
    """
def get_registered_writers() -> typing.Set[str]:
    """
    Returns list of discovered plugins that support rosbag2 recording
    """
