from __future__ import annotations
import rosbag2_py
import typing
from rosbag2_py._storage import BagMetadata
from rosbag2_py._storage import ConverterOptions
from rosbag2_py._info import Info
from rosbag2_py._transport import PlayOptions
from rosbag2_py._transport import Player
from rosbag2_py._transport import RecordOptions
from rosbag2_py._transport import Recorder
from rosbag2_py._reindexer import Reindexer
from rosbag2_py._reader import SequentialCompressionReader
from rosbag2_py._writer import SequentialCompressionWriter
from rosbag2_py._reader import SequentialReader
from rosbag2_py._writer import SequentialWriter
from rosbag2_py._storage import StorageFilter
from rosbag2_py._storage import StorageOptions
from rosbag2_py._storage import TopicInformation
from rosbag2_py._storage import TopicMetadata

__all__ = [
    "BagMetadata",
    "ConverterOptions",
    "Info",
    "PlayOptions",
    "Player",
    "RecordOptions",
    "Recorder",
    "Reindexer",
    "SequentialCompressionReader",
    "SequentialCompressionWriter",
    "SequentialReader",
    "SequentialWriter",
    "StorageFilter",
    "StorageOptions",
    "TopicInformation",
    "TopicMetadata",
    "bag_rewrite",
    "get_registered_compressors",
    "get_registered_readers",
    "get_registered_serializers",
    "get_registered_writers"
]


def bag_rewrite(arg0: typing.List[_storage.StorageOptions], arg1: str) -> None:
    """
    Given one or more input bags, output one or more bags with new settings.
    """
def get_registered_compressors() -> typing.Set[str]:
    """
    Returns list of compression plugins available for rosbag2 recording
    """
def get_registered_readers() -> typing.Set[str]:
    """
    Returns list of discovered plugins that support rosbag2 playback.
    """
def get_registered_serializers() -> typing.Set[str]:
    """
    Returns list of serialization format plugins available for rosbag2 recording
    """
def get_registered_writers() -> typing.Set[str]:
    """
    Returns list of discovered plugins that support rosbag2 recording
    """
__all__ = ['bag_rewrite', 'ConverterOptions', 'get_registered_readers', 'get_registered_writers', 'get_registered_compressors', 'get_registered_serializers', 'Reindexer', 'SequentialCompressionReader', 'SequentialCompressionWriter', 'SequentialReader', 'SequentialWriter', 'StorageFilter', 'StorageOptions', 'TopicMetadata', 'TopicInformation', 'BagMetadata', 'Info', 'Player', 'PlayOptions', 'Recorder', 'RecordOptions']
