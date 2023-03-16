"""Python wrapper of the rosbag2_cpp reindexer API"""
from __future__ import annotations
import rosbag2_py._reindexer
import typing
import rosbag2_py._storage

__all__ = [
    "Reindexer"
]


class Reindexer():
    def __init__(self) -> None: ...
    def reindex(self, arg0: rosbag2_py._storage.StorageOptions) -> None: ...
    pass
