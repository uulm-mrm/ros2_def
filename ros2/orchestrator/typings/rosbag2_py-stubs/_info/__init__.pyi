"""Python wrapper of the rosbag2_cpp info API"""
from __future__ import annotations
import rosbag2_py._info
import typing
import rosbag2_py._storage

__all__ = [
    "Info"
]


class Info():
    def __init__(self) -> None: ...
    def read_metadata(self, arg0: str, arg1: str) -> rosbag2_py._storage.BagMetadata: ...
    pass
