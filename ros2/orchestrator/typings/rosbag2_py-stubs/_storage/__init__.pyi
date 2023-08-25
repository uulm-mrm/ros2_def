"""Python wrapper of the rosbag2 utilities API"""
from __future__ import annotations
import rosbag2_py._storage
import typing
import datetime

__all__ = [
    "BagMetadata",
    "ConverterOptions",
    "FileInformation",
    "StorageFilter",
    "StorageOptions",
    "TopicInformation",
    "TopicMetadata"
]


class BagMetadata():
    def __init__(self, version: int, bag_size: int, storage_identifier: str, relative_file_paths: typing.List[str], files: typing.List[FileInformation], duration: datetime.timedelta, starting_time: datetime.datetime, message_count: int, topics_with_message_count: typing.List[TopicInformation], compression_format: str, compression_mode: str) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def bag_size(self) -> int:
        """
        :type: int
        """
    @bag_size.setter
    def bag_size(self, arg0: int) -> None:
        pass
    @property
    def compression_format(self) -> str:
        """
        :type: str
        """
    @compression_format.setter
    def compression_format(self, arg0: str) -> None:
        pass
    @property
    def compression_mode(self) -> str:
        """
        :type: str
        """
    @compression_mode.setter
    def compression_mode(self, arg0: str) -> None:
        pass
    @property
    def duration(self) -> datetime.timedelta:
        """
        :type: datetime.timedelta
        """
    @duration.setter
    def duration(self, arg0: datetime.timedelta) -> None:
        pass
    @property
    def files(self) -> typing.List[FileInformation]:
        """
        :type: typing.List[FileInformation]
        """
    @files.setter
    def files(self, arg0: typing.List[FileInformation]) -> None:
        pass
    @property
    def message_count(self) -> int:
        """
        :type: int
        """
    @message_count.setter
    def message_count(self, arg0: int) -> None:
        pass
    @property
    def relative_file_paths(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    @relative_file_paths.setter
    def relative_file_paths(self, arg0: typing.List[str]) -> None:
        pass
    @property
    def starting_time(self) -> datetime.datetime:
        """
        :type: datetime.datetime
        """
    @starting_time.setter
    def starting_time(self, arg0: datetime.datetime) -> None:
        pass
    @property
    def storage_identifier(self) -> str:
        """
        :type: str
        """
    @storage_identifier.setter
    def storage_identifier(self, arg0: str) -> None:
        pass
    @property
    def topics_with_message_count(self) -> typing.List[TopicInformation]:
        """
        :type: typing.List[TopicInformation]
        """
    @topics_with_message_count.setter
    def topics_with_message_count(self, arg0: typing.List[TopicInformation]) -> None:
        pass
    @property
    def version(self) -> int:
        """
        :type: int
        """
    @version.setter
    def version(self, arg0: int) -> None:
        pass
    pass
class ConverterOptions():
    def __init__(self, input_serialization_format: str, output_serialization_format: str) -> None: ...
    @property
    def input_serialization_format(self) -> str:
        """
        :type: str
        """
    @input_serialization_format.setter
    def input_serialization_format(self, arg0: str) -> None:
        pass
    @property
    def output_serialization_format(self) -> str:
        """
        :type: str
        """
    @output_serialization_format.setter
    def output_serialization_format(self, arg0: str) -> None:
        pass
    pass
class FileInformation():
    def __init__(self, path: str, starting_time: datetime.datetime, duration: datetime.timedelta, message_count: int) -> None: ...
    @property
    def duration(self) -> datetime.timedelta:
        """
        :type: datetime.timedelta
        """
    @duration.setter
    def duration(self, arg0: datetime.timedelta) -> None:
        pass
    @property
    def message_count(self) -> int:
        """
        :type: int
        """
    @message_count.setter
    def message_count(self, arg0: int) -> None:
        pass
    @property
    def path(self) -> str:
        """
        :type: str
        """
    @path.setter
    def path(self, arg0: str) -> None:
        pass
    @property
    def starting_time(self) -> datetime.datetime:
        """
        :type: datetime.datetime
        """
    @starting_time.setter
    def starting_time(self, arg0: datetime.datetime) -> None:
        pass
    pass
class StorageFilter():
    def __init__(self, topics: typing.List[str]) -> None: ...
    @property
    def topics(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    @topics.setter
    def topics(self, arg0: typing.List[str]) -> None:
        pass
    pass
class StorageOptions():
    def __init__(self, uri: str, storage_id: str = '', max_bagfile_size: int = 0, max_bagfile_duration: int = 0, max_cache_size: int = 0, storage_preset_profile: str = '', storage_config_uri: str = '', snapshot_mode: bool = False) -> None: ...
    @property
    def max_bagfile_duration(self) -> int:
        """
        :type: int
        """
    @max_bagfile_duration.setter
    def max_bagfile_duration(self, arg0: int) -> None:
        pass
    @property
    def max_bagfile_size(self) -> int:
        """
        :type: int
        """
    @max_bagfile_size.setter
    def max_bagfile_size(self, arg0: int) -> None:
        pass
    @property
    def max_cache_size(self) -> int:
        """
        :type: int
        """
    @max_cache_size.setter
    def max_cache_size(self, arg0: int) -> None:
        pass
    @property
    def snapshot_mode(self) -> bool:
        """
        :type: bool
        """
    @snapshot_mode.setter
    def snapshot_mode(self, arg0: bool) -> None:
        pass
    @property
    def storage_config_uri(self) -> str:
        """
        :type: str
        """
    @storage_config_uri.setter
    def storage_config_uri(self, arg0: str) -> None:
        pass
    @property
    def storage_id(self) -> str:
        """
        :type: str
        """
    @storage_id.setter
    def storage_id(self, arg0: str) -> None:
        pass
    @property
    def storage_preset_profile(self) -> str:
        """
        :type: str
        """
    @storage_preset_profile.setter
    def storage_preset_profile(self, arg0: str) -> None:
        pass
    @property
    def uri(self) -> str:
        """
        :type: str
        """
    @uri.setter
    def uri(self, arg0: str) -> None:
        pass
    pass
class TopicInformation():
    def __init__(self, topic_metadata: TopicMetadata, message_count: int) -> None: ...
    @property
    def message_count(self) -> int:
        """
        :type: int
        """
    @message_count.setter
    def message_count(self, arg0: int) -> None:
        pass
    @property
    def topic_metadata(self) -> TopicMetadata:
        """
        :type: TopicMetadata
        """
    @topic_metadata.setter
    def topic_metadata(self, arg0: TopicMetadata) -> None:
        pass
    pass
class TopicMetadata():
    def __init__(self, name: str, type: str, serialization_format: str, offered_qos_profiles: str = '') -> None: ...
    def equals(self, arg0: TopicMetadata) -> bool: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg0: str) -> None:
        pass
    @property
    def offered_qos_profiles(self) -> str:
        """
        :type: str
        """
    @offered_qos_profiles.setter
    def offered_qos_profiles(self, arg0: str) -> None:
        pass
    @property
    def serialization_format(self) -> str:
        """
        :type: str
        """
    @serialization_format.setter
    def serialization_format(self, arg0: str) -> None:
        pass
    @property
    def type(self) -> str:
        """
        :type: str
        """
    @type.setter
    def type(self, arg0: str) -> None:
        pass
    pass
