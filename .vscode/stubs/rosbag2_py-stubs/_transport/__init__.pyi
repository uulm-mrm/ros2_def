"""Python wrapper of the rosbag2_transport API"""
from __future__ import annotations
import rosbag2_py._transport
import typing
import rosbag2_py._storage

__all__ = [
    "PlayOptions",
    "Player",
    "RecordOptions",
    "Recorder",
    "bag_rewrite"
]


class PlayOptions():
    def __init__(self) -> None: ...
    @property
    def clock_publish_frequency(self) -> float:
        """
        :type: float
        """
    @clock_publish_frequency.setter
    def clock_publish_frequency(self, arg0: float) -> None:
        pass
    @property
    def delay(self) -> float:
        """
        :type: float
        """
    @delay.setter
    def delay(self, arg1: float) -> None:
        pass
    @property
    def disable_keyboard_controls(self) -> bool:
        """
        :type: bool
        """
    @disable_keyboard_controls.setter
    def disable_keyboard_controls(self, arg0: bool) -> None:
        pass
    @property
    def disable_loan_message(self) -> bool:
        """
        :type: bool
        """
    @disable_loan_message.setter
    def disable_loan_message(self, arg0: bool) -> None:
        pass
    @property
    def loop(self) -> bool:
        """
        :type: bool
        """
    @loop.setter
    def loop(self, arg0: bool) -> None:
        pass
    @property
    def node_prefix(self) -> str:
        """
        :type: str
        """
    @node_prefix.setter
    def node_prefix(self, arg0: str) -> None:
        pass
    @property
    def rate(self) -> float:
        """
        :type: float
        """
    @rate.setter
    def rate(self, arg0: float) -> None:
        pass
    @property
    def read_ahead_queue_size(self) -> int:
        """
        :type: int
        """
    @read_ahead_queue_size.setter
    def read_ahead_queue_size(self, arg0: int) -> None:
        pass
    @property
    def start_offset(self) -> float:
        """
        :type: float
        """
    @start_offset.setter
    def start_offset(self, arg1: float) -> None:
        pass
    @property
    def start_paused(self) -> bool:
        """
        :type: bool
        """
    @start_paused.setter
    def start_paused(self, arg0: bool) -> None:
        pass
    @property
    def topic_qos_profile_overrides(self) -> dict:
        """
        :type: dict
        """
    @topic_qos_profile_overrides.setter
    def topic_qos_profile_overrides(self, arg1: dict) -> None:
        pass
    @property
    def topic_remapping_options(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    @topic_remapping_options.setter
    def topic_remapping_options(self, arg0: typing.List[str]) -> None:
        pass
    @property
    def topics_to_filter(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    @topics_to_filter.setter
    def topics_to_filter(self, arg0: typing.List[str]) -> None:
        pass
    @property
    def wait_acked_timeout(self) -> int:
        """
        :type: int
        """
    @wait_acked_timeout.setter
    def wait_acked_timeout(self, arg0: int) -> None:
        pass
    pass
class Player():
    def __init__(self) -> None: ...
    def play(self, arg0: rosbag2_py._storage.StorageOptions, arg1: PlayOptions) -> None: ...
    pass
class RecordOptions():
    def __init__(self) -> None: ...
    @property
    def all(self) -> bool:
        """
        :type: bool
        """
    @all.setter
    def all(self, arg0: bool) -> None:
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
    def compression_queue_size(self) -> int:
        """
        :type: int
        """
    @compression_queue_size.setter
    def compression_queue_size(self, arg0: int) -> None:
        pass
    @property
    def compression_threads(self) -> int:
        """
        :type: int
        """
    @compression_threads.setter
    def compression_threads(self, arg0: int) -> None:
        pass
    @property
    def exclude(self) -> str:
        """
        :type: str
        """
    @exclude.setter
    def exclude(self, arg0: str) -> None:
        pass
    @property
    def ignore_leaf_topics(self) -> bool:
        """
        :type: bool
        """
    @ignore_leaf_topics.setter
    def ignore_leaf_topics(self, arg0: bool) -> None:
        pass
    @property
    def include_hidden_topics(self) -> bool:
        """
        :type: bool
        """
    @include_hidden_topics.setter
    def include_hidden_topics(self, arg0: bool) -> None:
        pass
    @property
    def include_unpublished_topics(self) -> bool:
        """
        :type: bool
        """
    @include_unpublished_topics.setter
    def include_unpublished_topics(self, arg0: bool) -> None:
        pass
    @property
    def is_discovery_disabled(self) -> bool:
        """
        :type: bool
        """
    @is_discovery_disabled.setter
    def is_discovery_disabled(self, arg0: bool) -> None:
        pass
    @property
    def node_prefix(self) -> str:
        """
        :type: str
        """
    @node_prefix.setter
    def node_prefix(self, arg0: str) -> None:
        pass
    @property
    def regex(self) -> str:
        """
        :type: str
        """
    @regex.setter
    def regex(self, arg0: str) -> None:
        pass
    @property
    def rmw_serialization_format(self) -> str:
        """
        :type: str
        """
    @rmw_serialization_format.setter
    def rmw_serialization_format(self, arg0: str) -> None:
        pass
    @property
    def start_paused(self) -> bool:
        """
        :type: bool
        """
    @start_paused.setter
    def start_paused(self, arg0: bool) -> None:
        pass
    @property
    def topic_polling_interval(self) -> datetime.timedelta:
        """
        :type: datetime.timedelta
        """
    @topic_polling_interval.setter
    def topic_polling_interval(self, arg0: datetime.timedelta) -> None:
        pass
    @property
    def topic_qos_profile_overrides(self) -> dict:
        """
        :type: dict
        """
    @topic_qos_profile_overrides.setter
    def topic_qos_profile_overrides(self, arg1: dict) -> None:
        pass
    @property
    def topics(self) -> typing.List[str]:
        """
        :type: typing.List[str]
        """
    @topics.setter
    def topics(self, arg0: typing.List[str]) -> None:
        pass
    @property
    def use_sim_time(self) -> bool:
        """
        :type: bool
        """
    @use_sim_time.setter
    def use_sim_time(self, arg0: bool) -> None:
        pass
    pass
class Recorder():
    def __init__(self) -> None: ...
    def cancel(self) -> None: ...
    def record(self, arg0: rosbag2_py._storage.StorageOptions, arg1: RecordOptions) -> None: ...
    pass
def bag_rewrite(arg0: typing.List[rosbag2_py._storage.StorageOptions], arg1: str) -> None:
    """
    Given one or more input bags, output one or more bags with new settings.
    """
