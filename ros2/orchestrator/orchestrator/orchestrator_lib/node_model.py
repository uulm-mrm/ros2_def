# pyright: strict

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union, Tuple, List, Optional, Any

from typing_extensions import TypeAlias


@dataclass(frozen=True)
class TopicInput:
    input_topic: str


@dataclass(frozen=True)
class TimerInput:
    period: int  # Timer period in ns


@dataclass(frozen=True)
class TimeSyncInfo:
    input_topics: Tuple[str, ...]
    slop: float
    queue_size: int


Cause: TypeAlias = Union[TopicInput, TimerInput]


@dataclass(frozen=True)
class TopicPublish:
    output_topic: str


@dataclass(frozen=True)
class StatusPublish:
    pass


@dataclass
class ServiceCall:
    service_name: str


SimpleRemapRule: TypeAlias = Tuple[str, str]
SimpleRemapRules: TypeAlias = List[SimpleRemapRule]
Effect: TypeAlias = Union[TopicPublish, StatusPublish, ServiceCall]

ServiceName: TypeAlias = str


class NodeModel(ABC):
    def __init__(self, name: str, remappings: SimpleRemapRules) -> None:
        self.name = name

        self.remappings: SimpleRemapRules = remappings
        super().__init__()

    @abstractmethod
    def state_sequence_push(self, message: Any) -> None:
        ...

    @abstractmethod
    def dump_state_sequence(self) -> None:
        ...

    def get_name(self) -> str:
        return self.name

    def internal_name_from_topic(self, topic: str) -> str:
        """Map remapped topic name to internal name"""
        for input_name, input_topic in self.remappings:
            if input_topic == topic:
                return input_name
        raise ValueError(f"Topic {topic} is not known to node")

    def topic_name_from_internal(self, internal_name: str) -> str:
        """Map internal topic name to remapped name"""
        for input_name, input_topic in self.remappings:
            if input_name == internal_name:
                return input_topic
        raise ValueError(
            f"Internal name \"{internal_name}\" of node \"{self.get_name()}\" has no (external) topic name!")

    def internal_topic_pub(self, internal_name: str) -> TopicPublish:
        """Create TopicPublish effect by internal name"""
        return TopicPublish(self.topic_name_from_internal(internal_name))

    def internal_topic_input(self, internal_name: str) -> TopicInput:
        """Create TopicInput cause by internal name"""
        return TopicInput(self.topic_name_from_internal(internal_name))

    @abstractmethod
    def get_possible_inputs(self) -> List[Cause]:
        ...

    @abstractmethod
    def effects_for_input(self, input: Cause) -> List[Effect]:
        ...

    @abstractmethod
    def input_modifies_dataprovider_state(self, input: Cause) -> bool:
        ...

    @abstractmethod
    def input_may_cause_reconfiguration(self, input: Cause) -> bool:
        ...

    @abstractmethod
    def get_provided_services(self) -> List[ServiceName]:
        ...

    @abstractmethod
    def time_sync_info(self, topic_name: str) -> Optional[TimeSyncInfo]:
        ...

    @abstractmethod
    def time_sync_infos(self) -> List[TimeSyncInfo]:
        ...
