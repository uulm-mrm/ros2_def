from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TypeAlias


@dataclass(frozen=True)
class TopicInput():
    input_topic: str


@dataclass(frozen=True)
class TimerInput():
    period: int  # Timer period in ns


@dataclass(frozen=True)
class TimeSyncInfo():
    input_topics: tuple[str, ...]
    slop: float
    queue_size: int


Cause: TypeAlias = TopicInput | TimerInput


@dataclass(frozen=True)
class TopicPublish():
    output_topic: str


@dataclass(frozen=True)
class StatusPublish():
    pass


@dataclass
class ServiceCall():
    service_name: str


SimpleRemapRule: TypeAlias = tuple[str, str]
SimpleRemapRules: TypeAlias = list[SimpleRemapRule]
Effect: TypeAlias = TopicPublish | StatusPublish | ServiceCall

ServiceName: TypeAlias = str


class NodeModel(ABC):
    def __init__(self, name: str, remappings: SimpleRemapRules) -> None:
        self.name = name

        self.remappings: SimpleRemapRules = remappings
        super().__init__()

    def get_name(self) -> str:
        return self.name

    def internal_name_from_topic(self, topic: str):
        """Map remapped topic name to internal name"""
        for input_name, input_topic in self.remappings:
            if input_topic == topic:
                if not isinstance(input_name, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return input_name
        raise ValueError(f"Topic {topic} is not known to node")

    def topic_name_from_internal(self, internal_name: str) -> str:
        """Map internal topic name to remapped name"""
        for input_name, input_topic in self.remappings:
            if input_name == internal_name:
                if not isinstance(input_topic, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return input_topic
        raise ValueError(f"Internal name \"{internal_name}\" of node \"{self.get_name()}\" has no (external) topic name!")

    def internal_topic_pub(self, internal_name: str) -> TopicPublish:
        """Create TopicPublish effect by internal name"""
        return TopicPublish(self.topic_name_from_internal(internal_name))

    def internal_topic_input(self, internal_name: str) -> TopicInput:
        """Create TopicInput cause by internal name"""
        return TopicInput(self.topic_name_from_internal(internal_name))

    @abstractmethod
    def get_possible_inputs(self) -> list[Cause]:
        ...

    @abstractmethod
    def effects_for_input(self, input: Cause) -> list[Effect]:
        ...

    @abstractmethod
    def get_provided_services(self) -> list[ServiceName]:
        ...

    @abstractmethod
    def time_sync_info(self, topic_name: str) -> None | TimeSyncInfo:
        ...

    @abstractmethod
    def time_sync_infos(self) -> list[TimeSyncInfo]:
        ...
