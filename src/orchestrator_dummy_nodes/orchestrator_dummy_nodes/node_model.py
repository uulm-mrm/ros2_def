
from abc import ABC, abstractmethod
from dataclasses import dataclass
from launch_ros.remap_rule_type import SomeRemapRules


class Cause(ABC):
    pass


@dataclass(frozen=True)
class TopicInput(Cause):
    input_topic: str


class Effect:
    pass


@dataclass(frozen=True)
class TopicPublish(Effect):
    output_topic: str


@dataclass(frozen=True)
class StatusPublish(Effect):
    pass


class NodeModel(ABC):
    def __init__(self, name: str, input_topics: SomeRemapRules, output_topics: SomeRemapRules) -> None:
        self.name = name
        self.input_remappings: SomeRemapRules = input_topics
        self.output_remappings: SomeRemapRules = output_topics
        super().__init__()

    @abstractmethod
    def process_input(self, input) -> None:
        ...

    def get_name(self) -> str:
        return self.name

    def internal_name_from_topic(self, topic: str):
        for input_name, input_topic in self.input_remappings:
            if input_topic == topic:
                if not isinstance(input_name, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return input_name
        for output_name, output_topic in self.output_remappings:
            if output_topic == topic:
                if not isinstance(output_name, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return output_name
        raise ValueError(f"Topic {topic} is not known to node")

    def topic_name_from_internal(self, internal_name: str) -> str:
        for input_name, input_topic in self.input_remappings:
            if input_name == internal_name:
                if not isinstance(input_topic, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return input_topic
        for output_name, output_topic in self.output_remappings:
            if output_name == internal_name:
                if not isinstance(output_topic, str):
                    raise NotImplementedError("Remapping with substitutions is not implemented")
                return output_topic
        raise ValueError(f"Internal name {internal_name} has no topic name")

    def internal_topic_pub(self, internal_name: str) -> TopicPublish:
        return TopicPublish(self.topic_name_from_internal(internal_name))

    def internal_topic_input(self, internal_name: str) -> TopicInput:
        return TopicInput(self.topic_name_from_internal(internal_name))

    @abstractmethod
    def get_possible_inputs(self) -> list[Cause]:
        ...

    @abstractmethod
    def effects_for_input(self, input: Cause) -> list[Effect]:
        ...

    @abstractmethod
    def handle_event(self, event: Effect) -> None:
        ...

    @abstractmethod
    def ready_for_input(self, input) -> bool:
        ...
