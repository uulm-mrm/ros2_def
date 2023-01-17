
from abc import ABC, abstractmethod
from dataclasses import dataclass
from launch_ros.remap_rule_type import SomeRemapRules


class Cause:
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

