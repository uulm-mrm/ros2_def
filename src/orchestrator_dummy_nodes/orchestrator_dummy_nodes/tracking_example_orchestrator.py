# Release Input if any available
# Lookup expected actions
#  -> causes new inputs, which are not released yet!

# If no available: New input


from abc import ABC, abstractmethod
from dataclasses import dataclass


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
    def __init__(self, name: str) -> None:
        self.name = name
        super().__init__()

    @abstractmethod
    def set_busy(self) -> None:
        ...

    def get_name(self) -> str:
        return self.name

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


class TrackingNodeModel(NodeModel):

    def __init__(self, name: str) -> None:
        self.name: str = name
        self.busy: bool = False
        self.state_last: str = "input_lidar"

    def set_busy(self):
        self.busy = True

    def get_name(self) -> str:
        return self.name

    def get_possible_inputs(self) -> list[Cause]:
        return [TopicInput("input_lidar"),
                TopicInput("input_camera"),
                TopicInput("input_radar")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if not isinstance(input, TopicInput):
            raise ValueError()

        return {
            TopicInput("input_lidar"): [TopicPublish("tracks")],
            TopicInput("input_camera"): [StatusPublish()],
            TopicInput("input_radar"): [StatusPublish()]
        }[input]

    def handle_event(self, event: Effect):
        if isinstance(event, StatusPublish):
            if self.state_last == "input_lidar":
                self.state_last = "input_camera"
                self.busy = False
            if self.state_last == "input_radar":
                # If last processed was radar, next should be lidar.
                # Lidar does however cause TopicPublish, not StatusPublish!
                raise RuntimeError(f"Expected TopicPublish for tracks, but got status instead! state_last: {self.state_last}, event: {event}")
            if self.state_last == "input_camera":
                self.state_last = "input_radar"
                self.busy = False
        if isinstance(event, TopicPublish):
            if event == TopicPublish("track") and self.state_last == "input_radar":
                self.state_last = "input_lidar"
                self.busy = False
            else:
                raise RuntimeError(f"Expected TopicPublish for tracks, but got something else: {event}")

    def ready_for_input(self, input) -> bool:
        if self.busy:
            return False
        # Assert camera -> radar -> lidar order
        if input == TopicInput("input_lidar"):
            return self.state_last == "input_radar"
        if input == TopicInput("input_radar"):
            return self.state_last == "input_camera"
        if input == TopicInput("input_camera"):
            return self.state_last == "input_lidar"
        return False


class DetectionNodeModel(NodeModel):
    def __init__(self, name: str):
        self.name = name
        self.busy: bool = False

    def set_busy(self) -> None:
        self.busy = True

    def get_possible_inputs(self) -> list[Cause]:
        return [TopicInput("input")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if input != TopicInput("input"):
            raise ValueError(f"unknown input: {input}")
        return [TopicPublish("output")]

    def handle_event(self, event: Effect) -> None:
        if event != TopicPublish("output"):
            raise ValueError()
        self.busy = False

    def ready_for_input(self, input) -> bool:
        if input != TopicInput("input"):
            return False
        return not self.busy


d = DetectionNodeModel("abc")

# TODO: Adapt serializing_interceptor to accept these node models
