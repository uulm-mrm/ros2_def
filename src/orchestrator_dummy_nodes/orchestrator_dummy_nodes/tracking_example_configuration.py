from typing import Type
from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish
from std_msgs.msg import String


class TrackingNodeModel(NodeModel):

    def __init__(self, name: str) -> None:
        super().__init__(name,
                         [("input_radar", "detections/radar"),
                          ("input_lidar", "detections/lidar"),
                          ("input_camera", "detections/camera")],
                         [("tracks", "tracks"),
                          ("status", "status")])
        self.busy: bool = False
        self.state_last: str = "input_lidar"

    def process_input(self, input):
        self.busy = True

    def get_name(self) -> str:
        return self.name

    def get_possible_inputs(self) -> list[Cause]:
        return [self.internal_topic_input("input_lidar"),
                self.internal_topic_input("input_camera"),
                self.internal_topic_input("input_radar")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if not isinstance(input, TopicInput):
            raise ValueError()

        return {
            self.internal_topic_input("input_lidar"): [self.internal_topic_pub("tracks")],
            self.internal_topic_input("input_camera"): [StatusPublish()],
            self.internal_topic_input("input_radar"): [StatusPublish()]
        }[input]

    def handle_event(self, event: Effect):
        if isinstance(event, StatusPublish):
            if self.state_last == "input_lidar":
                self.state_last = "input_camera"
                self.busy = False
            elif self.state_last == "input_radar":
                # If last processed was radar, next should be lidar.
                # Lidar does however cause TopicPublish, not StatusPublish!
                raise RuntimeError(f"Expected TopicPublish for tracks, but got status instead! state_last: {self.state_last}, event: {event}")
            elif self.state_last == "input_camera":
                self.state_last = "input_radar"
                self.busy = False
        elif isinstance(event, TopicPublish):
            if event == self.internal_topic_pub("tracks") and self.state_last == "input_radar":
                self.state_last = "input_lidar"
                self.busy = False
            else:
                raise RuntimeError(f"Expected TopicPublish for tracks, but got something else: {event}")

    def ready_for_input(self, input) -> bool:
        if self.busy:
            return False
        # Assert camera -> radar -> lidar order
        if input == self.internal_topic_input("input_lidar"):
            return self.state_last == "input_radar"
        if input == self.internal_topic_input("input_radar"):
            return self.state_last == "input_camera"
        if input == self.internal_topic_input("input_camera"):
            return self.state_last == "input_lidar"
        return False


class DetectionNodeModel(NodeModel):
    def __init__(self, name: str, input_topic_name: str, output_topic_name: str):
        super().__init__(name, [("input", input_topic_name)], [("output", output_topic_name), ("status", "status")])
        self.busy: bool = False

    def process_input(self, input) -> None:
        self.busy = True

    def get_possible_inputs(self) -> list[Cause]:
        return [self.internal_topic_input("input")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if input != self.internal_topic_input("input"):
            raise ValueError(f"unknown input: {input}")
        return [self.internal_topic_pub(("output"))]

    def handle_event(self, event: Effect) -> None:
        if event != self.internal_topic_pub(("output")):
            raise ValueError()
        self.busy = False

    def ready_for_input(self, input) -> bool:
        if input != self.internal_topic_input("input"):
            return False
        return not self.busy


class PlausibilityCheckNodeModel(NodeModel):
    """
    Process 1: Gridmap -> Tracks
    Process 2: Tracks -> Tracks
    Both processes are independent.
    """

    def __init__(self, name: str, gridmap_input: str, tracks_input: str, tracks_output: str, tracks_output_gridmap: str) -> None:
        super().__init__(name,
                         [("gridmap", gridmap_input), ("tracks_in", tracks_input)],
                         [("tracks_out", tracks_output), ("tracks_out_gridmap", tracks_output_gridmap)])
        self.gridmap_processing_busy = False
        self.tracks_processing_busy = False

    def process_input(self, input) -> None:
        if input == self.internal_topic_input("gridmap"):
            assert not self.gridmap_processing_busy
            self.gridmap_processing_busy = True
        elif input == self.internal_topic_input("tracks_in"):
            assert not self.tracks_processing_busy
            self.tracks_processing_busy = True
        else:
            raise RuntimeError()

    def get_possible_inputs(self) -> list[Cause]:
        return [TopicInput(self.topic_name_from_internal("gridmap")),
                TopicInput(self.topic_name_from_internal("tracks_in"))]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if input == self.internal_topic_input("gridmap"):
            return [self.internal_topic_pub("tracks_out_gridmap")]
        elif input == self.internal_topic_input("tracks_in"):
            return [self.internal_topic_pub("tracks_out")]
        else:
            raise RuntimeError()

    def handle_event(self, event: Effect) -> None:
        if event == self.internal_topic_pub("tracks_out_gridmap"):
            assert self.gridmap_processing_busy
            self.gridmap_processing_busy = False
        elif event == self.internal_topic_pub("tracks_out"):
            assert self.tracks_processing_busy
            self.tracks_processing_busy = False
        else:
            raise RuntimeError()

    def ready_for_input(self, input) -> bool:
        if input == self.internal_topic_input("gridmap"):
            return not self.gridmap_processing_busy
        elif input == self.internal_topic_input("tracks_in"):
            return not self.tracks_processing_busy
        else:
            raise RuntimeError()


radar_detector = DetectionNodeModel("detector_radar", "meas/radar", "detections/radar")
camera_detector = DetectionNodeModel("detector_camera", "meas/camera", "detections/camera")
lidar_detector = DetectionNodeModel("detector_lidar", "meas/lidar", "detections/lidar")

tracking = TrackingNodeModel("tracking")
gridmap = DetectionNodeModel("gridmap", "meas/radar", "occupancy_grid")

plausibility_node = PlausibilityCheckNodeModel("plausibility", "occupancy_grid", "tracks", "plausible_tracks", "gridmap_tracks")

# TODO: Sensor nodes? We dont do anything with them, do we need to model them?

nodes: list[NodeModel] = [radar_detector, camera_detector, lidar_detector, tracking, gridmap, plausibility_node]

external_input_topics: list[tuple[Type, str]] = [(String, "meas/radar"), (String, "meas/camera"), (String, "meas/lidar")]

# Manually decide intercepted topics for now.
# Each topic has many names:
#  * canonical: /meas/camera
#  * inside nodes: camera: "output", detector_camera: "input"
#  * intercepted: "/intercepted/detector_camera/sub/meas/camera"  (could be more subscriptions)
