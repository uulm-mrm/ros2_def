from typing import Type
from orchestrator.orchestrator_lib.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput
from std_msgs.msg import String
from orchestrator_interfaces.msg import SampleMessage

class TrackingNodeModel(NodeModel):

    def __init__(self, name: str) -> None:
        super().__init__(name,
                         [("input_radar", "detections/radar"),
                          ("input_lidar", "detections/lidar"),
                          ("input_camera", "detections/camera")],
                         [("tracks", "tracks"),
                          ("status", "status")])

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


class DetectionNodeModel(NodeModel):
    def __init__(self, name: str, input_topic_name: str, output_topic_name: str):
        super().__init__(name, [("input", input_topic_name)], [("output", output_topic_name), ("status", "status")])

    def get_possible_inputs(self) -> list[Cause]:
        return [self.internal_topic_input("input")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if input != self.internal_topic_input("input"):
            raise ValueError(f"unknown input: {input}")
        return [self.internal_topic_pub(("output"))]


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


radar_detector = DetectionNodeModel("detector_radar", "meas/radar", "detections/radar")
camera_detector = DetectionNodeModel("detector_camera", "meas/camera", "detections/camera")
lidar_detector = DetectionNodeModel("detector_lidar", "meas/lidar", "detections/lidar")

tracking = TrackingNodeModel("tracking")
gridmap = DetectionNodeModel("gridmap", "meas/radar", "occupancy_grid")

plausibility_node = PlausibilityCheckNodeModel("plausibility", "occupancy_grid", "tracks", "plausible_tracks", "gridmap_tracks")

# TODO: Sensor nodes? We dont do anything with them, do we need to model them?

# TODO: The order here determines execution order!
nodes: list[NodeModel] = [radar_detector, camera_detector, lidar_detector, tracking, gridmap, plausibility_node]

external_input_topics: list[tuple[Type, str]] = [(String, "meas/radar"), (String, "meas/camera"), (String, "meas/lidar")]
output_topics: list[tuple[Type, str]] = [(String, "plausible_tracks"), (String, "gridmap_tracks"), (SampleMessage, "trajectory")]

# Manually decide intercepted topics for now.
# Each topic has many names:
#  * canonical: /meas/camera
#  * inside nodes: camera: "output", detector_camera: "input"
#  * intercepted: "/intercepted/detector_camera/sub/meas/camera"  (could be more subscriptions)
