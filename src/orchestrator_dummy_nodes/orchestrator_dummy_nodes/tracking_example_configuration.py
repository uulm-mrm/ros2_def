from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish


class TrackingNodeModel(NodeModel):

    def __init__(self, name: str) -> None:
        super().__init__(name,
                         [("input_radar", "/detections/radar"),
                          ("input_lidar", "/detections/lidar"),
                          ("input_camera", "/detections/camera")],
                         [("tracks", "tracks"),
                          ("status", "status")])
        self.busy: bool = False
        self.state_last: str = "input_lidar"

    def process_input(self, input):
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
    def __init__(self, name: str, input_topic_name: str, output_topic_name: str):
        super().__init__(name, [("input", input_topic_name)], [("output", output_topic_name), ("status", "status")])
        self.busy: bool = False

    def process_input(self, input) -> None:
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


radar_detector = DetectionNodeModel("detector_radar", "/meas/radar", "/detections/radar")
camera_detector = DetectionNodeModel("detector_camera", "/meas/camera", "/detections/camera")
lidar_detector = DetectionNodeModel("detector_lidar", "/meas/lidar", "/detections/lidar")

tracking = TrackingNodeModel("tracking")
gridmap = DetectionNodeModel("gridmap", "/meas/radar", "occupancy_grid")

# TODO: Sensor nodes? We dont do anything with them, do we need to model them?

nodes: list[NodeModel] = [radar_detector, camera_detector, lidar_detector, tracking, gridmap]

# Manually decide intercepted topics for now.
# Each topic has many names:
#  * canonical: /meas/camera
#  * inside nodes: camera: "output", detector_camera: "input"
#  * intercepted: "/intercepted/detector_camera/sub/meas/camera"  (could be more subscriptions)
