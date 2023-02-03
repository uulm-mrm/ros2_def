from typing import Type
from orchestrator_dummy_nodes.node_model import Cause, Effect, NodeModel, StatusPublish, TopicInput, TopicPublish
from std_msgs.msg import String


class CameraDetectionNodeModel(NodeModel):
    def __init__(self, name: str, image_input_topic_name: str, camera_info_topic_name: str, output_topic_name: str):
        super().__init__(name,
                         [("input_image", image_input_topic_name), ("input_camerainfo", camera_info_topic_name)],
                         [("output", output_topic_name), ("status", "status")])

    def process_input(self, input) -> None:
        raise NotADirectoryError()

    def get_possible_inputs(self) -> list[Cause]:
        return [self.internal_topic_input("input_camerainfo"),
                self.internal_topic_input("input_image")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        # This does not (yet) model integrated approximate-time sync, where the input order does not matter, and only the second input
        # causes the callback and subsequent publish
        if input == self.internal_topic_input("input_camerainfo"):
            return [StatusPublish()]
        elif input == self.internal_topic_input("input_image"):
            return [self.internal_topic_pub(("output"))]
        else:
            raise ValueError(f"unknown input: {input}")

    def handle_event(self, event: Effect) -> None:
        raise NotImplementedError()

    def ready_for_input(self, input) -> bool:
        raise NotImplementedError()


class TriggeredCameraNodeModel(NodeModel):
    def __init__(self, name: str, input_topic_name: str, image_topic_name: str, camera_info_topic_name: str):
        super().__init__(name, [("trigger", input_topic_name)],
                         [("output_image", image_topic_name), ("output_camerainfo", camera_info_topic_name)])

    def process_input(self, input) -> None:
        raise NotImplementedError()

    def get_possible_inputs(self) -> list[Cause]:
        return [self.internal_topic_input("trigger")]

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if input != self.internal_topic_input("trigger"):
            raise ValueError(f"unknown input: {input}")
        return [self.internal_topic_pub("output_image"), self.internal_topic_pub("output_camerainfo")]

    def handle_event(self, event: Effect) -> None:
        raise NotImplementedError()

    def ready_for_input(self, input) -> bool:
        raise NotImplementedError()


triggered_camera = TriggeredCameraNodeModel("camera", "trigger", "camera/image",  "camera/camerainfo")
camera_detector = CameraDetectionNodeModel("detector_camera", "camera/image", "camera/camerainfo", "detections/camera")

nodes: list[NodeModel] = [triggered_camera, camera_detector]

external_input_topics: list[tuple[Type, str]] = [(String, "trigger")]
