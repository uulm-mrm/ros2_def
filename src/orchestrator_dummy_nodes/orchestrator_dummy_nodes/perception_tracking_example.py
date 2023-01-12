from pprint import pprint
import random
from typing import Tuple

from orchestrator_dummy_nodes.events import CausalitySpecification, TopicInput, TopicPublish


topic_lidar = "/lidar"
topic_camera = "/camera"
topic_radar = "/radar"

interval_lidar = 0.1
interval_camera = 0.1
interval_radar = 0.1
deviation_radar = 0.005


def create_recording():
    recording: list[Tuple[float, str]] = []

    for i in range(20):
        recording.append((i*interval_lidar, topic_lidar))
        recording.append((i*interval_camera, topic_camera))
        recording.append((i*interval_radar + random.uniform(-deviation_radar, deviation_radar), topic_radar))

    recording.sort(key=lambda t: t[0])
    pprint(recording)


recording = [(0.0, '/lidar'),
             (0.0, '/camera'),
             (0.0012251532884920945, '/radar'),
             (0.09797592796272325, '/radar'),
             (0.1, '/lidar'),
             (0.1, '/camera'),
             (0.19971792480065848, '/radar'),
             (0.2, '/lidar'),
             (0.2, '/camera'),
             (0.30000000000000004, '/lidar'),
             (0.30000000000000004, '/camera'),
             (0.30336850459090525, '/radar'),
             (0.3954340950165439, '/radar'),
             (0.4, '/lidar'),
             (0.4, '/camera'),
             (0.5, '/lidar'),
             (0.5, '/camera'),
             (0.500391392697039, '/radar'),
             (0.5987513332730916, '/radar'),
             (0.6000000000000001, '/lidar'),
             (0.6000000000000001, '/camera'),
             (0.6999029399163417, '/radar'),
             (0.7000000000000001, '/lidar'),
             (0.7000000000000001, '/camera'),
             (0.8, '/lidar'),
             (0.8, '/camera'),
             (0.8017556107818055, '/radar'),
             (0.9, '/lidar'),
             (0.9, '/camera'),
             (0.9035258329289765, '/radar'),
             (0.9979417778023193, '/radar'),
             (1.0, '/lidar'),
             (1.0, '/camera'),
             (1.099159319768284, '/radar'),
             (1.1, '/lidar'),
             (1.1, '/camera'),
             (1.2000000000000002, '/lidar'),
             (1.2000000000000002, '/camera'),
             (1.2047117775870575, '/radar'),
             (1.29517058571003, '/radar'),
             (1.3, '/lidar'),
             (1.3, '/camera'),
             (1.3999424284136346, '/radar'),
             (1.4000000000000001, '/lidar'),
             (1.4000000000000001, '/camera'),
             (1.495730574581202, '/radar'),
             (1.5, '/lidar'),
             (1.5, '/camera'),
             (1.5966886370623374, '/radar'),
             (1.6, '/lidar'),
             (1.6, '/camera'),
             (1.6979800222709165, '/radar'),
             (1.7000000000000002, '/lidar'),
             (1.7000000000000002, '/camera'),
             (1.8, '/lidar'),
             (1.8, '/camera'),
             (1.8026570752875235, '/radar'),
             (1.8990399737279517, '/radar'),
             (1.9000000000000001, '/lidar'),
             (1.9000000000000001, '/camera')]


topic_lidar_detections = "/detections/lidar"
topic_camera_detections = "/detections/camera"
topic_radar_detections = "/detections/radar"
lidar_detector: CausalitySpecification = {TopicInput(topic_lidar, "lidar_detector"): [TopicPublish(topic_lidar_detections, "lidar_detector")]}
camera_detector: CausalitySpecification = {TopicInput(topic_camera, "camera_detector"): [TopicPublish(topic_camera_detections, "camera_detector")]}
radar_detector: CausalitySpecification = {TopicInput(topic_radar, "radar_detector"): [TopicPublish(topic_radar_detections, "radar_detector")]}

topic_tracking_result = "/tracks"
tracking: CausalitySpecification = {TopicInput(topic_lidar, "tracking"): [TopicPublish(topic_tracking_result, "tracking")]}

# TODO: Tracking ordered input constraint
