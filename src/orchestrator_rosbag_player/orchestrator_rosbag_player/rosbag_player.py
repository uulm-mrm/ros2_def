import time
from orchestrator.orchestrator_lib.model_loader import load_launch_config, load_launch_config_schema, load_models, load_node_config_schema
from orchestrator.orchestrator_lib.name_utils import type_from_string
from orchestrator.orchestrator_lib.orchestrator import Orchestrator
import rosbag2_py
import rclpy
import rclpy.node
import rclpy.publisher
import rclpy.serialization
import rclpy.time


def create_reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri)
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(serialization_format, serialization_format)
    reader.open(storage_options, converter_options)
    return reader


def main():
    rclpy.init()
    node = rclpy.node.Node("player")  # type: ignore

    reader = create_reader("/home/gja38/sandbox_otto/rosbag2_2023_03_15-14_01_00")

    publishers: dict[str, rclpy.publisher.Publisher] = {}
    types: dict[str, type] = {}

    for topic_metadata in reader.get_all_topics_and_types():
        message_type = type_from_string(topic_metadata.type)
        types[topic_metadata.name] = message_type
        publishers[topic_metadata.name] = node.create_publisher(message_type, topic_metadata.name, 10)

    launch_config = load_launch_config(
        "orchestrator_dummy_nodes",
        "time_sync_test_launch_config.json",
        load_launch_config_schema()
    )
    node_config = load_models(launch_config, load_node_config_schema())
    orchestrator = Orchestrator(node, node_config, node.get_logger().get_child("orchestrator"))

    orchestrator.initialize_ros_communication()

    while reader.has_next():
        topic_name, serialized_data, time_stamp_ns = reader.read_next()
        deserialized_message = rclpy.serialization.deserialize_message(serialized_data, types[topic_name])

        time_stamp = rclpy.time.Time(nanoseconds=time_stamp_ns)
        

        print(f"{time_stamp}: {topic_name}: {deserialized_message}")
        publishers[topic_name].publish(serialized_data)
        time.sleep(0.2)


if __name__ == '__main__':
    main()
