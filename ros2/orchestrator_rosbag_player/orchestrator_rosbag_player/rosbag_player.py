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
from rosgraph_msgs.msg import Clock


def create_reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri)
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(serialization_format, serialization_format)
    reader.open(storage_options, converter_options)
    return reader


def spin_until(node: rclpy.node.Node, time: rclpy.time.Time):
    f = rclpy.Future()
    while node.get_clock().now() < time:
        rclpy.spin_until_future_complete(node, f, timeout_sec=0.01)


def main():
    rclpy.init()
    node = rclpy.node.Node("player")  # type: ignore

    node.declare_parameter("bag_uri", "/home/gja38/sandbox_otto/rosbag2_2023_03_15-14_01_00")
    node.declare_parameter("rate", 1.0)
    node.declare_parameter("no_wait", False)
    node.declare_parameter("launch_config_package", "orchestrator_dummy_nodes")
    node.declare_parameter("launch_config_file", "time_sync_test_launch_config.json")

    reader = create_reader(node.get_parameter("bag_uri").get_parameter_value().string_value)

    publishers: dict[str, rclpy.publisher.Publisher] = {}
    types: dict[str, type] = {}

    for topic_metadata in reader.get_all_topics_and_types():
        message_type = type_from_string(topic_metadata.type)
        types[topic_metadata.name] = message_type
        publishers[topic_metadata.name] = node.create_publisher(message_type, topic_metadata.name, 10)

    clock_publisher = node.create_publisher(Clock, "clock", 10)

    launch_config = load_launch_config(
        node.get_parameter("launch_config_package").get_parameter_value().string_value,
        node.get_parameter("launch_config_file").get_parameter_value().string_value,
        load_launch_config_schema()
    )
    node_config = load_models(launch_config, load_node_config_schema())
    orchestrator = Orchestrator(node, node_config, node.get_logger().get_child("orchestrator"))

    orchestrator.initialize_ros_communication()

    playback_start_time = node.get_clock().now()
    bag_start_time = None

    time_factor = 1.0/node.get_parameter("rate").get_parameter_value().double_value
    no_wait = node.get_parameter("no_wait").get_parameter_value().bool_value

    while reader.has_next():
        topic_name, serialized_data, time_stamp_ns = reader.read_next()
        time_stamp = rclpy.time.Time(nanoseconds=time_stamp_ns)

        if bag_start_time is None:
            bag_start_time = time_stamp

        if not no_wait:
            bag_progress: rclpy.time.Duration = time_stamp-bag_start_time
            publish_time: rclpy.time.Time = playback_start_time + rclpy.time.Duration(nanoseconds=time_factor * bag_progress.nanoseconds)
            spin_until(node, publish_time)

        #_deserialized_message = rclpy.serialization.deserialize_message(serialized_data, types[topic_name])

        clock_msg = Clock()
        clock_msg.clock = time_stamp.to_msg()

        node.get_logger().info(f"Proposing time publish for {time_stamp}")
        f = orchestrator.wait_until_time_publish_allowed(time_stamp)
        rclpy.spin_until_future_complete(node, f)
        node.get_logger().info(f"Publishing time {time_stamp}")
        clock_publisher.publish(clock_msg)

        node.get_logger().info(f"Proposing message publish on {topic_name}")

        f = orchestrator.wait_until_publish_allowed(topic_name)
        rclpy.spin_until_future_complete(node, f)
        node.get_logger().info(f"Publishing message on {topic_name}")
        publishers[topic_name].publish(serialized_data)


    orchestrator.wait_until_pending_actions_complete()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
