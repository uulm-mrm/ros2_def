#!/usr/bin/env python3
import random
import rosbag2_py
from numpy.random import default_rng


def create_reader(uri: str, storage_identifier: str) -> (rosbag2_py.SequentialReader, rosbag2_py.ConverterOptions):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri, storage_id=storage_identifier)
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(serialization_format, serialization_format)
    reader.open(storage_options, converter_options)
    return reader, converter_options


def create_writer(uri: str, converter_options: rosbag2_py.ConverterOptions, storage_identifier: str):
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri, storage_id=storage_identifier)
    writer.open(storage_options, converter_options)
    return writer


def main():
    bag_uri = "/home/gja38/aduulm_sandbox_sil/rosbags/rosbag2_2023_05_12-13_11_59"
    prune_probability = 0.1
    timestamp_stddev_ns = 600000
    timestamp_max_offset_ns = 2000000
    zero_time = 1000000000000000000
    assert (timestamp_max_offset_ns >= 0)
    rng = default_rng(20230523)

    metadata: rosbag2_py.BagMetadata = rosbag2_py.Info().read_metadata(bag_uri, "")
    reader, converter_options = create_reader(bag_uri, metadata.storage_identifier)
    writer = create_writer(bag_uri + "_drop_reorder", converter_options, metadata.storage_identifier)

    for topic_metadata in reader.get_all_topics_and_types():
        writer.create_topic(topic_metadata)

    while reader.has_next():
        topic_name, serialized_data, time_stamp_ns = reader.read_next()
        if random.random() <= prune_probability:
            continue

        if time_stamp_ns == 0:
            time_stamp_ns = zero_time

        time_offset = max(-timestamp_max_offset_ns,
                          min(timestamp_max_offset_ns,
                              int(rng.normal(scale=timestamp_stddev_ns))))

        writer.write(topic_name, serialized_data, time_stamp_ns + time_offset)


if __name__ == '__main__':
    main()
