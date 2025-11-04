#!/usr/bin/env python3
"""Debug script to check trajectory timestamps and time_from_start values."""

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_path = sys.argv[1] if len(sys.argv) > 1 else "/home/danielsanchez/t4_dataset/or_test_bag3/out/latest/result_bag/result_bag_0.mcap"
topic = "/planning/trajectory_generator/diffusion_planner_node/output/trajectory"

storage_options = StorageOptions(uri=bag_path, storage_id='')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = SequentialReader()
reader.open(storage_options, converter_options)

msg_type = get_message('autoware_planning_msgs/msg/Trajectory')

count = 0
while reader.has_next() and count < 5:
    (topic_name, data, bag_time) = reader.read_next()
    if topic_name == topic:
        msg = deserialize_message(data, msg_type)

        header_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        bag_stamp = bag_time / 1e9

        print(f"\nTrajectory #{count}:")
        print(f"  Bag timestamp: {bag_stamp:.6f}s")
        print(f"  Header.stamp: {header_stamp:.6f}s")
        print(f"  Difference: {(header_stamp - bag_stamp):.6f}s")

        if msg.points:
            first_tfs = msg.points[0].time_from_start.sec + msg.points[0].time_from_start.nanosec / 1e9
            print(f"  First point time_from_start: {first_tfs:.6f}s")
            print(f"  Num points: {len(msg.points)}")
            if len(msg.points) > 1:
                last_tfs = msg.points[-1].time_from_start.sec + msg.points[-1].time_from_start.nanosec / 1e9
                print(f"  Last point time_from_start: {last_tfs:.6f}s")

        count += 1

del reader
print(f"\nChecked {count} trajectories from {bag_path}")
