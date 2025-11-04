#!/usr/bin/env python3
"""Check the delay between sampling time and trajectory header.stamp."""

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Simulate OR scene evaluation sampling
or_timestamp = 1761555937.195  # OR event #1
window_start = or_timestamp - 0.5
window_end = or_timestamp + 0.5

bag_path = "/home/danielsanchez/t4_dataset/or_test_bag3/out/latest/result_bag/result_bag_0.mcap"
topic = "/planning/trajectory_generator/diffusion_planner_node/output/trajectory"

storage_options = StorageOptions(uri=bag_path, storage_id='')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = SequentialReader()
reader.open(storage_options, converter_options)

msg_type = get_message('autoware_planning_msgs/msg/Trajectory')

# Simulate sampling at 100ms intervals in OR window
sample_times_ns = []
t = int(window_start * 1e9)
end_ns = int(window_end * 1e9)
while t <= end_ns:
    sample_times_ns.append(t)
    t += int(0.1 * 1e9)  # 100ms

print(f"Checking {len(sample_times_ns)} samples in OR window [{window_start:.3f}, {window_end:.3f}]")
print(f"Vehicle speed: 8.59 m/s")
print()

# For each sample time, find closest trajectory
for sample_t_ns in sample_times_ns:
    sample_t_s = sample_t_ns / 1e9

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    closest_msg = None
    closest_diff_ns = float('inf')
    closest_bag_t = None

    while reader.has_next():
        (topic_name, data, bag_t_ns) = reader.read_next()
        if topic_name == topic:
            diff_ns = abs(bag_t_ns - sample_t_ns)
            if diff_ns < closest_diff_ns and diff_ns < 50_000_000:  # 50ms tolerance
                msg = deserialize_message(data, msg_type)
                closest_msg = msg
                closest_diff_ns = diff_ns
                closest_bag_t = bag_t_ns

    del reader

    if closest_msg:
        header_t_ns = closest_msg.header.stamp.sec * int(1e9) + closest_msg.header.stamp.nanosec
        header_t_s = header_t_ns / 1e9

        delay_ns = sample_t_ns - header_t_ns
        delay_s = delay_ns / 1e9
        delay_m = delay_s * 8.59  # Distance at 8.59 m/s

        print(f"Sample t={sample_t_s - or_timestamp:+.3f}s: traj header.stamp delay = {delay_s*1000:.1f}ms = {delay_m:.3f}m")
    else:
        print(f"Sample t={sample_t_s - or_timestamp:+.3f}s: No trajectory found!")

print()
print("If delay is positive and ~0.9m, trajectory is stale (generated before current time)")
print("If delay is negative, trajectory is from future (impossible, indicates wrong sync)")
