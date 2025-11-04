#!/usr/bin/env python3
"""Verify LIVE and HISTORICAL trajectories are actually different by comparing raw waypoint data."""

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np

input_bag = "/home/danielsanchez/t4_dataset/or_test_bag3/input_bag/24dbb0c7-2ec1-422b-9558-e331ecc246a7_2025-10-27-17-57-10_p0900_8.db3"
result_bag = "/home/danielsanchez/t4_dataset/or_test_bag3/out/latest/result_bag/result_bag_0.mcap"
topic = "/planning/trajectory_generator/diffusion_planner_node/output/trajectory"

# Target timestamp from OR event
target_time_ns = int(1761555937.126 * 1e9)  # Near OR event

def get_trajectory_at_time(bag_path, target_ns):
    """Get trajectory closest to target time."""
    storage_options = StorageOptions(uri=bag_path, storage_id='')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    msg_type = get_message('autoware_planning_msgs/msg/Trajectory')

    closest_msg = None
    closest_diff = float('inf')
    closest_bag_t = None

    while reader.has_next():
        (topic_name, data, bag_t) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            header_t_ns = msg.header.stamp.sec * int(1e9) + msg.header.stamp.nanosec
            diff = abs(header_t_ns - target_ns)
            if diff < closest_diff:
                closest_diff = diff
                closest_msg = msg
                closest_bag_t = bag_t

    del reader
    return closest_msg, closest_bag_t

print("Reading HISTORICAL trajectory from input_bag...")
hist_traj, hist_bag_t = get_trajectory_at_time(input_bag, target_time_ns)

print("Reading LIVE trajectory from result_bag...")
live_traj, live_bag_t = get_trajectory_at_time(result_bag, target_time_ns)

if not hist_traj or not live_traj:
    print("ERROR: Could not find trajectories!")
    exit(1)

print(f"\nHISTORICAL (from {input_bag}):")
print(f"  Bag: {input_bag.split('/')[-1][:30]}...")
print(f"  Bag timestamp: {hist_bag_t/1e9:.6f}")
print(f"  Header.stamp: {hist_traj.header.stamp.sec + hist_traj.header.stamp.nanosec/1e9:.6f}")
print(f"  Frame ID: {hist_traj.header.frame_id}")
print(f"  Num points: {len(hist_traj.points)}")

if hist_traj.points:
    print(f"  First point: x={hist_traj.points[0].pose.position.x:.6f}, y={hist_traj.points[0].pose.position.y:.6f}, z={hist_traj.points[0].pose.position.z:.6f}")
    print(f"  Last point:  x={hist_traj.points[-1].pose.position.x:.6f}, y={hist_traj.points[-1].pose.position.y:.6f}")

print(f"\nLIVE (from {result_bag}):")
print(f"  Bag: {result_bag.split('/')[-1][:30]}...")
print(f"  Bag timestamp: {live_bag_t/1e9:.6f}")
print(f"  Header.stamp: {live_traj.header.stamp.sec + live_traj.header.stamp.nanosec/1e9:.6f}")
print(f"  Frame ID: {live_traj.header.frame_id}")
print(f"  Num points: {len(live_traj.points)}")

if live_traj.points:
    print(f"  First point: x={live_traj.points[0].pose.position.x:.6f}, y={live_traj.points[0].pose.position.y:.6f}, z={live_traj.points[0].pose.position.z:.6f}")
    print(f"  Last point:  x={live_traj.points[-1].pose.position.x:.6f}, y={live_traj.points[-1].pose.position.y:.6f}")

print("\n=== COMPARISON ===")
if len(hist_traj.points) == len(live_traj.points):
    # Compare first 5 waypoints in detail
    print("\nFirst 5 waypoints comparison:")
    for i in range(min(5, len(hist_traj.points))):
        h = hist_traj.points[i].pose.position
        l = live_traj.points[i].pose.position
        dist = np.sqrt((h.x - l.x)**2 + (h.y - l.y)**2 + (h.z - l.z)**2)
        print(f"  Point {i}: distance = {dist:.6f}m")
        if dist > 0.001:
            print(f"    HISTORICAL: ({h.x:.6f}, {h.y:.6f}, {h.z:.6f})")
            print(f"    LIVE:       ({l.x:.6f}, {l.y:.6f}, {l.z:.6f})")

    # Check all points
    max_dist = 0
    for i in range(len(hist_traj.points)):
        h = hist_traj.points[i].pose.position
        l = live_traj.points[i].pose.position
        dist = np.sqrt((h.x - l.x)**2 + (h.y - l.y)**2 + (h.z - l.z)**2)
        max_dist = max(max_dist, dist)

    print(f"\nMaximum distance between any waypoints: {max_dist:.6f}m")

    if max_dist < 0.001:
        print("*** TRAJECTORIES ARE IDENTICAL (within 1mm) ***")
        print("*** This confirms diffusion planner is deterministic OR using same model weights ***")
    else:
        print(f"*** TRAJECTORIES ARE DIFFERENT (max {max_dist:.3f}m difference) ***")
else:
    print(f"Different number of points: HISTORICAL={len(hist_traj.points)}, LIVE={len(live_traj.points)}")
