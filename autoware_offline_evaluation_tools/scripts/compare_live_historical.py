#!/usr/bin/env python3
"""
Compare LIVE vs HISTORICAL vs Ground Truth trajectories for OR scene evaluation.

This script reads trajectory data from both input bag (HISTORICAL) and result bag (LIVE)
at the same timestamps to visually verify they are different.
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry


def read_trajectories_at_timestamp(bag_path, topic, target_time, tolerance_ns=50_000_000):
    """Read trajectory from bag near target timestamp."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id='')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    closest_msg = None
    closest_diff = float('inf')

    msg_type = get_message('autoware_planning_msgs/msg/Trajectory')

    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            time_diff = abs(t - target_time)
            if time_diff < closest_diff and time_diff < tolerance_ns:
                closest_diff = time_diff
                closest_msg = deserialize_message(data, msg_type)
                closest_t = t

    del reader
    return closest_msg, closest_t if closest_msg else None


def read_odometry_data(bag_path, topic='/localization/kinematic_state'):
    """Read all odometry data from bag for GT generation."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id='')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    odometry_msgs = []
    msg_type = get_message('nav_msgs/msg/Odometry')

    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            odometry_msgs.append((t, msg))

    del reader
    return odometry_msgs


def plot_comparison(live_traj, historical_traj, gt_poses, timestamp, or_time, output_path):
    """Plot LIVE vs HISTORICAL vs GT trajectories."""
    fig, ax = plt.subplots(figsize=(14, 10))

    # Plot Ground Truth
    if gt_poses:
        gt_x = [p.position.x for p in gt_poses]
        gt_y = [p.position.y for p in gt_poses]
        ax.plot(gt_x, gt_y, 'g-', linewidth=2, label='Ground Truth', alpha=0.8)
        ax.plot(gt_x[0], gt_y[0], 'go', markersize=10, label='GT Start')
        ax.plot(gt_x[-1], gt_y[-1], 'gs', markersize=10, label='GT End')

    # Plot LIVE trajectory
    if live_traj and live_traj.points:
        live_x = [p.pose.position.x for p in live_traj.points]
        live_y = [p.pose.position.y for p in live_traj.points]
        ax.plot(live_x, live_y, 'b--', linewidth=2, label='LIVE (result_bag)', alpha=0.7)
        ax.plot(live_x[0], live_y[0], 'bs', markersize=8, label='LIVE Start')

        # Calculate distance between LIVE first point and GT first point
        if gt_poses:
            live_start_dist = np.sqrt((live_x[0] - gt_x[0])**2 + (live_y[0] - gt_y[0])**2)
            print(f"LIVE first point distance from GT: {live_start_dist:.3f}m")

    # Plot HISTORICAL trajectory
    if historical_traj and historical_traj.points:
        hist_x = [p.pose.position.x for p in historical_traj.points]
        hist_y = [p.pose.position.y for p in historical_traj.points]
        ax.plot(hist_x, hist_y, 'r:', linewidth=2, label='HISTORICAL (input_bag)', alpha=0.7)
        ax.plot(hist_x[0], hist_y[0], 'r^', markersize=8, label='HISTORICAL Start')

        # Calculate distance between HISTORICAL first point and GT first point
        if gt_poses:
            hist_start_dist = np.sqrt((hist_x[0] - gt_x[0])**2 + (hist_y[0] - gt_y[0])**2)
            print(f"HISTORICAL first point distance from GT: {hist_start_dist:.3f}m")

        # Calculate distance between LIVE and HISTORICAL first points
        if live_traj and live_traj.points:
            live_hist_dist = np.sqrt((live_x[0] - hist_x[0])**2 + (live_y[0] - hist_y[0])**2)
            print(f"LIVE vs HISTORICAL first point distance: {live_hist_dist:.3f}m")

    # Mark OR event location
    time_rel = (timestamp - or_time) / 1e9
    ax.plot([], [], 'r*', markersize=15, label=f'Prediction @ t={time_rel:.3f}s')

    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(f'LIVE vs HISTORICAL vs GT Comparison', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved comparison to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Compare LIVE vs HISTORICAL trajectories')
    parser.add_argument('--input-bag', required=True, help='Input bag with HISTORICAL trajectories')
    parser.add_argument('--result-bag', required=True, help='Result bag with LIVE trajectories')
    parser.add_argument('--or-events-json', required=True, help='OR events JSON file')
    parser.add_argument('--output-dir', default='/tmp/or_comparison', help='Output directory for visualizations')
    parser.add_argument('--trajectory-topic', default='/planning/trajectory_generator/diffusion_planner_node/output/trajectory')

    args = parser.parse_args()

    # Load OR events
    with open(args.or_events_json) as f:
        or_data = json.load(f)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Found {len(or_data['or_events'])} OR events")

    # Read odometry for GT (from input bag)
    print(f"Reading odometry from input bag...")
    odom_data = read_odometry_data(args.input_bag)
    print(f"Loaded {len(odom_data)} odometry messages")

    # For each OR event, compare trajectories at prediction times
    for event in or_data['or_events']:
        event_id = event['event_id']
        or_timestamp_ns = int(event['timestamp'] * 1e9)
        window_start_ns = int(event['window_start'] * 1e9)
        window_end_ns = int(event['window_end'] * 1e9)

        print(f"\n=== OR Event #{event_id} at t={event['timestamp']:.3f}s ===")

        # Sample a few prediction times in the window
        sample_times = [
            window_start_ns,
            (window_start_ns + or_timestamp_ns) // 2,
            or_timestamp_ns,
            (or_timestamp_ns + window_end_ns) // 2,
            window_end_ns
        ]

        for idx, pred_time_ns in enumerate(sample_times):
            print(f"\n  Checking prediction at t={(pred_time_ns - or_timestamp_ns)/1e9:.3f}s relative to OR...")

            # Read HISTORICAL trajectory from input bag
            hist_traj, hist_t = read_trajectories_at_timestamp(
                args.input_bag, args.trajectory_topic, pred_time_ns)

            # Read LIVE trajectory from result bag
            live_traj, live_t = read_trajectories_at_timestamp(
                args.result_bag, args.trajectory_topic, pred_time_ns)

            if not hist_traj:
                print(f"    No HISTORICAL trajectory found")
                continue
            if not live_traj:
                print(f"    No LIVE trajectory found")
                continue

            print(f"    HISTORICAL: {len(hist_traj.points)} points at t={hist_t/1e9:.3f}s")
            print(f"    LIVE: {len(live_traj.points)} points at t={live_t/1e9:.3f}s")

            # Generate simple GT (just use odom positions near this time)
            gt_poses = []
            for odom_t, odom in odom_data:
                if window_start_ns <= odom_t <= window_end_ns + int(8e9):  # 8s horizon
                    gt_poses.append(odom.pose.pose)

            # Plot comparison
            output_file = output_dir / f"comparison_event{event_id}_sample{idx}.png"
            plot_comparison(live_traj, hist_traj, gt_poses, pred_time_ns, or_timestamp_ns, output_file)

    print(f"\nDone! Visualizations saved to: {output_dir}")


if __name__ == '__main__':
    main()
