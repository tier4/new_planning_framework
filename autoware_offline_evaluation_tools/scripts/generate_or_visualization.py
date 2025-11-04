#!/usr/bin/env python3
"""
Generate debug visualization for OR scene trajectory comparison
"""
import sys
import json
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def quat_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)

def create_bbox_polygon(x, y, yaw, length, width):
    """Create bounding box polygon corners in global frame"""
    # Half dimensions
    half_l = length / 2.0
    half_w = width / 2.0

    # Corners in vehicle frame (front-left, front-right, rear-right, rear-left)
    corners_local = np.array([
        [half_l, half_w],    # Front-left
        [half_l, -half_w],   # Front-right
        [-half_l, -half_w],  # Rear-right
        [-half_l, half_w]    # Rear-left
    ])

    # Rotation matrix
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rot = np.array([[cos_yaw, -sin_yaw],
                    [sin_yaw, cos_yaw]])

    # Transform to global frame
    corners_global = corners_local @ rot.T
    corners_global[:, 0] += x
    corners_global[:, 1] += y

    return corners_global

def generate_or_visualization(data_json_path, output_image_path):
    """Generate bird's eye view plot of predicted vs GT trajectories"""

    with open(data_json_path, 'r') as f:
        data = json.load(f)

    # Extract data
    pred_poses = data['predicted_trajectory']
    gt_poses = data['ground_truth_trajectory']
    metrics = data['metrics']
    event_info = data['event_info']
    objects = data.get('objects', [])

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 12))

    # Plot trajectories
    pred_x = [p['x'] for p in pred_poses]
    pred_y = [p['y'] for p in pred_poses]
    gt_x = [p['x'] for p in gt_poses]
    gt_y = [p['y'] for p in gt_poses]

    # Debug output (optional)
    # print(f"Predicted trajectory: {len(pred_x)} points")
    # print(f"GT trajectory: {len(gt_x)} points")
    # print(f"Objects: {len(objects)}")

    # Make trajectories visible with thinner lines
    ax.plot(gt_x, gt_y, 'g-', linewidth=2.5, label='Ground Truth', alpha=0.9, zorder=5)
    ax.plot(pred_x, pred_y, 'b--', linewidth=2, label='Predicted', alpha=0.9, zorder=5, dashes=(5, 2))

    # Mark start and end points (same size as trajectory line markers)
    ax.plot(gt_x[0], gt_y[0], 'go', markersize=8, label='GT Start', zorder=6)
    ax.plot(gt_x[-1], gt_y[-1], 'gs', markersize=8, label='GT End', zorder=6)
    ax.plot(pred_x[0], pred_y[0], 'bo', markersize=8, label='Pred Start', zorder=6)
    ax.plot(pred_x[-1], pred_y[-1], 'bs', markersize=8, label='Pred End', zorder=6)

    # Mark OR event location (vehicle position at OR)
    or_x = event_info['vehicle_x_at_or']
    or_y = event_info['vehicle_y_at_or']
    ax.plot(or_x, or_y, 'r*', markersize=15, label='OR Event', zorder=10, markeredgewidth=1.5)

    # Plot objects as bounding boxes
    class_colors = {
        1: 'cyan',      # CAR
        2: 'orange',    # TRUCK
        3: 'purple',    # BUS
        6: 'yellow',    # BICYCLE
        7: 'pink',      # PEDESTRIAN
    }

    for i, obj in enumerate(objects):
        # Get object pose
        obj_x = obj['x']
        obj_y = obj['y']

        # Calculate yaw from quaternion
        quat = obj['orientation']
        yaw = quat_to_yaw(quat['x'], quat['y'], quat['z'], quat['w'])

        # Use footprint if available, otherwise create bbox from dimensions
        if 'footprint' in obj and obj['footprint']:
            # Use provided footprint polygon
            footprint_pts = np.array([[pt['x'], pt['y']] for pt in obj['footprint']])
            polygon = patches.Polygon(footprint_pts, fill=False, edgecolor='gray',
                                    linewidth=1.5, linestyle='-', alpha=0.7)
        else:
            # Create bounding box from dimensions
            length = obj['length']
            width = obj['width']
            corners = create_bbox_polygon(obj_x, obj_y, yaw, length, width)

            # Get color based on class
            class_label = obj.get('class_label', 0)
            color = class_colors.get(class_label, 'gray')

            polygon = patches.Polygon(corners, fill=False, edgecolor=color,
                                    linewidth=2.5, linestyle='-', alpha=0.8)

        ax.add_patch(polygon)

        # Add label for first object
        if i == 0:
            ax.plot([], [], color='gray', linewidth=1.5, label='Objects', alpha=0.7)

    # Add metrics text box
    coverage_pct = metrics.get('gt_coverage_ratio', 1.0) * 100
    num_objects = len(objects)

    # Calculate distance between starts
    if pred_x and gt_x:
        start_dist = np.sqrt((pred_x[0] - gt_x[0])**2 + (pred_y[0] - gt_y[0])**2)
    else:
        start_dist = 0.0

    metrics_text = f"""OR Scene Evaluation
Event #{event_info['event_id']}
Prediction: t={metrics['time_relative_to_or']:+.3f}s
Vehicle Speed: {event_info['vehicle_speed_at_or']:.2f} m/s

ADE: {metrics['ade']:.3f} m
FDE: {metrics['fde']:.3f} m
Lateral Dev: {metrics['mean_lateral_deviation']:+.3f} m
Start Dist: {start_dist:.3f} m
Min TTC: {metrics['min_ttc']:.1f} s

Coverage: {coverage_pct:.1f}%
Points: {metrics['num_points']}/{metrics.get('num_points_original', metrics['num_points'])}
Duration: {metrics['trajectory_duration']:.1f}/{metrics.get('trajectory_duration_original', metrics['trajectory_duration']):.1f} s
Objects: {num_objects}"""

    ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace')

    # Calculate bounds from trajectories ONLY (not objects)
    # Use start and end points + 10% epsilon as user requested
    all_traj_x = pred_x + gt_x
    all_traj_y = pred_y + gt_y

    if not all_traj_x or not all_traj_y:
        print("ERROR: No trajectory data to plot!")
        return

    x_min = min(all_traj_x)
    x_max = max(all_traj_x)
    y_min = min(all_traj_y)
    y_max = max(all_traj_y)

    # Add 10% epsilon on each side
    x_range = x_max - x_min
    y_range = y_max - y_min
    x_epsilon = max(1.0, x_range * 0.1)  # At least 1 meter
    y_epsilon = max(1.0, y_range * 0.1)  # At least 1 meter


    # Set bounds with epsilon
    ax.set_xlim(x_min - x_epsilon, x_max + x_epsilon)
    ax.set_ylim(y_min - y_epsilon, y_max + y_epsilon)

    # Formatting
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title(f'OR Scene Trajectory Comparison\n{event_info["bag_name"]}', fontsize=14)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)

    # Save
    plt.tight_layout()
    plt.savefig(output_image_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Saved visualization to: {output_image_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: generate_or_visualization.py <data.json> <output.png>")
        sys.exit(1)

    generate_or_visualization(sys.argv[1], sys.argv[2])
