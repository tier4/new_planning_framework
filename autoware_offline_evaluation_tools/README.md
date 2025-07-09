# Offline Evaluation Tools

This package provides tools for offline evaluation of Autoware's trajectory planning performance using recorded rosbag data.

## Overview

The offline evaluation tools support two evaluation modes:

- **Open Loop**: Evaluates manual driving trajectories (not yet implemented)
- **Closed Loop**: Evaluates autonomous driving trajectories by analyzing recorded data

## Features

### Closed Loop Evaluation

The closed loop evaluation mode analyzes autonomous driving data by:

1. **Time-synchronized data extraction**: Extracts kinematic states at 100ms intervals
2. **Data synchronization**: Synchronizes trajectory, acceleration, steering, and object data based on header timestamps
3. **Metric calculation**: Computes various performance metrics including:
   - Position errors (lateral and longitudinal)
   - Velocity errors
   - Yaw angle errors
   - Acceleration and jerk
   - Time to collision (TTC)
   - Path curvature

4. **Result storage**: Saves evaluation results to:
   - ROSbag file with time-series metrics
   - Summary text file with statistics

## Usage

```sh
ros2 launch autoware_offline_evaluation_tools offline_evaluator.launch.xml \
  bag_path:=<ROSBAG_PATH> \
  map_path:=<MAP_PATH> \
  vehicle_model:=<VEHICLE_MODEL> \
  sensor_model:=<SENSOR_MODEL>
```

## Parameters

Key parameters in `config/offline_evaluation.param.yaml`:

- `evaluation.mode`: Evaluation mode (`"open_loop"` or `"closed_loop"`)
- `evaluation_interval_ms`: Sampling interval for kinematic states (default: 100ms)
- `sync_tolerance_ms`: Time synchronization tolerance (default: 50ms)
- `evaluation_output_bag_path`: Output path for evaluation results
- `summary_output_file`: Output path for summary statistics

## Output

### Evaluation Metrics (ROSbag)

The following topics are recorded in the evaluation output bag:

| Topic | Type | Description |
|-------|------|-------------|
| `/evaluation/lateral_error` | `std_msgs/msg/Float64` | Lateral deviation from planned trajectory |
| `/evaluation/velocity_error` | `std_msgs/msg/Float64` | Velocity tracking error |
| `/evaluation/ttc` | `std_msgs/msg/Float64` | Time to collision |
| `/evaluation/position_errors` | `geometry_msgs/msg/PointStamped` | Combined position errors (x: lateral, y: longitudinal, z: yaw) |
| `/evaluation/acceleration_metrics` | `geometry_msgs/msg/PointStamped` | Acceleration metrics (x: longitudinal, y: lateral, z: jerk) |

### Summary Statistics (Text File)

The summary file includes:
- Total number of evaluated samples
- Mean and maximum lateral error
- Mean and maximum velocity error
- Mean and maximum acceleration
- Minimum TTC
- Total evaluation time

## Implementation Details

The evaluation system consists of:

1. **BagHandler**: Manages rosbag data buffering and time-based data retrieval
2. **ClosedLoopEvaluator**: Calculates evaluation metrics from synchronized data
3. **OfflineEvaluatorNode**: Main node orchestrating the evaluation process

The system uses message header timestamps for precise time synchronization, ensuring accurate correlation between different data streams.