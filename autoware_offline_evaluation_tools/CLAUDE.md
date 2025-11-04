# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Purpose

This package provides offline trajectory evaluation tools for Autoware's planning system. It analyzes recorded rosbag data to evaluate trajectory planning performance in three modes:

- **Open Loop**: Evaluates trajectory prediction accuracy by comparing predicted waypoints against interpolated ground truth from recorded odometry
- **Closed Loop**: Evaluates autonomous driving performance by analyzing how well the vehicle followed planned trajectories
- **OR Scene** (Override Regression): Evaluates planning performance around detected safety driver override events to assess if new models would prevent future overrides

This package is typically invoked as a post-processing step by `driving_log_replayer_v2` after simulation runs.

## Build and Test Commands

```bash
# Build this package only
colcon build --packages-select autoware_offline_evaluation_tools

# Build with dependencies
colcon build --packages-up-to autoware_offline_evaluation_tools

# Run tests
colcon test --packages-select autoware_offline_evaluation_tools

# Run with verbose output
colcon test --packages-select autoware_offline_evaluation_tools --event-handlers console_direct+

# Run pre-commit checks (from repository root)
pre-commit run --all-files
```

## Running Evaluations

```bash
# Standalone evaluation
ros2 launch autoware_offline_evaluation_tools offline_evaluator.launch.xml \
  bag_path:=/path/to/input.db3 \
  map_path:=/path/to/map_dir \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  rviz:=false

# Override topics if needed
ros2 launch autoware_offline_evaluation_tools offline_evaluator.launch.xml \
  bag_path:=/path/to/input.db3 \
  map_path:=/path/to/map_dir \
  input/trajectory:=/planning/diffusion_planner/trajectory

# Direct node execution (for debugging)
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  -p bag_path:=/path/to/input.db3 \
  -p evaluation.mode:=open_loop \
  -p trajectory_topic:=/planning/diffusion_planner/trajectory
```

## Architecture Overview

### Component Hierarchy

```
OfflineEvaluatorNode (Main ROS 2 node)
├── Manages rosbag I/O and route handler
├── Delegates evaluation to mode-specific evaluators
│
├── BaseEvaluator (Abstract base class)
│   ├── Common bag processing logic
│   ├── Metrics calculation framework
│   ├── JSON/bag output writing
│   │
│   ├── ClosedLoopEvaluator
│   │   └── Evaluates how well vehicle followed planned trajectory
│   │
│   └── OpenLoopEvaluator
│       └── Evaluates prediction accuracy vs interpolated ground truth
│
└── BagHandler (Data synchronization layer)
    ├── Time-windowed message buffering (Buffer<T>)
    ├── Multi-sensor data synchronization (SynchronizedData)
    └── Closest-timestamp matching with tolerance
```

### Key Design Patterns

**1. Template-Based Type Safety**
- `Buffer<T>` provides generic message buffering
- `process_and_append_message<T>()` handles type-safe deserialization
- Compile-time type checking with `has_header_stamp<T>` trait

**2. Dual Timestamp System**
- `timestamp`: Header timestamp for logical synchronization
- `bag_timestamp`: Recording timestamp for bag output alignment

**3. Circular Buffer Memory Management**
- Configurable buffer duration (default: 20 seconds)
- Automatic cleanup removes oldest 10% when size threshold exceeded
- Prevents unbounded memory growth during long evaluations

**4. Polymorphic Evaluator Selection**
- Runtime mode selection via parameter: `"open_loop"` or `"closed_loop"`
- Virtual interface enables extensibility for new evaluation modes

### Data Flow

```
ROSbag Input
    ↓
BagHandler::process_bag_common()
    ↓ (buffers messages in time windows)
BagHandler::get_synchronized_data()
    ↓ (finds closest-timestamp matches within tolerance)
SynchronizedData (multi-sensor fusion)
    ↓
Evaluator::evaluate() (mode-specific)
    ↓
TrajectoryMetrics (per-point and summary)
    ↓
Output: ROSbag topics + JSON files
```

### Time Synchronization

The system synchronizes multiple sensor streams using header timestamps:

- **Evaluation interval**: 100ms (configurable via `evaluation_interval_ms`)
- **Sync tolerance**: 50ms default (configurable via `sync_tolerance_ms`)
- **Buffer duration**: 20 seconds sliding window (configurable via `buffer_duration_sec`)

Example: At timestamp T, the system retrieves the closest messages within ±50ms from:
- Kinematic state (odometry)
- Planned trajectory
- Acceleration
- Steering status
- Predicted objects

## Critical Files and Locations

### Source Code
- `src/node.{hpp,cpp}` - Main ROS 2 node (431 LOC)
- `src/bag_handler.{hpp,cpp}` - Data synchronization (567 LOC)
- `src/base_evaluator.{hpp,cpp}` - Abstract evaluator (578 LOC)
- `src/closed_loop_evaluator.{hpp,cpp}` - Closed-loop implementation (708 LOC)
- `src/open_loop_evaluator.{hpp,cpp}` - Open-loop implementation (893 LOC)

### Configuration
- `config/offline_evaluation.param.yaml` - All runtime parameters
- `launch/offline_evaluator.launch.xml` - Launch configuration with topic remapping

### Tests
- `test/test_bag_handler.cpp` - Buffer and synchronization tests
- `test/test_replay_evaluation.cpp` - End-to-end evaluation tests
- `test/test_displacement_errors.cpp` - Error calculation validation
- `test/test_ground_truth_generation.cpp` - Interpolation tests

## Parameter Management

All parameters defined in `config/offline_evaluation.param.yaml`:

### Core Evaluation
```yaml
evaluation:
  mode: "open_loop"              # "open_loop" or "closed_loop"

evaluation_interval_ms: 100.0    # Sampling interval for kinematic states
sync_tolerance_ms: 50.0          # Time sync tolerance
```

### Buffer Configuration
```yaml
buffer_duration_sec: 20.0        # Sliding window duration
max_buffer_messages: 10000       # Memory limit per buffer
```

### Topic Configuration (with defaults)
```yaml
# Optional overrides - defaults shown in comments
trajectory_topic: "/planning/diffusion_planner/trajectory"
objects_topic: "/perception/object_recognition/objects"
# odometry_topic: "/localization/kinematic_state"
# acceleration_topic: "/localization/acceleration"
# steering_topic: "/vehicle/status/steering_status"
# tf_topic: "/tf"
# route_topic: "/planning/mission_planning/route"
```

### Output Paths
```yaml
evaluation_output_bag_path: "~/trajectory_evaluation_results.bag"
json_output_path: "~/evaluation_result.json"
summary_output_file: "~/evaluation_summary.txt"
```

## Evaluation Modes Explained

### Closed Loop Evaluation

**Purpose**: Measure how well the vehicle followed its planned trajectory during autonomous driving.

**Metrics Calculated** (`ClosedLoopTrajectoryMetrics`):
- `lateral_error` - Deviation from preferred lane centerline
- `lateral_acceleration` - Lateral motion acceleration
- `longitudinal_acceleration` - Forward/backward acceleration
- `jerk` - Rate of acceleration change
- `min_ttc` - Minimum time to collision with predicted objects
- `steering_angular_velocity` - Steering rate change (oscillation detection)
- `yaw_rate` - Vehicle rotation rate

**Output Topics**:
```
/closed_loop/lateral_error         (Float64)
/closed_loop/lateral_jerk           (Float64)
/closed_loop/acceleration           (Float64)
/closed_loop/jerk                   (Float64)
/closed_loop/ttc                    (Float64)
/closed_loop/trajectory_markers     (MarkerArray)
```

**Summary Statistics**:
- Mean/max/std lateral error
- Mean/max lateral jerk and acceleration
- Minimum TTC across trajectory
- Steering reversals and oscillation metrics
- Distance and time tracking

### Open Loop Evaluation

**Purpose**: Measure trajectory prediction accuracy by comparing predicted waypoints against ground truth.

**Metrics Calculated** (`OpenLoopTrajectoryMetrics`):
- `lateral_deviations` - Per-point lateral error in vehicle frame
- `longitudinal_deviations` - Per-point longitudinal error in vehicle frame
- `displacement_errors` - Euclidean distance per point
- `ade` - Average Displacement Error (mean across trajectory points)
- `fde` - Final Displacement Error (error at trajectory endpoint)
- `ttc` - Time to collision at each predicted point

**Output Topics**:
```
/open_loop/lateral_deviation       (Float64)
/open_loop/longitudinal_deviation  (Float64)
/open_loop/displacement_error      (Float64)
/open_loop/ade                     (Float64)
/open_loop/ttc                     (Float64)
/open_loop/trajectory_markers      (MarkerArray)
```

**Ground Truth Generation**:
- Interpolates recorded odometry data over trajectory evaluation window
- Uses SLERP for pose orientation interpolation
- Handles time gaps via configurable tolerance

**Summary Statistics**:
- Mean/std/max ADE and FDE
- Mean/std/max lateral/longitudinal deviation
- Coverage metrics: valid trajectories, fully valid trajectories, mean coverage ratio

### OR Scene Evaluation (Override Regression Testing)

**Purpose**: Evaluate planning model performance around safety driver override (OR) events to assess if newly trained models would prevent future overrides.

**Use Case**: Training models with OR data and testing if new versions would have performed better in historical override scenarios.

**How It Works - Two-Stage Approach**:

**Stage 1: OR Event Detection**
- Reads `/vehicle/status/control_mode` topic from bag
- Detects transitions: AUTONOMOUS (1) → MANUAL (4)
- Records OR timestamp and creates symmetric evaluation window: [OR - t, OR + t]
- Saves OR events to JSON for reuse

**Stage 2: Trajectory Evaluation**
- For each OR event:
  - Loads all bag data (needed for GT interpolation over 8-second trajectory horizon)
  - Filters trajectory predictions in time window [OR - 0.5s, OR + 0.5s]
  - Compares each predicted trajectory to ground truth (from actual kinematic_state)
  - Calculates ADE, FDE, lateral deviation, TTC
- Aggregates metrics per-event and across all events

**Key Topics**:
- **OR Detection**: `/vehicle/status/control_mode` (ControlModeReport)
  - AUTONOMOUS (1) = Autonomous driving
  - MANUAL (4) = Safety driver override
- **Trajectory**: Configurable (default: `/planning/diffusion_planner/trajectory`)
- **Ground Truth**: `/localization/kinematic_state` (interpolated poses)

**Metrics** (`ORTrajectoryMetrics` - per prediction):
- `ade` - Average Displacement Error vs ground truth
- `fde` - Final Displacement Error at trajectory endpoint
- `mean_lateral_deviation` - Average lateral error in vehicle frame
- `max_lateral_deviation` - Maximum lateral error
- `min_ttc` - Minimum time to collision
- `time_relative_to_or_sec` - When prediction was made relative to OR (negative = before)

**Output Structure** (Three Levels):

1. **Per-Trajectory** (finest granularity):
```json
{
  "prediction_time": 1761557741.545,
  "time_relative_to_or": -0.454,  // 0.454s before OR
  "ade": 0.559,
  "fde": 2.755,
  "mean_lateral_deviation": -0.022,
  "num_points": 80,
  "trajectory_duration": 7.9
}
```

2. **Per-OR-Event** (aggregate per override):
```json
{
  "event_id": 0,
  "or_timestamp": 1761557741.999,
  "total_predictions": 10,
  "predictions_before_or": 5,
  "predictions_after_or": 5,
  "mean_ade": 0.736,
  "min_ade": 0.559,
  "max_ade": 0.931,
  "mean_fde": 3.249
}
```

3. **Overall Summary** (across all OR events):
```json
{
  "total_or_events": 1,
  "events_with_valid_predictions": 1,
  "mean_ade_all_events": 0.736,
  "mean_fde_all_events": 3.249,
  "mean_predictions_per_event": 10.0
}
```

**Success Criteria** (Optional - disabled by default):
```yaml
success_criteria:
  enabled: false  # Like open_loop, just measure by default
  max_ade: 1.0
  max_fde: 1.5
  max_lateral_deviation: 0.5
  min_ttc: 3.0
```

**Configuration** (`config/offline_evaluation.param.yaml`):
```yaml
evaluation:
  mode: "or_scene"

or_scene_evaluation:
  time_window_sec: 0.5  # Window on each side: [OR-0.5s, OR+0.5s]

  # Trajectory source selection
  evaluate_live_trajectories: true  # true = DLR result_bag (LIVE model output)
                                    # false = input_bag (HISTORICAL recorded trajectories)
  input_bag_path: ""  # Path to input bag (only if evaluate_live_trajectories=false)

  control_mode_topic: "/vehicle/status/control_mode"

  # Stage 1: OR extraction
  skip_or_extraction: false  # Set true if or_events.json exists
  or_events_input_path: ""  # Load pre-extracted OR events
  or_events_output_path: "~/or_events.json"

  # Success criteria (optional)
  success_criteria:
    enabled: false

  # Debug visualization
  enable_debug_visualization: false
  debug_output_dir: "~/or_scene_debug_images"
```

**Live vs Historical Trajectory Evaluation**:

OR scene evaluation supports two modes:

**Mode 1: LIVE Trajectories (Default)** - Evaluate newly trained model
- Used with DLR workflow
- Evaluates trajectories from `result_bag_0.mcap` (DLR simulation output)
- Compares: What NEW model predicted during replay vs what actually happened (GT)
- Use case: Testing if newly trained model would prevent historical ORs

**Mode 2: HISTORICAL Trajectories** - Analyze original performance
- Evaluates trajectories from input bag (original recording)
- Compares: What model predicted during original drive vs what actually happened
- Use case: Understanding why historical ORs occurred

**Workflow with DLR** (LIVE trajectories):
```bash
# 1. Run DLR simulation (replays bag, runs planning nodes, records to result_bag)
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml

# 2. DLR post-processing automatically runs OR scene evaluation on result_bag
# (Or run manually):
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  -p bag_path:=/path/to/output_dir/result_bag/result_bag_0.mcap \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/planning/diffusion_planner/trajectory
```

**Standalone Usage** (HISTORICAL trajectories):
```bash
# Evaluate original recorded trajectories
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  -p bag_path:=/path/to/input.db3 \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.evaluate_live_trajectories:=false \
  -p or_scene_evaluation.enable_debug_visualization:=true
```

**Interpreting Results**:
- **Low ADE/FDE**: Model predictions were similar to actual vehicle behavior (OR may have been for non-trajectory reasons)
- **High ADE/FDE**: Model would have done something very different (OR likely necessary to correct trajectory)
- **Increasing ADE over time**: Model performance degraded as OR approached (warning signal)
- **Check predictions_before_or vs predictions_after_or**: Evaluate how early model issues were detectable

**Partial GT Comparison**:

When OR events occur near the end of a rosbag, trajectories may extend beyond available data. The system handles this gracefully:

- **Truncates** trajectory to available GT time range
- **Reports** coverage ratio (e.g., 25% = only first 2 seconds of 8-second trajectory compared)
- **Provides** metrics for truncated portion (ADE/FDE still meaningful for available range)
- **Tracks** both original and truncated trajectory info in JSON output:
  ```json
  {
    "num_points": 23,              // Points actually evaluated
    "num_points_original": 80,     // Original trajectory length
    "gt_coverage_ratio": 0.287,    // 28.7% of trajectory has GT
    "trajectory_duration": 2.3,    // Evaluated duration
    "trajectory_duration_original": 7.9  // Full trajectory duration
  }
  ```

**Debug Visualization**:

When enabled (`enable_debug_visualization: true`), generates PNG images for each prediction:

- **Image naming**: `or_event_<id>_pred_<timestamp>.png`
- **Location**: Configurable via `debug_output_dir` parameter
- **Content**:
  - Green solid line: Ground truth trajectory (actual vehicle path)
  - Blue dashed line: Predicted trajectory
  - Red star: OR event location
  - Colored bounding boxes: Objects (cyan=CAR, orange=TRUCK, yellow=BICYCLE, pink=PEDESTRIAN)
  - Metrics overlay: ADE, FDE, lateral deviation, TTC, coverage, speed, object count, relative time
- **Requirements**: Python 3 with matplotlib and numpy installed
- **Object Source**: Uses objects from `objects_topic` parameter (default: `/perception/object_recognition/objects`)
- **Bounding Box Generation**: For BOUNDING_BOX type objects, generates oriented rectangles from position, orientation, and dimensions

**Important Notes**:
- GT generation requires full bag data (8-second trajectory horizon needs future kinematic_state)
- Currently processes entire bag for each OR event (optimization opportunity for future)
- Uses proper ROS message deserialization (not byte offset parsing)
- Time values: negative = before OR, positive = after OR
- Partial GT allows evaluation even when OR occurs near bag end

## Integration with driving_log_replayer_v2

This package is invoked by `driving_log_replayer_v2` during post-processing:

**Location**: `/home/danielsanchez/pilot-auto/src/simulator/driving_log_replayer_v2/driving_log_replayer_v2/launch/post_process.launch.py` (lines 193-239)

**Invocation**:
```python
openloop_analysis_cmd = [
    "ros2", "run",
    "autoware_offline_evaluation_tools",
    "offline_evaluator_node",
    "--ros-args",
    "-p", f"bag_path:={conf['result_bag_path']}/result_bag_0.mcap",
    "-p", "evaluation.mode:=open_loop",
    "-p", "trajectory_topic:=/planning/diffusion_planner/trajectory",
    "-p", f"evaluation_output_bag_path:={conf['result_bag_path']}/post_process",
]
```

**Workflow**:
1. `driving_log_replayer_v2` runs simulation and records Autoware outputs
2. Post-processing calls this package to evaluate trajectory quality
3. Results merged back into result bag for unified analysis

## Output Format

### JSON Output Structure

**Summary Level**:
```json
{
  "evaluation_mode": "closed_loop",
  "summary": {
    "mean_lateral_error": 0.123,
    "max_lateral_error": 0.456,
    "std_lateral_error": 0.089,
    "mean_acceleration": 0.234,
    "min_ttc": 2.5
  },
  "metadata": {
    "bag_path": "/path/to/input.db3",
    "evaluation_duration": 120.5,
    "num_samples": 1205
  }
}
```

**Detailed Results**: Per-point metrics for every evaluation sample, enabling post-hoc analysis.

### ROSbag Output

Each evaluation produces a rosbag with:
- Time-series metric topics (listed above per mode)
- Trajectory visualization markers
- Static TF transforms (`/tf_static`)

## Important Implementation Details

### Special Template Handling

`SteeringReport` and `TFMessage` have custom template specializations in `BagHandler` because their timestamps are not in standard header locations:

```cpp
template <>
inline std::optional<rclcpp::Time> BagHandler::get_message_stamp<SteeringReport>(
  const std::shared_ptr<SteeringReport> & msg)
{
  return rclcpp::Time(msg->stamp);  // Not msg->header.stamp
}
```

### Memory Management

Buffers automatically clean up old data:
```cpp
if (msgs.size() > max_buffer_size) {
  const size_t remove_count = max_buffer_size / 10;  // Remove oldest 10%
  msgs.erase(msgs.begin(), msgs.begin() + remove_count);
}
```

### Timestamp Normalization

Output timestamps are normalized to start at 0 for consistency:
```cpp
relative_time = current_time - evaluation_start_time;
```

## Adding New Metrics

To add a new metric to an evaluator:

1. **Add to metrics struct** (e.g., `ClosedLoopTrajectoryMetrics` in `closed_loop_evaluator.hpp`)
2. **Calculate metric** in `evaluate()` method
3. **Add to summary** in `get_summary_as_json()`
4. **Create output topic** in `get_result_topics()` and publish in `evaluate()`
5. **Update tests** in `test/`

Example flow:
```cpp
// 1. Add to struct
struct ClosedLoopTrajectoryMetrics {
  double my_new_metric;
};

// 2. Calculate in evaluate()
metrics.my_new_metric = calculate_my_metric(data);

// 3. Add to summary JSON
summary["my_new_metric"] = calculate_statistics(metrics.my_new_metric);

// 4. Publish to topic
auto msg = std_msgs::msg::Float64();
msg.data = metrics.my_new_metric;
publishers_["my_new_metric"]->publish(msg);
```

## Dependencies

### Autoware Packages
- `autoware_trajectory_selector_common` - Shared utilities
- `autoware_route_handler` - Route queries for lane-based metrics
- `autoware_lanelet2_extension` - Lanelet2 map utilities
- `autoware_motion_utils` - Trajectory interpolation
- `autoware_planning_msgs` - Trajectory message types
- `autoware_vehicle_msgs` - Vehicle state message types

### External Libraries
- `nlohmann/json` - JSON serialization
- `magic_enum` - Enum reflection
- `rosbag2_cpp` - Rosbag file I/O
- `tf2` / `tf2_ros` - Transform handling

## Common Development Patterns

### Reading Multiple Files in Parallel

When exploring code, use parallel reads:
```cpp
// Read related files simultaneously for context
Read(bag_handler.hpp)
Read(bag_handler.cpp)
Read(base_evaluator.hpp)
```

### Template Pattern Usage

Follow existing template patterns for type-safe message handling:
```cpp
template <typename T>
void Buffer<T>::append(const std::shared_ptr<T> & msg) {
  msgs.push_back(msg);
  cleanup_old_messages();
}
```

### Error Handling

Use optional types for uncertain data:
```cpp
auto closest_msg = buffer->get_closest(timestamp, tolerance);
if (!closest_msg.has_value()) {
  RCLCPP_WARN(logger_, "No message found within tolerance");
  return;
}
```

## Troubleshooting

### "No synchronized data available"

- Check that input bag contains all required topics
- Verify sync tolerance is appropriate for your data rate
- Ensure buffer duration is sufficient for your evaluation interval

### Memory Issues with Large Bags

- Reduce `buffer_duration_sec` in config
- Decrease `max_buffer_messages` limit
- Process bag in smaller time chunks

### Time Synchronization Failures

- Increase `sync_tolerance_ms` if sensor timing is irregular
- Check that bag was recorded with proper time synchronization
- Verify all message headers have valid timestamps

## Code Style and Standards

- **C++ Standard**: C++17
- **Formatting**: clang-format (Autoware style)
- **Linting**: clang-tidy, autoware_lint_common
- **Namespaces**: All code in `autoware::trajectory_selector::offline_evaluation_tools`
- **File Extensions**: `.hpp` for headers, `.cpp` for implementation
- **Test Naming**: `test_*.cpp` pattern

Run pre-commit hooks before committing:
```bash
pre-commit run --all-files
```
