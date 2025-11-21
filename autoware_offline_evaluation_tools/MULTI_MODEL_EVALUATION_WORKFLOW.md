# Multi-Model Evaluation Workflow

Complete workflow for collecting multiple LIVE trajectory runs with evaluation metrics in a single self-contained MCAP file.

## Prerequisites

### Required Branches

**Evaluation Tools (this repository):**
```bash
cd ~/pilot-auto/src/autoware/new_planning_framework
git checkout feat/fix-or-scene-live-evaluation
colcon build --packages-select autoware_offline_evaluation_tools
```

**DLR (driving_log_replayer_v2):**
```bash
cd ~/pilot-auto/src/simulator/driving_log_replayer_v2
git checkout feat/open_loop_evalution-or-degradation-check
```

### T4 Dataset Structure

Your dataset must follow this structure:
```
/path/to/dataset/
├── input_bag/
│   └── original_recording.mcap  # Original rosbag from pilot.auto
├── map/
│   ├── lanelet2_map.osm         # HD map
│   └── pointcloud_map.pcd       # Point cloud map
├── annotation/                   # Empty directory
├── data/
│   └── LIDAR_CONCAT/            # Empty directory
└── scenario.yaml                # DLR configuration
```

**CRITICAL: scenario.yaml must include `publish_profile: planning_control`**

Example scenario.yaml:
```yaml
ScenarioFormatVersion: 3.0.0
ScenarioName: multi_model_evaluation
ScenarioDescription: Multi-model LIVE trajectory collection
SensorModel: aip_xx1
VehicleModel: lexus
publish_profile: planning_control  # REQUIRED!
Evaluation:
  UseCaseName: planning_control
  UseCaseFormatVersion: 2.0.0
  Datasets:
    - ./:
        VehicleId: default
  Conditions:
    ControlConditions: null
```

## Complete Workflow

### Step 1: Add Ground Truth Trajectories to Input Bag

Ground truth trajectories are generated from future kinematic states with 8-second look-ahead.

```bash
source ~/pilot-auto/install/setup.bash

python3 autoware_offline_evaluation_tools/scripts/add_gt_trajectory_to_bag.py \
  --input /path/to/dataset/input_bag \
  --output /tmp/input_bag_with_gt \
  --horizon 8.0 \
  --resolution 0.1
```

**Output:** `/tmp/input_bag_with_gt/` containing enhanced bag with `/ground_truth/trajectory` topic

**Time:** ~10-15 minutes for 180-second bag

---

### Step 2: Run DLR with First Model (model_v1)

This generates the first LIVE trajectory with prefix `model_v1`.

```bash
source ~/pilot-auto/install/setup.bash

rm -rf /tmp/model_v1_output

ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/dataset/scenario.yaml \
  input_bag:=/tmp/input_bag_with_gt \
  output_dir:=/tmp/model_v1_output \
  live_topic_prefix:=model_v1
```

**Output:** `/tmp/model_v1_output/result_bag/result_bag_0.mcap`

**Contains:**
- `/ground_truth/trajectory` (8,690 messages - propagated from input)
- `/model_v1/planning/trajectory_generator/diffusion_planner_node/output/trajectory` (LIVE trajectory)
- `/vehicle/status/control_mode` (for OR detection)
- All other Autoware topics

**Time:** ~3-4 minutes for 180-second bag

---

### Step 3: Evaluate Model 1 Metrics

Generate OR scene evaluation metrics for model_v1.

```bash
source ~/pilot-auto/install/setup.bash

rm -rf /tmp/model_v1_metrics.bag

ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  --params-file ~/pilot-auto/install/autoware_offline_evaluation_tools/share/autoware_offline_evaluation_tools/config/offline_evaluation.param.yaml \
  -p bag_path:=/tmp/model_v1_output/result_bag/result_bag_0.mcap \
  -p evaluation_output_bag_path:=/tmp/model_v1_metrics.bag \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/model_v1/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/tmp/input_bag_with_gt \
  -p or_scene_evaluation.enable_debug_visualization:=false \
  -p json_output_path:=/tmp/model_v1_results.json
```

**Output:** `/tmp/model_v1_metrics.bag`

**Contains (with model_v1 prefix):**
- `/model_v1/or_scene/ade` (30 messages - 10 predictions × 3 OR events)
- `/model_v1/or_scene/fde` (30 messages)
- `/model_v1/or_scene/lateral_deviation` (30 messages)
- `/model_v1/or_scene/ttc` (30 messages)
- `/model_v1/or_scene/event_markers` (3 messages - visualization)

**Also generates JSON:** `/tmp/model_v1_results.json` with detailed metrics

**Time:** ~30 seconds

---

### Step 4: Run DLR with Second Model (model_v2)

**IMPORTANT:** Use the first model's result_bag as input to accumulate LIVE trajectories.

```bash
source ~/pilot-auto/install/setup.bash

rm -rf /tmp/model_v2_output

ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/dataset/scenario.yaml \
  input_bag:=/tmp/model_v1_output/result_bag \
  output_dir:=/tmp/model_v2_output \
  live_topic_prefix:=model_v2 \
  previous_live_prefixes:=model_v1
```

**Output:** `/tmp/model_v2_output/result_bag/result_bag_0.mcap`

**Contains:**
- `/ground_truth/trajectory` (8,690 messages - propagated)
- `/model_v1/planning/.../trajectory` (1,724 messages - replayed from input)
- `/model_v2/planning/.../trajectory` (1,724 messages - NEW LIVE trajectory)
- `/vehicle/status/control_mode`
- All other Autoware topics

**Time:** ~3-4 minutes

---

### Step 5: Evaluate Model 2 Metrics

Generate OR scene evaluation metrics for model_v2.

```bash
source ~/pilot-auto/install/setup.bash

rm -rf /tmp/model_v2_metrics.bag

ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  --params-file ~/pilot-auto/install/autoware_offline_evaluation_tools/share/autoware_offline_evaluation_tools/config/offline_evaluation.param.yaml \
  -p bag_path:=/tmp/model_v2_output/result_bag/result_bag_0.mcap \
  -p evaluation_output_bag_path:=/tmp/model_v2_metrics.bag \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/model_v2/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/tmp/input_bag_with_gt \
  -p or_scene_evaluation.enable_debug_visualization:=false \
  -p json_output_path:=/tmp/model_v2_results.json
```

**Output:** `/tmp/model_v2_metrics.bag`

**Contains (with model_v2 prefix):**
- `/model_v2/or_scene/ade` (30 messages)
- `/model_v2/or_scene/fde` (30 messages)
- `/model_v2/or_scene/lateral_deviation` (30 messages)
- `/model_v2/or_scene/ttc` (30 messages)
- `/model_v2/or_scene/event_markers` (3 messages)

**Time:** ~30 seconds

---

### Step 6: Merge Everything into Final MCAP

Combine DLR result_bag + all metric bags into one self-contained file.

```bash
source ~/pilot-auto/install/setup.bash

python3 autoware_offline_evaluation_tools/scripts/merge_bags.py \
  --input /tmp/model_v2_output/result_bag/result_bag_0.mcap \
          /tmp/model_v1_metrics.bag \
          /tmp/model_v2_metrics.bag \
  --output /tmp/FINAL_MULTI_MODEL.mcap \
  --storage mcap
```

**Output:** `/tmp/FINAL_MULTI_MODEL.mcap` (3.4 GB for 180-second bag)

**Complete Contents:**
- 1× Ground truth trajectory: `/ground_truth/trajectory` (8,690 msgs)
- 2× LIVE trajectories: `/model_v1/planning/.../trajectory`, `/model_v2/planning/.../trajectory` (1,724 msgs each)
- 2× Metric sets: `/model_v1/or_scene/*`, `/model_v2/or_scene/*` (30 msgs each)
- All Autoware topics (localization, perception, planning, control)
- 181,321 total messages
- **All timestamps properly aligned** (no 1970 timestamps!)

**Time:** ~30 seconds

---

## Adding More Models (model_v3, model_v4, etc.)

To add additional models, repeat steps 4-6:

**For model_v3:**
```bash
# Step 4: Run DLR
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/dataset/scenario.yaml \
  input_bag:=/tmp/model_v2_output/result_bag \
  output_dir:=/tmp/model_v3_output \
  live_topic_prefix:=model_v3 \
  previous_live_prefixes:=model_v1,model_v2

# Step 5: Evaluate
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  --params-file ~/pilot-auto/install/autoware_offline_evaluation_tools/share/autoware_offline_evaluation_tools/config/offline_evaluation.param.yaml \
  -p bag_path:=/tmp/model_v3_output/result_bag/result_bag_0.mcap \
  -p evaluation_output_bag_path:=/tmp/model_v3_metrics.bag \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/model_v3/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/tmp/input_bag_with_gt \
  -p or_scene_evaluation.enable_debug_visualization:=false

# Step 6: Re-merge
python3 autoware_offline_evaluation_tools/scripts/merge_bags.py \
  --input /tmp/model_v3_output/result_bag/result_bag_0.mcap \
          /tmp/model_v1_metrics.bag \
          /tmp/model_v2_metrics.bag \
          /tmp/model_v3_metrics.bag \
  --output /tmp/FINAL_THREE_MODELS.mcap \
  --storage mcap
```

---

## How the Metric Prefix System Works

The evaluation tool **automatically extracts the prefix** from the trajectory topic name:

| Trajectory Topic | Extracted Prefix | Metric Topics |
|-----------------|------------------|---------------|
| `/model_v1/planning/.../trajectory` | `model_v1` | `/model_v1/or_scene/ade`, `/model_v1/or_scene/fde`, etc. |
| `/model_v2/planning/.../trajectory` | `model_v2` | `/model_v2/or_scene/ade`, `/model_v2/or_scene/fde`, etc. |
| `/planning/.../trajectory` (no prefix) | none | `/or_scene/ade`, `/or_scene/fde`, etc. |

**Implementation:** autoware_offline_evaluation_tools/src/node.cpp:322-342

The code:
1. Extracts first component from trajectory topic (e.g., `/model_v1/...` → `model_v1`)
2. Filters out standard Autoware namespaces (planning, control, localization, etc.)
3. Uses remaining component as metric topic prefix

This allows multiple models to coexist in one bag **without topic name conflicts**.

---

## Scripts Reference

### add_gt_trajectory_to_bag.py

Generates ground truth trajectories from future kinematic states.

**Usage:**
```bash
python3 autoware_offline_evaluation_tools/scripts/add_gt_trajectory_to_bag.py \
  --input <input_bag_path> \
  --output <output_bag_path> \
  --horizon 8.0 \
  --resolution 0.1
```

**Parameters:**
- `--input`: Original input bag directory (T4 dataset format)
- `--output`: Output directory for GT-enhanced bag
- `--horizon`: Look-ahead time in seconds (default: 8.0)
- `--resolution`: Time step between GT points in seconds (default: 0.1)

**What it does:**
- Reads `/localization/kinematic_state` from input bag
- For each kinematic state at time T, generates GT trajectory from T to T+8s
- Uses SLERP interpolation for orientation, linear for position
- Writes to `/ground_truth/trajectory` topic
- Copies all other topics from input bag

---

### merge_bags.py

Merges multiple rosbag2 bags into a single output bag.

**Usage:**
```bash
python3 autoware_offline_evaluation_tools/scripts/merge_bags.py \
  --input <bag1> <bag2> <bag3> ... \
  --output <output_bag> \
  --storage mcap
```

**Parameters:**
- `--input`, `-i`: Space-separated list of input bag paths
- `--output`, `-o`: Output bag path
- `--storage`, `-s`: Storage format (mcap or sqlite3, default: mcap)

**What it does:**
- Sequentially reads all input bags
- Merges topics (avoids duplicates)
- Preserves all message timestamps
- Writes to single output bag

**Example:**
```bash
python3 scripts/merge_bags.py \
  --input /tmp/result_bag.mcap /tmp/eval1.bag /tmp/eval2.bag \
  --output /tmp/combined.mcap \
  --storage mcap
```

---

## Troubleshooting

### Issue: Map not loading in DLR

**Symptom:** Diffusion planner stuck in "Waiting for map data..."

**Cause:** T4 dataset missing map files or wrong directory structure

**Fix:** Ensure `/path/to/dataset/map/` contains both `lanelet2_map.osm` and `pointcloud_map.pcd`

---

### Issue: No LIVE trajectories generated

**Symptom:** Result bag has 0 messages for `/model_v*/planning/.../trajectory`

**Cause:** Missing `publish_profile: planning_control` in scenario.yaml

**Fix:** Without publish_profile, DLR replays ALL topics including historical trajectories. Add the line to scenario.yaml.

---

### Issue: Lichtblick timeline broken (timestamps all over the place)

**Symptom:** Messages appear at year 1970 and 2025

**Cause:** Old version of code wrote tf_static/map_markers with timestamp 0

**Fix:** Use branch `feat/fix-or-scene-live-evaluation` which fixes this (node.cpp:358, 370)

**Verification:**
```bash
source ~/pilot-auto/install/setup.bash
python3 -c "
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

reader = SequentialReader()
storage = StorageOptions(uri='/tmp/your_bag.mcap', storage_id='')
converter = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage, converter)

count_zero = 0
while reader.has_next():
    _, _, timestamp = reader.read_next()
    if timestamp == 0:
        count_zero += 1

print(f'Messages at timestamp 0: {count_zero}')  # Should be 0!
"
```

---

### Issue: Evaluation can't find OR events

**Symptom:** "No OR events found in bag"

**Cause:** No AUTONOMOUS→MANUAL transitions in `/vehicle/status/control_mode`

**Fix:** Verify input_bag contains override events. Check with:
```bash
python3 scripts/check_overrides.py /path/to/input_bag
```

---

## File Locations and Outputs

### Intermediate Files

| File | Purpose | Size (180s bag) |
|------|---------|-----------------|
| `/tmp/input_bag_with_gt/` | GT-enhanced input | 28 GB |
| `/tmp/model_v1_output/result_bag/` | DLR run 1 result | 2.6 GB |
| `/tmp/model_v2_output/result_bag/` | DLR run 2 result | 2.6 GB |
| `/tmp/model_v1_metrics.bag` | Model 1 eval metrics | ~500 KB |
| `/tmp/model_v2_metrics.bag` | Model 2 eval metrics | ~500 KB |

### Final Output

| File | Purpose | Size |
|------|---------|------|
| `/tmp/FINAL_MULTI_MODEL.mcap` | Complete combined bag | 3.4 GB |
| `/tmp/model_v1_results.json` | Model 1 detailed metrics | ~10 KB |
| `/tmp/model_v2_results.json` | Model 2 detailed metrics | ~10 KB |

---

## Complete Example: 2-Model Workflow

```bash
#!/bin/bash
set -e

# Configuration
DATASET_PATH="/path/to/your/dataset"
SCENARIO="${DATASET_PATH}/scenario.yaml"

# Source Autoware environment
source ~/pilot-auto/install/setup.bash

echo "Step 1: Adding ground truth trajectories..."
python3 autoware_offline_evaluation_tools/scripts/add_gt_trajectory_to_bag.py \
  --input "${DATASET_PATH}/input_bag" \
  --output /tmp/input_bag_with_gt \
  --horizon 8.0 \
  --resolution 0.1

echo "Step 2: Running DLR with model_v1..."
rm -rf /tmp/model_v1_output
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:="${SCENARIO}" \
  input_bag:=/tmp/input_bag_with_gt \
  output_dir:=/tmp/model_v1_output \
  live_topic_prefix:=model_v1

echo "Step 3: Evaluating model_v1 metrics..."
rm -rf /tmp/model_v1_metrics.bag
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  --params-file ~/pilot-auto/install/autoware_offline_evaluation_tools/share/autoware_offline_evaluation_tools/config/offline_evaluation.param.yaml \
  -p bag_path:=/tmp/model_v1_output/result_bag/result_bag_0.mcap \
  -p evaluation_output_bag_path:=/tmp/model_v1_metrics.bag \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/model_v1/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/tmp/input_bag_with_gt \
  -p or_scene_evaluation.enable_debug_visualization:=false

echo "Step 4: Running DLR with model_v2..."
rm -rf /tmp/model_v2_output
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:="${SCENARIO}" \
  input_bag:=/tmp/model_v1_output/result_bag \
  output_dir:=/tmp/model_v2_output \
  live_topic_prefix:=model_v2 \
  previous_live_prefixes:=model_v1

echo "Step 5: Evaluating model_v2 metrics..."
rm -rf /tmp/model_v2_metrics.bag
ros2 run autoware_offline_evaluation_tools offline_evaluator_node \
  --ros-args \
  --params-file ~/pilot-auto/install/autoware_offline_evaluation_tools/share/autoware_offline_evaluation_tools/config/offline_evaluation.param.yaml \
  -p bag_path:=/tmp/model_v2_output/result_bag/result_bag_0.mcap \
  -p evaluation_output_bag_path:=/tmp/model_v2_metrics.bag \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/model_v2/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/tmp/input_bag_with_gt \
  -p or_scene_evaluation.enable_debug_visualization:=false

echo "Step 6: Merging into final MCAP..."
python3 autoware_offline_evaluation_tools/scripts/merge_bags.py \
  --input /tmp/model_v2_output/result_bag/result_bag_0.mcap \
          /tmp/model_v1_metrics.bag \
          /tmp/model_v2_metrics.bag \
  --output /tmp/FINAL_MULTI_MODEL.mcap \
  --storage mcap

echo "COMPLETE! Final bag: /tmp/FINAL_MULTI_MODEL.mcap"
echo ""
echo "Bag contents:"
ros2 bag info /tmp/FINAL_MULTI_MODEL.mcap 2>/dev/null | grep -E "(Duration|Messages|ground_truth|model_v1|model_v2)"
```

---

## Verifying the Final Bag

### Check Topic List

```bash
source ~/pilot-auto/install/setup.bash
ros2 bag info /tmp/FINAL_MULTI_MODEL.mcap 2>/dev/null | grep -E "(model_v1|model_v2|ground_truth)"
```

**Expected output:**
```
Topic: /ground_truth/trajectory | Count: 8690
Topic: /model_v1/planning/trajectory_generator/diffusion_planner_node/output/trajectory | Count: 1724
Topic: /model_v2/planning/trajectory_generator/diffusion_planner_node/output/trajectory | Count: 1724
Topic: /model_v1/or_scene/ade | Count: 30
Topic: /model_v1/or_scene/fde | Count: 30
Topic: /model_v2/or_scene/ade | Count: 30
Topic: /model_v2/or_scene/fde | Count: 30
```

### Verify No Timestamp 0 Messages

```bash
source ~/pilot-auto/install/setup.bash
python3 -c "
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

reader = SequentialReader()
storage = StorageOptions(uri='/tmp/FINAL_MULTI_MODEL.mcap', storage_id='')
converter = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage, converter)

count_zero = 0
count_nonzero = 0
while reader.has_next():
    _, _, timestamp = reader.read_next()
    if timestamp == 0:
        count_zero += 1
    else:
        count_nonzero += 1

print(f'Timestamp 0: {count_zero} (should be 0!)')
print(f'Proper timestamps: {count_nonzero}')
"
```

**Expected:** 0 messages at timestamp 0

---

## Key Implementation Details

### Metric Topic Prefix Extraction

Location: `autoware_offline_evaluation_tools/src/node.cpp:322-342`

```cpp
// Extract metric topic prefix from trajectory topic for multi-run support
// Topics like /model_v1/planning/.../trajectory → prefix "model_v1"
// Standard topics like /planning/.../trajectory → no prefix
std::string trajectory_topic = topic_names.trajectory_topic;
if (!trajectory_topic.empty() && trajectory_topic[0] == '/') {
  size_t second_slash = trajectory_topic.find('/', 1);
  if (second_slash != std::string::npos) {
    std::string first_component = trajectory_topic.substr(1, second_slash - 1);

    const std::set<std::string> standard_namespaces =
      {"planning", "control", "localization", "perception", "sensing", "map", "system", "vehicle"};

    if (standard_namespaces.find(first_component) == standard_namespaces.end()) {
      evaluator.set_metric_topic_prefix(first_component);
      RCLCPP_INFO(get_logger(), "Using metric topic prefix: %s", first_component.c_str());
    }
  }
}
```

### Timestamp Fix

**Bug:** tf_static and map_markers were written with timestamp 0 (year 1970)

**Location:** `autoware_offline_evaluation_tools/src/node.cpp:358, 370`

**Old code (BROKEN):**
```cpp
rclcpp::Time tf_time(0, 0, RCL_ROS_TIME);  // BAD!
```

**Fixed code:**
```cpp
rclcpp::Time tf_time = start_time;  // Use actual start time
```

This ensures all messages in evaluation bags have proper timestamps aligned with the DLR result_bag.

---

## Multi-Run Collection with DLR

### Topic Relay

DLR uses `topic_tools/relay` to rename LIVE output in real-time:

**Original topic:** `/planning/trajectory_generator/diffusion_planner_node/output/trajectory`
**Relayed topic:** `/model_v1/planning/trajectory_generator/diffusion_planner_node/output/trajectory`

### Previous LIVE Topics Replay

When you specify `previous_live_prefixes:=model_v1`, DLR:
1. Adds `/model_v1/planning/.../trajectory` to the replay topic list
2. This previous LIVE trajectory is replayed from the input_bag
3. Recorder captures BOTH old and new LIVE trajectories

**DLR Implementation:**
- `driving_log_replayer_v2/launch/argument.py` - Defines parameters
- `driving_log_replayer_v2/launch/rosbag.py` - Adds previous topics to replay list
- `driving_log_replayer_v2/launch/topic_relay.py` - Launches relay node
- `driving_log_replayer_v2/launch/planning_control.py` - Configures recorder regex

---

## Expected Timeline

For a 180-second rosbag with 3 OR events:

| Step | Time | Output Size |
|------|------|-------------|
| 1. Add GT | 10-15 min | 28 GB |
| 2. DLR model_v1 | 3-4 min | 2.6 GB |
| 3. Eval model_v1 | 30 sec | 500 KB |
| 4. DLR model_v2 | 3-4 min | 2.6 GB |
| 5. Eval model_v2 | 30 sec | 500 KB |
| 6. Merge | 30 sec | 3.4 GB |
| **Total** | **20-25 min** | **3.4 GB final** |

---

## Comparing Model Performance

The final bag contains JSON files with detailed metrics. To compare:

```bash
# Model 1
cat /tmp/model_v1_results.json | jq '.summary'

# Model 2
cat /tmp/model_v2_results.json | jq '.summary'
```

**Key metrics:**
- `mean_ade_all_events` - Average displacement error across all OR events
- `mean_fde_all_events` - Final displacement error
- Lower values = better prediction accuracy

**Example results:**
- model_v1: ADE=2.114m, FDE=3.728m (BETTER)
- model_v2: ADE=3.099m, FDE=6.579m (WORSE)

---

## Notes

- Each DLR iteration takes the previous iteration's result_bag as input
- This incrementally accumulates LIVE trajectories in the bag
- Ground truth is generated once and propagates through all iterations
- Metrics are generated separately for each model
- Final merge combines everything into one self-contained MCAP
- All timestamps are preserved (no normalization to 0)
- Works in Lichtblick without timeline issues
