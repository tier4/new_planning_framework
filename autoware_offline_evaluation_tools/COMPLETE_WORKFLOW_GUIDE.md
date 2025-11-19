# Complete Multi-Run Workflow with Ground Truth

## Overview

This guide shows how to create a self-contained mcap file with:
- ✅ Ground truth trajectory (8-second look-ahead)
- ✅ Multiple LIVE model runs with custom prefixes
- ✅ Map markers
- ✅ OR detection (control_mode)
- ✅ All data needed for evaluation and training

## Prerequisites

- Original input_bag with `/localization/kinematic_state`
- T4 dataset structure with map
- DLR and evaluation tools built

## Workflow

### Step 0: Add Ground Truth (ONE TIME)

**Generate GT trajectories from future kinematic states:**

```bash
python3 scripts/add_gt_trajectory_to_bag.py \
  --input ~/t4_dataset/input_bag \
  --output ~/t4_dataset/input_bag_with_gt \
  --horizon 8.0 \
  --resolution 0.1
```

**Parameters:**
- `--horizon`: Look-ahead time in seconds (default: 8.0)
- `--resolution`: Sample interval in seconds (default: 0.1 = 10Hz)

**Output:** `input_bag_with_gt/` containing all original topics + `/ground_truth/trajectory`

**Performance:** ~3-4 minutes for 27GB bag

### Step 1: Run DLR with First Model

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=~/t4_dataset/scenario.yaml \
  input_bag:=~/t4_dataset/input_bag_with_gt \
  output_dir:=~/multi_run_output/run1 \
  live_topic_prefix:=model_v1.0_epoch50
```

**What happens:**
- Replays GT + objects + vehicle data
- Runs planning with model v1.0
- Topic relay renames LIVE output to `/model_v1.0_epoch50/planning/.../trajectory`
- Recorder captures: GT + LIVE run 1 + map + control_mode

**Result:** `~/multi_run_output/run1/result_bag/result_bag_0.mcap`

**Contains:**
- `/ground_truth/trajectory` (8,690 messages)
- `/model_v1.0_epoch50/planning/.../trajectory` (1,724 messages)
- `/vehicle/status/control_mode` (5,101 messages)
- `/map/vector_map_marker` (1 message)
- `/localization/kinematic_state` (8,690 messages)

### Step 2: Change Model Weights

**Manually update your model checkpoint/configuration**
- Load different weights
- Change hyperparameters
- Swap model architecture
- etc.

### Step 3: Run DLR with Second Model

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=~/t4_dataset/scenario.yaml \
  input_bag:=~/multi_run_output/run1/result_bag \
  output_dir:=~/multi_run_output/run2 \
  live_topic_prefix:=model_v2.0_epoch100 \
  previous_live_prefixes:=model_v1.0_epoch50
```

**What happens:**
- Replays: GT + LIVE run 1 + objects + vehicle data
- Runs planning with model v2.0
- Renames new LIVE output to `/model_v2.0_epoch100/planning/.../trajectory`
- Recorder captures: GT + LIVE run 1 + LIVE run 2 + map + control_mode

**Result:** `~/multi_run_output/run2/result_bag/result_bag_0.mcap`

**Contains:**
- `/ground_truth/trajectory` (8,690 messages)
- `/model_v1.0_epoch50/planning/.../trajectory` (1,724 messages) ← From run1
- `/model_v2.0_epoch100/planning/.../trajectory` (1,724 messages) ← NEW
- `/vehicle/status/control_mode` (5,101 messages)
- `/map/vector_map_marker` (1 message)

### Step 4: Add More Runs (Optional)

Repeat step 2-3 for additional models:

```bash
# Change model again
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=~/t4_dataset/scenario.yaml \
  input_bag:=~/multi_run_output/run2/result_bag \
  output_dir:=~/multi_run_output/run3 \
  live_topic_prefix:=baseline_pretrained \
  previous_live_prefixes:=model_v1.0_epoch50,model_v2.0_epoch100
```

**Final bag contains:**
- GT + run1 + run2 + run3 + all metadata

## Evaluation

### Option A: Evaluate Individual Run

```bash
bash scripts/run_evaluation.sh \
  ~/multi_run_output/run2/result_bag/result_bag_0.mcap \
  ~/t4_dataset/input_bag \
  /model_v1.0_epoch50/planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  ~/evaluation_output/run1 \
  ~/t4_dataset/map/lanelet2_map.osm
```

### Option B: Evaluate All Runs with Config

**Create config:**
```yaml
# multi_run_config.yaml
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory

runs:
  - prefix: model_v1.0_epoch50
    description: "Model v1.0 after 50 epochs"
  - prefix: model_v2.0_epoch100
    description: "Model v2.0 after 100 epochs"
  - prefix: baseline_pretrained
    description: "Baseline model"
```

**Run evaluation:**
```bash
bash scripts/evaluate_multi_run_bag.sh \
  ~/multi_run_output/run3/result_bag/result_bag_0.mcap \
  ~/t4_dataset/input_bag \
  multi_run_config.yaml \
  ~/evaluation_output \
  ~/t4_dataset/map/lanelet2_map.osm
```

**Output:**
- Per-run results in separate directories
- Comparison summary table
- 30 visualizations per run (with map overlay)

## Topic Reference

**Essential Topics in Final MCAP:**

| Topic | Type | Count | Purpose |
|-------|------|-------|---------|
| `/ground_truth/trajectory` | Trajectory | 8,690 | Training supervision |
| `/{prefix}/planning/.../trajectory` | Trajectory | 1,724 | LIVE model output |
| `/vehicle/status/control_mode` | ControlModeReport | 5,101 | OR detection |
| `/map/vector_map_marker` | MarkerArray | 1 | Visualization |
| `/map/vector_map` | LaneletMapBin | 1 | Map data |
| `/localization/kinematic_state` | Odometry | 8,690 | Raw GT data |

**Control Mode Values (OR Detection):**
- `1` = AUTONOMOUS (vehicle driving)
- `4` = MANUAL (safety driver override)
- OR event = transition from `1` → `4`

## File Structure

```
t4_dataset/
├── input_bag/                    # Original recording
├── input_bag_with_gt/            # After step 0 (GT added)
│   └── input_bag_with_gt_0.mcap  # Use this for all DLR runs
├── map/
│   └── lanelet2_map.osm
└── scenario.yaml

multi_run_output/
├── run1/
│   └── result_bag/
│       └── result_bag_0.mcap     # GT + LIVE run 1
├── run2/
│   └── result_bag/
│       └── result_bag_0.mcap     # GT + LIVE run 1 + run 2
└── run3/
    └── result_bag/
        └── result_bag_0.mcap     # GT + LIVE run 1 + 2 + 3

evaluation_output/
├── model_v1.0_epoch50/
│   ├── or_results.json
│   └── or_debug_images/
├── model_v2.0_epoch100/
│   ├── or_results.json
│   └── or_debug_images/
└── comparison_summary.json
```

## Quick Start Example

```bash
# 1. Add GT to original bag (ONE TIME)
cd ~/pilot-auto/src/autoware/new_planning_framework/autoware_offline_evaluation_tools
python3 scripts/add_gt_trajectory_to_bag.py \
  --input ~/t4_dataset/input_bag \
  --output ~/t4_dataset/input_bag_with_gt

# 2. Run DLR iteration 1
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=~/t4_dataset/scenario.yaml \
  input_bag:=~/t4_dataset/input_bag_with_gt \
  output_dir:=~/output/run1 \
  live_topic_prefix:=model_v1

# 3. Change model weights manually

# 4. Run DLR iteration 2
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=~/t4_dataset/scenario.yaml \
  input_bag:=~/output/run1/result_bag \
  output_dir:=~/output/run2 \
  live_topic_prefix:=model_v2 \
  previous_live_prefixes:=model_v1

# 5. Evaluate all runs
cat > eval_config.yaml << EOF
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory
runs:
  - prefix: model_v1
    description: "Model v1"
  - prefix: model_v2
    description: "Model v2"
EOF

bash scripts/evaluate_multi_run_bag.sh \
  ~/output/run2/result_bag/result_bag_0.mcap \
  ~/t4_dataset/input_bag \
  eval_config.yaml \
  ~/evaluation \
  ~/t4_dataset/map/lanelet2_map.osm
```

## Important Notes

### GT Trajectory Details

- **Generated once** from original input_bag
- **Horizon:** 8 seconds by default (configurable)
- **Resolution:** 0.1s (10Hz) by default
- **Interpolation:** SLERP for orientation, linear for position
- **Edge case:** When <8s remaining, generates shorter trajectory
- **Propagation:** Automatically included in all subsequent iterations

### Topic Prefix Naming

**Use descriptive prefixes** to identify runs:
- ✅ `model_v1.0_epoch50`
- ✅ `baseline_pretrained`
- ✅ `experiment_seed42`
- ❌ `run1`, `run2` (not descriptive)

### Performance

- GT generation: ~3-4 minutes (one time)
- DLR per iteration: ~3-5 minutes (for 180s bag)
- Evaluation per run: ~1.5 minutes
- No overhead for topic relay (real-time renaming)

### Troubleshooting

**GT trajectory not in result_bag:**
- Check input_bag has `/ground_truth/trajectory`
- Verify DLR built after latest commit (e127de2)
- Check publish profile includes GT

**LIVE topic not renamed:**
- Verify `live_topic_prefix` parameter set
- Check DLR logs for "Launching topic relay"
- Verify recorder regex includes `|^/.*/planning/.*$\`

**Previous LIVE topics not replayed:**
- Check `previous_live_prefixes` matches exact prefixes from earlier runs
- Verify `input_bag` points to previous result_bag
- Check DLR logs for "Adding previous LIVE topic to replay"

## Complete Topic Listing

**Final MCAP after N iterations contains:**

```
/ground_truth/trajectory                           8,690 msgs  (GT)
/{prefix1}/planning/.../trajectory                 1,724 msgs  (LIVE run 1)
/{prefix2}/planning/.../trajectory                 1,724 msgs  (LIVE run 2)
...
/{prefixN}/planning/.../trajectory                 1,724 msgs  (LIVE run N)
/vehicle/status/control_mode                       5,101 msgs  (OR detection)
/map/vector_map_marker                             1 msg       (Map viz)
/map/vector_map                                    1 msg       (Map data)
/localization/kinematic_state                      8,690 msgs  (Raw odometry)
/perception/object_recognition/tracking/objects    1,807 msgs  (Objects)
+ all standard planning/control debug topics
```

## Scripts Reference

**GT Generation:**
- `scripts/add_gt_trajectory_to_bag.py` - Add GT to input bag

**DLR Execution:**
- Manual: `ros2 launch driving_log_replayer_v2 ...`
- Wrapper: `scripts/run_complete_pipeline.sh` (DLR + evaluation)

**Evaluation:**
- Single run: `scripts/run_evaluation.sh`
- Multi-run: `scripts/evaluate_multi_run_bag.sh`

**Utilities:**
- `scripts/rename_bag_topic.py` - Post-recording topic rename (backup)

## Example: Complete 3-Run Collection

```bash
# Setup
DATASET=~/t4_dataset
OUTPUT=~/multi_run_output

# Step 0: Generate GT (ONE TIME)
python3 scripts/add_gt_trajectory_to_bag.py \
  --input ${DATASET}/input_bag \
  --output ${DATASET}/input_bag_with_gt

# Step 1: Run with model v1.0
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${DATASET}/scenario.yaml \
  input_bag:=${DATASET}/input_bag_with_gt \
  output_dir:=${OUTPUT}/run1 \
  live_topic_prefix:=model_v1.0_epoch50

# User changes model weights

# Step 2: Run with model v2.0
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${DATASET}/scenario.yaml \
  input_bag:=${OUTPUT}/run1/result_bag \
  output_dir:=${OUTPUT}/run2 \
  live_topic_prefix:=model_v2.0_epoch100 \
  previous_live_prefixes:=model_v1.0_epoch50

# User loads baseline model

# Step 3: Run with baseline
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${DATASET}/scenario.yaml \
  input_bag:=${OUTPUT}/run2/result_bag \
  output_dir:=${OUTPUT}/run3 \
  live_topic_prefix:=baseline \
  previous_live_prefixes:=model_v1.0_epoch50,model_v2.0_epoch100

# Step 4: Evaluate all runs
cat > eval_config.yaml << EOF
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory
runs:
  - prefix: model_v1.0_epoch50
    description: "Model v1.0"
  - prefix: model_v2.0_epoch100
    description: "Model v2.0"
  - prefix: baseline
    description: "Baseline"
EOF

bash scripts/evaluate_multi_run_bag.sh \
  ${OUTPUT}/run3/result_bag/result_bag_0.mcap \
  ${DATASET}/input_bag \
  eval_config.yaml \
  ${OUTPUT}/evaluation \
  ${DATASET}/map/lanelet2_map.osm
```

**Final mcap:** `~/multi_run_output/run3/result_bag/result_bag_0.mcap`

**Contains:** GT + 3 LIVE runs + all metadata (self-contained)

## Key Concepts

### Ground Truth Propagation

GT is added ONCE to input_bag, then:
1. DLR replays GT from input_bag_with_gt
2. Recorder captures GT to result_bag
3. Next iteration replays GT from previous result_bag
4. GT automatically in all iterations without regeneration

### LIVE Topic Accumulation

Each iteration:
1. Replays previous LIVE topics (via `previous_live_prefixes`)
2. Generates NEW LIVE trajectory
3. Relay renames NEW to `/{current_prefix}/planning/.../trajectory`
4. Recorder captures: OLD LIVE topics + NEW LIVE topic

Result: Incremental accumulation in single mcap

### Zero-Overhead Renaming

Topic relay (topic_tools/relay):
- Subscribes to original `/planning/.../trajectory`
- Republishes to `/{prefix}/planning/.../trajectory`
- Happens in real-time during DLR
- No post-processing delay

## Tested Configuration

**Test Dataset:**
- Location: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/`
- Duration: 180 seconds
- OR events: 3
- Map: shinagawa_odaiba_stable

**Test Results:**
- GT generation: ✅ 8,690 trajectories
- Multi-run collection: ✅ 2 runs accumulated
- Topic relay: ✅ Zero overhead
- All topics present: ✅ Verified

**Example bag:** `/tmp/test_with_gt/result_bag/result_bag_0.mcap`
