# Multi-Run LIVE Trajectory Collection - User Guide

## Overview

This feature allows you to collect multiple LIVE trajectory outputs from different model versions/configurations into a single mcap file for comparison against ground truth.

## Workflow

### Step 1: Run DLR with First Model

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  output_dir:=/path/to/multi_run/run1 \
  live_topic_prefix:=model_v1.0_epoch50
```

**What happens:**
- DLR runs normally, generates LIVE trajectories
- After recording completes, automatically renames:
  - `/planning/trajectory_generator/diffusion_planner_node/output/trajectory`
  - → `/model_v1.0_epoch50/planning/trajectory_generator/diffusion_planner_node/output/trajectory`
- Output: `run1/result_bag/result_bag_0.mcap` with renamed topic

### Step 2: Change Model and Run DLR Again

**Manually:** Load different model checkpoint/weights in your planning node configuration

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  input_bag:=/path/to/multi_run/run1/result_bag \
  output_dir:=/path/to/multi_run/run2 \
  live_topic_prefix:=model_v2.0_epoch100 \
  previous_live_prefixes:=model_v1.0_epoch50
```

**What happens:**
- DLR replays: GT + objects + `/model_v1.0_epoch50/.../trajectory` (from run1)
- Generates NEW LIVE trajectory with current model
- Renames new output to: `/model_v2.0_epoch100/.../trajectory`
- Output: `run2/result_bag/result_bag_0.mcap` with BOTH trajectories

### Step 3: Add More Runs (Optional)

Repeat step 2 with additional models:

```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  input_bag:=/path/to/multi_run/run2/result_bag \
  output_dir:=/path/to/multi_run/run3 \
  live_topic_prefix:=baseline_pretrained \
  previous_live_prefixes:=model_v1.0_epoch50,model_v2.0_epoch100
```

**Final result:** `run3/result_bag/result_bag_0.mcap` contains:
- `/localization/kinematic_state` (ground truth)
- `/perception/object_recognition/tracking/objects`
- `/model_v1.0_epoch50/planning/.../trajectory`
- `/model_v2.0_epoch100/planning/.../trajectory`
- `/baseline_pretrained/planning/.../trajectory`

### Step 4: Create Evaluation Configuration

Create a YAML file listing all runs to evaluate:

```yaml
# eval_config.yaml
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory

runs:
  - prefix: model_v1.0_epoch50
    description: "First model checkpoint"

  - prefix: model_v2.0_epoch100
    description: "Improved model"

  - prefix: baseline_pretrained
    description: "Baseline for comparison"
```

### Step 5: Run Multi-Run Evaluation

```bash
./scripts/evaluate_multi_run_bag.sh \
  /path/to/multi_run/run3/result_bag/result_bag_0.mcap \
  /path/to/original_input_bag \
  /path/to/eval_config.yaml \
  /path/to/evaluation_output \
  /path/to/map/lanelet2_map.osm
```

**Output:**
```
evaluation_output/
├── model_v1.0_epoch50/
│   ├── or_results.json
│   └── or_debug_images/
├── model_v2.0_epoch100/
│   ├── or_results.json
│   └── or_debug_images/
├── baseline_pretrained/
│   ├── or_results.json
│   └── or_debug_images/
└── comparison_summary.json
```

**Comparison Output:**
```
================================================================================
Multi-Run Comparison Results (Sorted by Mean ADE)
================================================================================
Description                                  Prefix                     Mean ADE (m)         Mean FDE (m)
--------------------------------------------------------------------------------
Improved model                               model_v2.0_epoch100         1.856 ±0.523         3.245 ±1.234
First model checkpoint                       model_v1.0_epoch50          2.134 ±0.678         3.892 ±1.567
Baseline for comparison                      baseline_pretrained         2.445 ±0.812         4.123 ±1.890
================================================================================

Best Performing Run: Improved model
  Mean ADE: 1.856m (±0.523m)
  Mean FDE: 3.245m (±1.234m)

Improvement (best vs worst): 24.1%
```

## Parameters

### DLR Parameters

**`live_topic_prefix`** (optional, default: "")
- Descriptive prefix for current LIVE run
- Example: `model_v1.0_epoch50`, `baseline`, `experiment_seed42`
- Automatically renames trajectory topic after recording

**`base_trajectory_topic`** (optional, default: diffusion_planner output)
- Which trajectory topic to rename
- Change if using different planner (e.g., trajectory_optimizer)

**`previous_live_prefixes`** (optional, default: "")
- Comma-separated list of previous run prefixes
- Used for iterations 2+ to replay previous LIVE topics
- Example: `model_v1.0_epoch50,model_v2.0_epoch100`

## Complete Example

```bash
# Iteration 1
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/media/.../t4_dataset/scenario.yaml \
  output_dir:=/media/.../multi_run/run1 \
  live_topic_prefix:=model_v1.0_epoch50

# User changes model checkpoint

# Iteration 2
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/media/.../t4_dataset/scenario.yaml \
  input_bag:=/media/.../multi_run/run1/result_bag \
  output_dir:=/media/.../multi_run/run2 \
  live_topic_prefix:=model_v2.0_epoch100 \
  previous_live_prefixes:=model_v1.0_epoch50

# User loads baseline model

# Iteration 3
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/media/.../t4_dataset/scenario.yaml \
  input_bag:=/media/.../multi_run/run2/result_bag \
  output_dir:=/media/.../multi_run/run3 \
  live_topic_prefix:=baseline \
  previous_live_prefixes:=model_v1.0_epoch50,model_v2.0_epoch100

# Create eval config
cat > eval_config.yaml << 'EOF'
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory
runs:
  - prefix: model_v1.0_epoch50
    description: "Model v1.0"
  - prefix: model_v2.0_epoch100
    description: "Model v2.0"
  - prefix: baseline
    description: "Baseline"
EOF

# Evaluate all runs
./scripts/evaluate_multi_run_bag.sh \
  /media/.../multi_run/run3/result_bag/result_bag_0.mcap \
  /media/.../t4_dataset/input_bag \
  eval_config.yaml \
  /media/.../evaluation \
  /media/.../t4_dataset/map/lanelet2_map.osm
```

## Important Notes

### Topic Naming Convention

**Format:** `/{prefix}{base_trajectory_topic}`

**Examples:**
- Prefix: `model_v1.0`, Base: `/planning/.../trajectory`
  → Result: `/model_v1.0/planning/.../trajectory`

- Prefix: `baseline_seed42`, Base: `/planning/.../trajectory`
  → Result: `/baseline_seed42/planning/.../trajectory`

### Storage Requirements

Each iteration adds ~500MB-1GB to the bag (just the new trajectory):
- Run 1: ~2.5GB (GT + objects + 1 LIVE)
- Run 2: ~3.0GB (GT + objects + 2 LIVE)
- Run 3: ~3.5GB (GT + objects + 3 LIVE)

Store on SSD for best performance.

### Troubleshooting

**"Topic not found in bag"**
- Check that `base_trajectory_topic` matches your planner's output topic
- Use `ros2 bag info <bag>` to list all available topics

**"Previous LIVE topics not replayed"**
- Verify `previous_live_prefixes` exactly matches prefixes from previous runs
- Check DLR logs for "Adding previous LIVE topic to replay" message

**"Renaming failed"**
- Ensure `rename_bag_topic.py` is in `scripts/` directory
- Check disk space (renaming creates temporary copy)
- Verify Python has rosbag2_py installed

## Files Reference

**DLR:**
- `driving_log_replayer_v2/launch/argument.py` - Parameter definitions
- `driving_log_replayer_v2/launch/rosbag.py` - Replay of previous LIVE topics
- `driving_log_replayer_v2/launch/post_process.launch.py` - Automatic renaming

**Evaluation Tools:**
- `scripts/rename_bag_topic.py` - Topic renaming utility
- `scripts/evaluate_multi_run_bag.sh` - Multi-run evaluation wrapper
- `scripts/multi_run_evaluator.py` - Config-based evaluation orchestrator
- `config/example_multi_run_eval.yaml` - Example configuration file
