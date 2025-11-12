# Multi-Run LIVE Trajectory Collection - Implementation Plan

## Overview

Enable collecting multiple LIVE trajectory outputs from different model versions/configurations into a single mcap file for comparison against ground truth.

## Current Status

### Completed Work (Session 2025-11-11)

1. ✅ **Map Visualization Feature** - Added lanelet2 map overlay to debug images
   - Modified: `generate_or_visualization.py`, `or_scene_evaluator.{hpp,cpp}`, `node.cpp`
   - Parameter: `or_scene_evaluation.map_path`
   - Status: Fully working and tested

2. ✅ **Parameterized Evaluation Scripts**
   - Created: `run_dlr.sh`, `run_evaluation.sh`, `run_dlr_and_evaluate_fixed.sh`
   - Location: `autoware_offline_evaluation_tools/scripts/`
   - Documentation: `scripts/README.md`

3. ✅ **LIVE vs HISTORICAL Verification**
   - Confirmed DLR does NOT replay historical trajectories
   - `publish_profile: planning_control` only replays inputs (route, perception, localization)
   - LIVE trajectories generated fresh during DLR simulation

## Goal: Multi-Run Feature

### User Requirements

**Workflow:**
1. Run DLR iteration 1 with model v1.0
   - Records: GT + objects + `/prefix1/.../trajectory`
2. User manually changes model weights/config
3. Run DLR iteration 2 using result_bag_1 as input
   - Replays: GT + objects + `/prefix1/.../trajectory` (from previous run)
   - Records: All above + `/prefix2/.../trajectory` (NEW)
4. Repeat for N iterations
5. Final bag contains: GT + objects + N different LIVE trajectory topics
6. Evaluate all N trajectories against GT using config file

**Key Insight:** Each iteration builds on previous result_bag (incremental accumulation), not bag merging.

### Design Decisions

**Confirmed Requirements:**
1. ✅ Each iteration uses previous result_bag as input
2. ✅ Descriptive prefix names (e.g., "model_v1.0_epoch50", not "run_0")
3. ✅ Base trajectory topic parameterized (default: diffusion_planner output)
4. ✅ Manual iteration (user changes model between runs)
5. ✅ Evaluation uses YAML/JSON config to specify which topics to evaluate

**Not Needed:**
- ❌ Bag merging (incremental is cleaner)
- ❌ Multi-trajectory overlay visualization (not important for now)
- ❌ Automated iteration (manual model changes required)

## Implementation Approach

### Phase 1: DLR Topic Remapping

**Goal:** Add parameters to DLR for topic prefix management

**New DLR Parameters:**

1. **`live_topic_prefix`** (string, default: "")
   - Prefix for current run's LIVE trajectory output
   - Example: "model_v1.0_epoch50"
   - Result: `/{prefix}/planning/trajectory_generator/.../trajectory`

2. **`base_trajectory_topic`** (string, default: "/planning/trajectory_generator/diffusion_planner_node/output/trajectory")
   - Which trajectory topic to remap
   - Parameterized for different planners

3. **`previous_live_prefixes`** (comma-separated string, default: "")
   - List of previous run prefixes to replay
   - Example: "model_v1.0_epoch50,model_v2.0_epoch100"
   - Auto-converted to topic patterns for bag player

**Files to Modify:**
- `driving_log_replayer_v2/launch/argument.py` - Add 3 parameters
- `driving_log_replayer_v2/launch/rosbag.py` - Implement recorder remap + player topic inclusion

**Implementation Details:**

```python
# rosbag.py - launch_bag_recorder()
def launch_bag_recorder(context):
    conf = context.launch_configurations
    live_prefix = conf.get("live_topic_prefix", "")
    base_topic = conf.get("base_trajectory_topic", "<default>")

    record_cmd = [...]  # Existing setup

    # Add --remap for current LIVE output
    if live_prefix:
        remapped = f"/{live_prefix}{base_topic}"
        record_cmd += ["--remap", f"{base_topic}:={remapped}"]

    return [ExecuteProcess(cmd=record_cmd)]

# rosbag.py - launch_bag_player()
def launch_bag_player(context):
    conf = context.launch_configurations
    prev_prefixes = conf.get("previous_live_prefixes", "")
    base_topic = conf.get("base_trajectory_topic", "<default>")

    # Build topic patterns for previous LIVE runs
    extra_topics = []
    if prev_prefixes:
        for prefix in prev_prefixes.split(","):
            pattern = f"/{prefix.strip()}/planning/.*"
            extra_topics.append(pattern)

    # Add to publish_topic_from_rosbag
    existing = conf.get("publish_topic_from_rosbag", "")
    if extra_topics:
        combined = ",".join(extra_topics)
        conf["publish_topic_from_rosbag"] = f"{existing},{combined}" if existing else combined

    # ... rest of player setup
```

### Phase 2: Evaluation Configuration File

**Goal:** Allow evaluation script to read config file specifying which topics to evaluate

**Evaluation Config Format (YAML):**

```yaml
# evaluation_config.yaml
base_trajectory_topic: /planning/trajectory_generator/diffusion_planner_node/output/trajectory

runs:
  - prefix: model_v1.0_epoch50
    description: "Diffusion model v1.0 after 50 epochs"

  - prefix: model_v2.0_epoch100
    description: "Diffusion model v2.0 after 100 epochs"

  - prefix: baseline_pretrained
    description: "Baseline model for comparison"
```

**New Evaluation Script:** `scripts/evaluate_multi_run_bag.sh`

```bash
#!/bin/bash
# Evaluate all LIVE runs from config file
# Usage: ./evaluate_multi_run_bag.sh <final_bag> <input_bag> <config.yaml> <output_dir> <map_path>

python3 scripts/multi_run_evaluator.py \
  --final-bag "$1" \
  --input-bag "$2" \
  --config "$3" \
  --output-dir "$4" \
  --map-path "$5"
```

**Python Implementation:** `scripts/multi_run_evaluator.py`

```python
#!/usr/bin/env python3
"""Evaluate multiple LIVE trajectory topics from config file"""
import yaml
import subprocess
import json
from pathlib import Path
import argparse

def evaluate_all_runs(final_bag, input_bag, config_path, output_dir, map_path):
    # Load configuration
    with open(config_path) as f:
        config = yaml.safe_load(f)

    base_topic = config['base_trajectory_topic']
    runs = config['runs']

    results = []
    for run_config in runs:
        prefix = run_config['prefix']
        description = run_config.get('description', prefix)

        # Build full topic name
        topic = f"/{prefix}{base_topic}"
        eval_output = Path(output_dir) / prefix
        eval_output.mkdir(parents=True, exist_ok=True)

        print(f"\n{'='*80}")
        print(f"Evaluating: {description}")
        print(f"Prefix: {prefix}")
        print(f"Topic: {topic}")
        print(f"{'='*80}\n")

        # Run evaluation
        cmd = [
            "bash",
            "scripts/run_evaluation.sh",
            final_bag,
            input_bag,
            topic,
            str(eval_output),
            map_path
        ]
        subprocess.run(cmd, check=True)

        # Collect results
        result_json = eval_output / "or_results.json"
        with open(result_json) as f:
            data = json.load(f)
            results.append({
                "prefix": prefix,
                "description": description,
                "mean_ade": data["summary"]["ade"]["mean"],
                "std_ade": data["summary"]["ade"]["std"],
                "mean_fde": data["summary"]["fde"]["mean"],
                "std_fde": data["summary"]["fde"]["std"],
            })

    # Generate comparison
    generate_comparison(results, output_dir)

def generate_comparison(results, output_dir):
    # Sort by ADE
    results.sort(key=lambda x: x["mean_ade"])

    # Print table
    print("\n" + "="*80)
    print("Multi-Run Comparison Results")
    print("="*80)
    print(f"{'Description':<40} {'Mean ADE':<20} {'Mean FDE':<20}")
    print("-"*80)
    for r in results:
        print(f"{r['description']:<40} {r['mean_ade']:>6.3f} ±{r['std_ade']:<6.3f}    {r['mean_fde']:>6.3f} ±{r['std_fde']:<6.3f}")
    print("="*80)
    print(f"\nBest Performing: {results[0]['description']}")
    print(f"  ADE: {results[0]['mean_ade']:.3f}m")
    print(f"  Improvement vs worst: {((results[-1]['mean_ade'] - results[0]['mean_ade']) / results[-1]['mean_ade'] * 100):.1f}%")

    # Save to JSON
    summary_path = Path(output_dir) / "comparison_summary.json"
    with open(summary_path, 'w') as f:
        json.dump({
            "runs": results,
            "best_run": results[0],
            "worst_run": results[-1]
        }, f, indent=2)
    print(f"\nSaved to: {summary_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--final-bag", required=True)
    parser.add_argument("--input-bag", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--map-path", required=True)
    args = parser.parse_args()

    evaluate_all_runs(args.final_bag, args.input_bag, args.config,
                     args.output_dir, args.map_path)
```

## Research Questions (In Progress)

### Question 1: Can ros2 bag record remap topics during recording?

**Current Investigation:**
- Checking `ros2 bag record --help` for `--remap` option
- DLR already has `remap_arg` parameter for bag player
- Need to test if recorder supports remapping

**Test Needed:**
```bash
# Test if this works:
ros2 bag record --remap /planning/trajectory:=/live_model_v1/planning/trajectory \
  -e "^/planning/.*$"
```

### Question 2: Can we rename topics post-recording?

**Approaches to Test:**
1. `ros2 bag convert` with output options
2. `rosbag2_py` SequentialReader + SequentialWriter
3. Custom script using rosbag2_py

**Preferred:** Remap during recording (cleaner)
**Fallback:** Post-processing topic renaming if recorder doesn't support remap

### Question 3: Does DLR recorder already support remapping?

**Current Findings:**
- DLR has `remap_arg` for **player** (lines 165-167 in argument.py)
- DLR recorder at line ~177 in rosbag.py
- No obvious `--remap` in recorder command
- **Needs investigation:** Does recorder inherit remappings?

## Next Steps

1. **Test `ros2 bag record --remap`** to confirm it works
2. **Check DLR recorder code** for existing remap support
3. **If remapping works:** Implement in DLR
4. **If not:** Investigate post-recording topic rename
5. **Implement evaluation config reader**
6. **Test end-to-end workflow**

## Files Summary

**Completed:**
- Map visualization: 7 files modified
- Scripts: 3 new scripts + README

**In Progress:**
- Multi-run DLR support: TBD (researching best approach)
- Multi-run evaluation: Design complete, pending implementation

## Example Usage (Planned)

**DLR Iteration 1:**
```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  output_dir:=/path/to/run1 \
  live_topic_prefix:=model_v1.0_epoch50
```

**DLR Iteration 2:**
```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  input_bag:=/path/to/run1/result_bag \
  output_dir:=/path/to/run2 \
  live_topic_prefix:=model_v2.0_epoch100 \
  previous_live_prefixes:=model_v1.0_epoch50
```

**Evaluation:**
```bash
./scripts/evaluate_multi_run_bag.sh \
  /path/to/run2/result_bag/result_bag_0.mcap \
  /path/to/original_input_bag \
  /path/to/eval_config.yaml \
  /path/to/evaluation_output \
  /path/to/map.osm
```

## Dependencies

- DLR repository: `~/pilot-auto/src/simulator/driving_log_replayer_v2`
- Branch: `feat/open_loop_evalution-or-degradation-check`
- ROS 2 bag tools (already available)
- Python: yaml, json, subprocess (standard library)

## Test Dataset

- Location: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/`
- Duration: 180 seconds
- OR events: 3
- Map: shinagawa_odaiba_stable
