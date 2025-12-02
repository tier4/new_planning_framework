# Multi-Model OR Scene Evaluation Workflow

This document describes the complete workflow for evaluating multiple trajectory planning models against Override Regression (OR) events using segmented rosbags.

## Overview

The workflow evaluates multiple diffusion planner model versions (v2.0, v2.1, etc.) against historical OR events by:
1. Detecting OR events and extracting route segments
2. Cutting the input bag into segments centered on OR events
3. Running each model on each segment using DLR (driving_log_replayer_v2)
4. Merging model outputs into combined bags with prefixed topics
5. Evaluating all models simultaneously and injecting metrics back into bags

## Prerequisites

### Required Software
- ROS 2 Humble
- Autoware built and sourced
- Python 3 with rosbag2_py, rclpy

### Required Repository Branches

**CRITICAL: driving_log_replayer_v2 must be on the correct branch:**
```bash
cd ~/pilot-auto/src/simulator/driving_log_replayer_v2
git checkout feat/open_loop_evalution-or-degradation-check
```

This branch includes:
- Turn indicators topic remapping fix (required for diffusion planner to generate trajectories)
- `/perception/object_recognition/tracking/objects` in publish profile

Without this branch, LIVE trajectories will NOT be generated (Count: 0 in result bags).

### Required Model Files
- Model weight directories (e.g., `~/autoware_data/diffusion_planner/v2.0`)
- Each must contain: `diffusion_planner.onnx` and `diffusion_planner.param.json`
- Config file: `autoware_diffusion_planner/config/diffusion_planner.param.yaml`

## Quick Start

**Run the complete workflow in one command:**

```bash
bash autoware_offline_evaluation_tools/scripts/multi_model_or_segments.sh \
  --models /home/danielsanchez/autoware_data/diffusion_planner/v2.0 /home/danielsanchez/autoware_data/diffusion_planner/v2.1 \
  --config /home/danielsanchez/pilot-auto/src/autoware/universe/planning/autoware_diffusion_planner/config/diffusion_planner.param.yaml \
  --dataset /path/to/t4_dataset \
  --output /path/to/output
```

This automatically:
1. Detects OR events
2. Cuts segments
3. Adds ground truth
4. Runs DLR for v2.0 and v2.1 on all segments
5. Merges model outputs with prefixed topics

**Then run metrics evaluation:**

```bash
python3 autoware_offline_evaluation_tools/scripts/evaluate_or_segments.py \
  --input-dir /path/to/output \
  --map-path /path/to/dataset/map/lanelet2_map.osm \
  --time-window 5.0  # Optional: evaluation window on each side of OR event (default: 5.0s)
```

This auto-discovers all `or_event_*` bags in the input directory and generates `or_event_*_WITH_METRICS` bags with embedded evaluation results in the same directory.

**Parameters:**
- `--input-dir`: Directory created by multi_model_or_segments.sh (contains or_event_0, or_event_1, etc.)
- `--map-path`: Path to lanelet2 map file (used for route handling)
- `--time-window`: Evaluation window duration on each side of the Override Regression event (in seconds)
  - Default: 5.0 seconds (evaluates from OR-5s to OR+5s)
  - Example: `--time-window 10.0` creates a ±10 second window around each OR event
  - Larger windows capture more trajectory predictions but may include less relevant data

## Directory Structure

```
/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/
├── input_bag/              # Original input bag (db3 format)
├── map/
│   └── lanelet2_map.osm    # HD map for route handler
└── scenario.yaml           # DLR configuration

/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/multi_model_output/
├── or_event_0/             # DLR output for segment 0 (v2.0 + v2.1 merged)
├── or_event_1/             # DLR output for segment 1
├── or_event_2/             # DLR output for segment 2
├── or_event_0_WITH_METRICS/  # With evaluation metrics injected
├── or_event_1_WITH_METRICS/
├── or_event_2_WITH_METRICS/
├── or_event_0_results.json   # Evaluation metrics (v2.0 and v2.1)
├── or_event_1_results.json
└── or_event_2_results.json
```

## Scripts

All scripts are located in `~/pilot-auto/scripts/`:

### 1. `detect_or_and_route.py`
Detects OR events and extracts route segments from input bag.

**Usage:**
```bash
python3 ~/pilot-auto/scripts/detect_or_and_route.py \
  --bag /path/to/input_bag \
  --map-path /path/to/lanelet2_map.osm \
  --output or_events.json
```

**Output:**
```json
{
  "or_events": [
    {
      "timestamp": 1761555479.858451,
      "route_start": 25.0,
      "route_end": 145.0
    }
  ]
}
```

### 2. `cut_or_segment_with_route.py`
Cuts input bag into segments with route filtering.

**Usage:**
```bash
python3 ~/pilot-auto/scripts/cut_or_segment_with_route.py \
  --bag /path/to/input_bag \
  --or-events or_events.json \
  --output-dir /path/to/output \
  --before-sec 5.0 \
  --after-sec 10.0
```

**Output:**
- `output_dir/or_event_0/input_bag/` (db3 format)
- `output_dir/or_event_1/input_bag/`
- etc.

### 3. `multi_model_or_segments.sh`
Main orchestration script that runs the complete workflow.

**Usage:**
```bash
bash ~/pilot-auto/scripts/multi_model_or_segments.sh
```

**What it does:**
1. Detects OR events and extracts routes
2. Cuts segments
3. Copies map files to segment directories
4. Creates scenario.yaml for each segment
5. Runs DLR for v2.0 model
6. Adds prefix to v2.0 trajectories
7. Runs DLR for v2.1 model
8. Adds prefix to v2.1 trajectories
9. Merges v2.0 and v2.1 outputs
10. Cleans up temporary files

### 4. `evaluate_or_segments.py`
Evaluates all segments with metrics injection.

**Usage:**
```bash
python3 ~/pilot-auto/scripts/evaluate_or_segments.py \
  --input-dir /path/to/multi_model_output \
  --map-path /path/to/lanelet2_map.osm \
  --time-window 5.0
```

**What it does:**
1. Finds all segment bags (or_event_*)
2. Runs `evaluate_all_live_trajectories.py` on each
3. Creates WITH_METRICS bags with embedded metrics
4. Generates JSON results for each segment

### 5. `evaluate_all_live_trajectories.py`
Evaluates all LIVE trajectory topics in a bag simultaneously.

**Usage:**
```bash
python3 ~/pilot-auto/scripts/evaluate_all_live_trajectories.py \
  --bag /path/to/segment/bag \
  --input-bag /path/to/original/input_bag \
  --map-path /path/to/lanelet2_map.osm \
  --output-bag /path/to/output_WITH_METRICS \
  --json-output /path/to/results.json \
  --time-window 0.5
```

**What it does:**
1. Detects all LIVE trajectory topics with prefixes (e.g., `/v2_0/planning/.../trajectory`)
2. Runs offline_evaluator_node for each model
3. Merges all metric bags into output bag
4. Preserves message schemas (no reindexing!)
5. Sorts messages by timestamp
6. Filters out empty topics

**Key Implementation Details:**
- Uses temp directory approach for safe bag writing
- Collects all messages (original + metrics) before writing
- Sorts by timestamp: `all_messages.sort(key=lambda x: x[2])`
- **Skips Python reindexing** to preserve Autoware message schemas
- Filters topics: only creates topics that have messages

## Complete Workflow

### Step 1: Setup
```bash
# Source Autoware environment
source ~/pilot-auto/install/setup.bash

# Set paths
INPUT_BAG=/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/input_bag
MAP=/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/map/lanelet2_map.osm
OUTPUT=/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/multi_model_output
```

### Step 2: Run Multi-Model Workflow
```bash
# This runs everything: detection, cutting, DLR for all models, merging
bash ~/pilot-auto/scripts/multi_model_or_segments.sh
```

### Step 3: Evaluate All Segments
```bash
# Evaluate all segments and inject metrics
python3 ~/pilot-auto/scripts/evaluate_or_segments.py \
  --input-dir $OUTPUT \
  --map-path $MAP \
  --time-window 5.0
```

### Step 4: View Results

**In Lichtblick:**
1. Open a WITH_METRICS bag: `or_event_0_WITH_METRICS`
2. Visualize trajectories:
   - `/v2_0/planning/trajectory_generator/diffusion_planner_node/output/trajectory`
   - `/v2_1/planning/trajectory_generator/diffusion_planner_node/output/trajectory`
   - `/ground_truth/trajectory`
3. Plot metrics:
   - `/v2_0/or_scene/ade`
   - `/v2_0/or_scene/fde`
   - `/v2_0/or_scene/lateral_deviation`
   - `/v2_0/or_scene/ttc`
   - Same for v2_1

**JSON Results:**
```bash
# View summary for segment 0
cat $OUTPUT/or_event_0_results.json | python3 -m json.tool | less

# Compare models
cat $OUTPUT/or_event_0_results.json | python3 -c "
import json, sys
d = json.load(sys.stdin)
print('=== v2.0 Summary ===')
print(f'Events: {d[\"v2_0\"][\"summary\"][\"total_or_events\"]}')
print(f'Mean ADE: {d[\"v2_0\"][\"summary\"][\"ade\"][\"mean\"]:.3f}m')
print(f'Mean FDE: {d[\"v2_0\"][\"summary\"][\"fde\"][\"mean\"]:.3f}m')
print()
print('=== v2.1 Summary ===')
print(f'Events: {d[\"v2_1\"][\"summary\"][\"total_or_events\"]}')
print(f'Mean ADE: {d[\"v2_1\"][\"summary\"][\"ade\"][\"mean\"]:.3f}m')
print(f'Mean FDE: {d[\"v2_1\"][\"summary\"][\"fde\"][\"mean\"]:.3f}m')
"
```

## Results Example (or_event_0)

```
=== v2.0 Summary ===
Events: 3
Mean ADE: 2.693m
Mean FDE: 6.931m
Mean Lateral Dev: 0.020m

=== v2.1 Summary ===
Events: 3
Mean ADE: 2.692m
Mean FDE: 6.930m
Mean Lateral Dev: 0.020m
```

Both models show similar performance on this segment. The low lateral deviation (2cm) suggests good lane-keeping behavior.

## Bag Structure

### Original Segment Bag (or_event_0)
- Duration: 14.35s
- Messages: 24,743
- Topics: 348 (including v2_0 and v2_1 trajectory outputs)

### WITH_METRICS Bag (or_event_0_WITH_METRICS)
- Duration: 14.35s
- Messages: 24,829 (+86 metric messages)
- Topics: 348
- Additional metric topics:
  - `/v2_0/or_scene/ade` (10 messages)
  - `/v2_0/or_scene/fde` (10 messages)
  - `/v2_0/or_scene/lateral_deviation` (10 messages)
  - `/v2_0/or_scene/ttc` (10 messages)
  - `/v2_0/or_scene/event_markers` (1 message)
  - Same for v2_1
  - `/ground_truth/trajectory` (1397 messages)

## Metrics Explanation

### ADE (Average Displacement Error)
Mean distance between predicted trajectory and ground truth across all waypoints.

### FDE (Final Displacement Error)
Distance between final predicted waypoint and ground truth at trajectory end.

### Lateral Deviation
Perpendicular distance from predicted trajectory to ground truth (in vehicle frame).

### TTC (Time to Collision)
Minimum time to collision with predicted objects. Default 10.0 = no collision.

## Troubleshooting

### Lichtblick Cannot Visualize Trajectories
**Problem:** Trajectory topics show up but cannot be visualized.

**Cause:** Python `Reindexer()` strips Autoware message schemas from MCAP files.

**Solution:** The evaluation script now skips reindexing. Message schemas are preserved by `SequentialWriter`.

**Verification:**
```bash
# Check if bag is valid
source ~/pilot-auto/install/setup.bash
ros2 bag info /path/to/WITH_METRICS/bag

# Should show proper topic counts without errors
```

### Empty Metric Topics
**Problem:** Metric topics exist but have 0 messages.

**Cause:** Evaluation failed or no trajectories in time window.

**Solution:** Check evaluation logs:
```bash
# Re-run evaluation with verbose output
python3 ~/pilot-auto/scripts/evaluate_all_live_trajectories.py \
  --bag /path/to/segment \
  --input-bag /path/to/input_bag \
  --map-path /path/to/map.osm \
  --output-bag /tmp/test_metrics \
  --json-output /tmp/test_results.json
```

### Timestamp Ordering Issues
**Problem:** Bag playback is jumpy or out of order.

**Cause:** Messages not sorted by timestamp.

**Solution:** The script now sorts all messages before writing:
```python
all_messages.sort(key=lambda x: x[2])  # Sort by timestamp
```

## Model Configuration

The workflow uses model prefixes to distinguish outputs:
- **v2_0**: Model version 2.0
- **v2_1**: Model version 2.1

To add more models:
1. Edit `multi_model_or_segments.sh`
2. Add new DLR run + prefix + merge steps
3. Models automatically detected by `evaluate_all_live_trajectories.py`

## Performance Notes

- Segment evaluation takes ~2-3 minutes per segment per model
- Full workflow (3 segments, 2 models): ~15-20 minutes
- Bag merging is fast (<10 seconds per segment)
- Most time is spent in offline_evaluator_node running OR scene evaluation

## Future Improvements

1. **Parallel Evaluation**: Run multiple models in parallel instead of sequentially
2. **Incremental Metrics**: Skip re-evaluation if metrics already exist
3. **Three-Way Visualization**: Add HISTORICAL trajectory comparison to debug plots
4. **Aggregate Report**: Generate HTML report comparing all models across all segments
5. **Metric Thresholds**: Add pass/fail criteria based on ADE/FDE thresholds

## References

- DLR Repository: `~/pilot-auto/src/simulator/driving_log_replayer_v2`
- Evaluation Tool: `~/pilot-auto/src/autoware/new_planning_framework/autoware_offline_evaluation_tools`
- OR Scene Evaluation Guide: `~/pilot-auto/src/autoware/new_planning_framework/CLAUDE.md`

## Git Commits

The Lichtblick visualization fix was committed as:
- `c8483bf0`: Skip Python reindexing to preserve message schemas
- `5a9366b7`: Sort messages by timestamp before writing
- `d84f5242`: Filter out empty metric topics
- `b169db6f` + `08d5fda7`: Use temp directory approach for bag writing

## License

This workflow is part of the Autoware planning framework evaluation pipeline.
