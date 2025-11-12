# Multi-Run Feature - Implementation Status

## ✅ Implementation Complete

### Features Implemented

**1. Post-Recording Topic Renaming**
- Script: `scripts/rename_bag_topic.py`
- Tested: Successfully renamed 1724 trajectory messages in 2.5GB bag
- Performance: ~2 minutes for full bag copy + rename
- Uses: rosbag2_py SequentialReader/Writer (based on ros2bag_extensions pattern)

**2. DLR Integration**
- Added 3 new parameters to DLR:
  - `live_topic_prefix` - Descriptive prefix for current run
  - `base_trajectory_topic` - Which topic to rename (default: diffusion_planner)
  - `previous_live_prefixes` - Comma-separated list of previous runs to replay

**3. Automatic Renaming in DLR Post-Processing**
- File: `driving_log_replayer_v2/launch/post_process.launch.py`
- Function: `rename_live_topic_if_needed()`
- When: Runs automatically after bag recording completes
- What: Renames LIVE trajectory topic if `live_topic_prefix` is set

**4. Previous LIVE Topic Replay**
- File: `driving_log_replayer_v2/launch/rosbag.py`
- Function: `user_defined_publish()` enhanced
- When: During bag playback setup
- What: Adds previous LIVE topics to replay list based on `previous_live_prefixes`

**5. Config-Based Multi-Run Evaluation**
- Script: `scripts/evaluate_multi_run_bag.sh`
- Script: `scripts/multi_run_evaluator.py`
- Config: `config/example_multi_run_eval.yaml`
- Reads YAML config specifying runs to evaluate
- Generates comparison table and summary JSON

## Implementation Details

### DLR Changes

**Files Modified:**
1. `driving_log_replayer_v2/launch/argument.py` (+12 lines)
   - Added `live_topic_prefix` parameter
   - Added `base_trajectory_topic` parameter
   - Added `previous_live_prefixes` parameter

2. `driving_log_replayer_v2/launch/rosbag.py` (+13 lines)
   - Enhanced `user_defined_publish()` to include previous LIVE topics

3. `driving_log_replayer_v2/launch/post_process.launch.py` (+44 lines)
   - Added `rename_live_topic_if_needed()` function
   - Integrated into post_process() workflow

### Evaluation Tools Changes

**Files Created:**
1. `scripts/rename_bag_topic.py` (107 lines)
   - Core topic renaming utility
   - Based on ros2bag_extensions filter.py pattern
   - Handles mcap and sqlite3 formats
   - Auto-reindexes after renaming

2. `scripts/evaluate_multi_run_bag.sh` (60 lines)
   - Wrapper for multi-run evaluation
   - Calls Python evaluator with config file

3. `scripts/multi_run_evaluator.py` (150 lines)
   - Reads YAML config
   - Runs evaluation for each prefix
   - Generates comparison table
   - Saves summary JSON

4. `config/example_multi_run_eval.yaml` (11 lines)
   - Example configuration template

5. `MULTI_RUN_USER_GUIDE.md` (200+ lines)
   - Complete user documentation
   - Step-by-step workflow
   - Examples and troubleshooting

## Testing Status

### ✅ Tested

1. **Topic Renaming**
   - Input: result_bag_0.mcap (2.5GB, 160K messages)
   - Renamed: 1724 trajectory messages
   - Output: Verified with `ros2 bag info`
   - Topic: `/planning/.../trajectory` → `/live_model_test/.../trajectory`

2. **Build Success**
   - DLR rebuilt successfully with new parameters
   - No compilation errors

### ⏳ Needs Testing

1. **End-to-End DLR with Renaming**
   - Run DLR with `live_topic_prefix:=test_model`
   - Verify automatic renaming in post-processing
   - Check result_bag contains renamed topic

2. **Multi-Iteration Workflow**
   - Run iteration 1 with prefix A
   - Run iteration 2 with prefix B + previous A
   - Verify both topics in final bag

3. **Config-Based Evaluation**
   - Create config with 2-3 runs
   - Run multi_run_evaluator.py
   - Verify all evaluations complete
   - Check comparison summary

## Usage

### Run 1 (First Model)
```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  output_dir:=/path/to/run1 \
  live_topic_prefix:=model_v1.0
```

### Run 2 (Second Model, builds on Run 1)
```bash
ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=/path/to/scenario.yaml \
  input_bag:=/path/to/run1/result_bag \
  output_dir:=/path/to/run2 \
  live_topic_prefix:=model_v2.0 \
  previous_live_prefixes:=model_v1.0
```

### Evaluate All Runs
```bash
./scripts/evaluate_multi_run_bag.sh \
  /path/to/run2/result_bag/result_bag_0.mcap \
  /path/to/original_input_bag \
  /path/to/eval_config.yaml \
  /path/to/output \
  /path/to/map.osm
```

## Known Limitations

1. **Topic renaming takes ~2 minutes** for 2.5GB bag (acceptable overhead)
2. **Must manually change model** between iterations (as intended)
3. **Previous LIVE topics must be explicitly listed** (no auto-detection)
4. **Base topic must match** across all iterations (can't mix diffusion + optimizer)

## Next Steps for Testing

1. Run small DLR test with `live_topic_prefix`
2. Verify automatic renaming works
3. Run second iteration with `previous_live_prefixes`
4. Test multi-run evaluation with config file
5. Document any issues found

## Files Changed Summary

**DLR (3 files):**
- `launch/argument.py` - Added parameters
- `launch/rosbag.py` - Replay previous LIVE topics
- `launch/post_process.launch.py` - Automatic renaming

**Evaluation Tools (5 files created):**
- `scripts/rename_bag_topic.py` - Renaming utility
- `scripts/evaluate_multi_run_bag.sh` - Evaluation wrapper
- `scripts/multi_run_evaluator.py` - Config-based evaluator
- `config/example_multi_run_eval.yaml` - Example config
- `MULTI_RUN_USER_GUIDE.md` - User documentation

**Documentation (2 files):**
- `MULTI_RUN_IMPLEMENTATION_PLAN.md` - Original plan
- `MULTI_RUN_IMPLEMENTATION_STATUS.md` - This file
