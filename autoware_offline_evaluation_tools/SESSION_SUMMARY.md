# Session Summary - 2025-11-11

## Work Completed

### 1. Dataset Setup

**Downloaded and prepared 3 rosbags:**
- IDs: c0053313, 69b43245, 149d3181
- Merged into single 180-second bag (27.1 GiB)
- Location: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/`
- T4 dataset structure created with map symlink

**Reindexed bags:**
- All bags reindexed with `ros2 bag reindex -s sqlite3`
- Generated metadata.yaml files

### 2. Map Visualization Feature

**Implemented lanelet2 map overlay in OR scene debug images.**

**Files Modified:**
1. `scripts/generate_or_visualization.py` (+60 lines)
   - Added lanelet2 import with fallback handling
   - Added `load_lanelet_map()`, `get_lanelets_in_area()`, `plot_lanelet_boundaries()`
   - Enhanced legend with object types (Car, Truck, Pedestrian)

2. `src/or_scene_evaluator.hpp` (+7 lines)
   - Added `map_path_` member variable
   - Added `set_map_path()` method

3. `src/or_scene_evaluator.cpp` (+10 lines)
   - Implemented `set_map_path()` setter
   - Added map_path to visualization JSON

4. `src/node.cpp` (+5 lines)
   - Wired `or_scene_evaluation.map_path` parameter

5. `config/offline_evaluation.param.yaml` (+1 line)
   - Added map_path parameter

**Visual Results:**
- Light gray lanelet boundaries (solid, alpha=0.6, linewidth=1.0, zorder=1)
- Light gray centerlines (dashed, alpha=0.4, linewidth=0.5, zorder=1)
- Trajectories remain focus (zorder=5)
- File size: 230-400KB (vs 145KB without map)
- 2-6 lanelets plotted per image (spatially filtered)

**Testing:**
- ✅ Full DLR + evaluation pipeline tested
- ✅ 30 images generated with map visualization
- ✅ Map provides road context without overwhelming trajectories
- ✅ Console confirms: "Plotted N lanelets from map"

### 3. Parameterized Scripts

**Created 3 new scripts in `autoware_offline_evaluation_tools/scripts/`:**

1. **`run_dlr.sh`**
   - Run DLR simulation only
   - Usage: `./run_dlr.sh <scenario_path>`

2. **`run_evaluation.sh`**
   - Run evaluation only (on existing result_bag)
   - Usage: `./run_evaluation.sh <result_bag> <input_bag> <topic> <output_dir> [map_path]`
   - Fully parameterized, no hardcoded paths

3. **`run_dlr_and_evaluate_fixed.sh`**
   - Complete pipeline (DLR + evaluation)
   - Usage: `./run_dlr_and_evaluate_fixed.sh <scenario> <topic> <output> [map]`
   - Includes cleanup, runs both phases, displays summary

4. **`README.md`**
   - Comprehensive user documentation
   - Examples for all workflows
   - T4 dataset preparation guide
   - Troubleshooting section

**Note:** Original `run_dlr_and_evaluate.sh` had argument parsing issues, created `_fixed.sh` version.

### 4. LIVE vs HISTORICAL Verification

**Confirmed DLR correctly distinguishes LIVE from HISTORICAL:**

**Evidence:**
- Result bag (LIVE): 1724-1725 trajectory messages
- Input bag (HISTORICAL): 1795 trajectory messages
- Different message counts = different data ✓

**Mechanism:**
- `publish_profile: planning_control` whitelists only inputs (route, perception, localization)
- `/planning/trajectory_generator/.../trajectory` NOT in whitelist
- Historical trajectories NEVER replayed
- Planning nodes run LIVE, generate new trajectories

**Verification Performed:**
- Ran both LIVE and HISTORICAL evaluations
- LIVE: Mean ADE 2.453m (±0.565m)
- HISTORICAL: Mean ADE 2.600m (±1.537m)
- Metrics differ by 6% - confirms different data

### 5. DLR Configuration Investigation

**Researched existing DLR capabilities:**

**Found:**
- `publish_topic_from_rosbag` parameter exists (for adding topics to replay)
- `remap_arg` parameter exists (for bag **player** remapping)
- `remap_profile` parameter exists (YAML-based remap configs)

**Recorder Implementation:**
- Located at `rosbag.py:launch_bag_recorder()`
- Uses `ros2 bag record` with `-e` (regex) for topic selection
- **No obvious `--remap` support for recorder yet**

## Current Investigation

### Testing `ros2 bag record --remap`

**Question:** Does `ros2 bag record` support `--remap` flag for renaming topics during recording?

**Need to test:**
```bash
ros2 bag record --remap /planning/trajectory:=/live_test/planning/trajectory \
  -e "^/planning/.*$" \
  -o /tmp/test_remap
```

**Alternative:** Post-recording topic renaming with `ros2 bag convert` or `rosbag2_py`

## Next Steps for Multi-Run Feature

### Immediate Tasks

1. ✅ Document current work (this file)
2. ⏳ Test `ros2 bag record --remap` capability
3. ⏳ Investigate post-recording topic renaming if needed
4. ⏳ Implement DLR parameter additions
5. ⏳ Implement multi-run evaluator script
6. ⏳ Test complete multi-run workflow

### Implementation Plan

**If recorder supports --remap:**
- Modify DLR `launch_bag_recorder()` to add `--remap` flags
- Easier, cleaner approach

**If not:**
- Create post-processing script to rename topics in result_bag
- Use `rosbag2_py` SequentialReader/Writer
- Slightly more complex but still feasible

## Important Context for Next Claude Instance

### Repository Structure
- Evaluation tools: `/home/danielsanchez/pilot-auto/src/autoware/new_planning_framework/autoware_offline_evaluation_tools/`
- DLR: `/home/danielsanchez/pilot-auto/src/simulator/driving_log_replayer_v2/`
- Test dataset: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/`

### Build Commands
```bash
# Build evaluation tools
cd ~/pilot-auto && colcon build --packages-select autoware_offline_evaluation_tools

# Build DLR
cd ~/pilot-auto && colcon build --packages-select driving_log_replayer_v2
```

### Key Files to Understand

**DLR:**
- `driving_log_replayer_v2/launch/argument.py` - Parameter definitions
- `driving_log_replayer_v2/launch/rosbag.py` - Bag player/recorder implementation
- `driving_log_replayer_v2/config/publish/planning_control.yaml` - Topic whitelist

**Evaluation:**
- `src/or_scene_evaluator.{hpp,cpp}` - OR scene evaluation logic
- `src/node.cpp` - Parameter wiring
- `scripts/generate_or_visualization.py` - Visualization generation

### Recent Results

**Latest evaluation (with map):**
- Location: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/final_test_with_map/`
- JSON: `/home/danielsanchez/or_scene_evaluation_results_20251111_120459.json`
- Images: 30 PNG files with map overlay
- Metrics: Mean ADE 1.977m (±1.082m), Mean FDE 3.523m (±2.706m)

### User Preferences

From `~/.claude/CLAUDE.md`:
- Check existing code patterns before implementing
- Reuse existing functions/code
- Review own code before claiming completion
- No emojis in code/documentation
- Use webauto with save location (don't download then copy)
