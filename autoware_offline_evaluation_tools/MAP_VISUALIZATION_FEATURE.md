# Map Visualization Feature for OR Scene Evaluation

## Overview

Added lanelet map visualization to OR scene debug images. Lane boundaries and centerlines are now drawn as a subtle background layer, providing road context for trajectory analysis.

## What Changed

### Files Modified

**Python Script:**
- `scripts/generate_or_visualization.py` (+60 lines)
  - Added lanelet2 import with fallback handling
  - Added `load_lanelet_map()` function
  - Added `get_lanelets_in_area()` for spatial filtering
  - Added `plot_lanelet_boundaries()` for visualization
  - Added object type legend (Car, Truck, Pedestrian, etc.)

**C++ Code:**
- `src/or_scene_evaluator.hpp` (+7 lines)
  - Added `map_path_` member variable
  - Added `set_map_path()` setter method

- `src/or_scene_evaluator.cpp` (+10 lines)
  - Implemented `set_map_path()` setter
  - Added map_path to visualization JSON data

- `src/node.cpp` (+5 lines)
  - Wired `or_scene_evaluation.map_path` parameter

**Configuration:**
- `config/offline_evaluation.param.yaml` (+1 line)
  - Added `or_scene_evaluation.map_path` parameter

**Scripts:**
- `scripts/run_evaluation.sh` - Added optional map_path argument
- `scripts/run_dlr_and_evaluate.sh` - Added optional map_path argument
- `scripts/README.md` - Updated documentation

## Visual Results

**Before (no map):**
- Trajectories and objects on blank canvas
- File size: ~145-175 KB

**After (with map):**
- Light gray lanelet boundaries (solid lines, alpha=0.6, linewidth=1.0, zorder=1)
- Light gray centerlines (dashed lines, alpha=0.4, linewidth=0.5, zorder=1)
- Trajectories clearly visible on top (zorder=5)
- File size: ~230-400 KB (increased due to map data)
- Object legend shows types (Car, Truck, Pedestrian)

## Usage

### Method 1: Using Evaluation Script

```bash
./scripts/run_evaluation.sh \
  /path/to/result_bag_0.mcap \
  /path/to/input_bag \
  /planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  /path/to/output_dir \
  /path/to/map/lanelet2_map.osm
```

### Method 2: Using Combined DLR + Evaluation Script

```bash
./scripts/run_dlr_and_evaluate.sh \
  /path/to/scenario.yaml \
  /planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  /path/to/output_dir \
  /path/to/map/lanelet2_map.osm
```

### Method 3: Direct ROS 2 Command

```bash
ros2 run autoware_offline_evaluation_tools offline_evaluator_node --ros-args \
  -p bag_path:=/path/to/result_bag_0.mcap \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=/planning/.../output/trajectory \
  -p or_scene_evaluation.input_bag_path:=/path/to/input_bag \
  -p or_scene_evaluation.enable_debug_visualization:=true \
  -p or_scene_evaluation.debug_output_dir:=/path/to/output \
  -p or_scene_evaluation.map_path:=/path/to/lanelet2_map.osm \
  -p json_output_path:=/path/to/results.json
```

### Method 4: Using T4 Dataset Structure (Auto-Detect Map)

If using T4 dataset format with map in `<dataset>/map/lanelet2_map.osm`:

```bash
./scripts/run_evaluation.sh \
  ~/t4_dataset/my_test/out/latest/result_bag/result_bag_0.mcap \
  ~/t4_dataset/my_test/input_bag \
  /planning/trajectory_generator/diffusion_planner_node/output/trajectory \
  ~/t4_dataset/my_test/results \
  ~/t4_dataset/my_test/map/lanelet2_map.osm
```

## Feature Behavior

**With Map Path:**
- Loads lanelet2 map from OSM file
- Filters lanelets within visualization bounds (trajectory extent + 10% epsilon)
- Plots lane boundaries and centerlines as background layer
- Console output: `Plotted N lanelets from map`

**Without Map Path (or if loading fails):**
- Gracefully degrades to original visualization (no map)
- Warning printed to console if map loading fails
- All other functionality unchanged

## Dependencies

**Python Requirements:**
- `lanelet2` Python bindings
- `autoware_lanelet2_extension_python` (MGRSProjector)
- Fallback to `UtmProjector` if MGRSProjector unavailable

**Already included in Autoware environment** - no additional installation needed.

## Implementation Details

### Lanelet Loading

```python
from lanelet2.io import Origin, load
from autoware_lanelet2_extension_python.projection import MGRSProjector

projector = MGRSProjector(Origin(0.0, 0.0))
lanelet_map = load(map_path, projector)
```

### Spatial Filtering

Only lanelets within visualization bounds are plotted for performance:
- Checks if any centerline point falls within `[x_min - epsilon, x_max + epsilon] × [y_min - epsilon, y_max + epsilon]`
- Typical result: 2-10 lanelets per image (vs thousands in full map)

### Rendering Order (zorder)

1. **Map layer (zorder=1)** - Lanelet boundaries and centerlines
2. **Object layer (zorder=4)** - Bounding boxes
3. **Trajectory layer (zorder=5)** - GT and predicted paths
4. **Marker layer (zorder=6)** - Start/end points
5. **Event layer (zorder=10)** - OR event marker

## Visual Styling

| Element | Color | Style | Width | Alpha | zorder |
|---------|-------|-------|-------|-------|--------|
| Lane bounds | lightgray | solid | 1.0 | 0.6 | 1 |
| Lane center | lightgray | dashed | 0.5 | 0.4 | 1 |
| GT trajectory | green | solid | 2.5 | 0.9 | 5 |
| Predicted | blue | dashed | 2.0 | 0.9 | 5 |
| Objects | class color | solid | 2.5 | 0.8 | 4 |
| OR marker | red | star | 15 | 1.0 | 10 |

## Testing Results

Tested with:
- Dataset: `/media/danielsanchez/2fb4af16-188c-4b7d-8ebb-4a7d0c90d207/t4_dataset_merged/`
- Map: `shinagawa_odaiba_stable/lanelet2_map.osm`
- 3 OR events, 30 total images generated
- All images successfully show map + trajectories + objects

**Observations:**
- Map provides excellent road context
- Lane boundaries help understand if trajectories stayed in lane
- Centerlines show intended path
- Visualization remains focused on trajectories (map is subtle background)
- No performance issues (loads quickly, <1s per image)

## Backward Compatibility

✅ **Fully backward compatible**
- Map path parameter is optional (default: empty string)
- If no map path provided, visualization works exactly as before
- If map loading fails, gracefully falls back to no-map mode
- Existing scripts and commands continue to work unchanged

## Future Enhancements (Optional)

Potential improvements:
1. **Color-code lane types** - Different colors for different road types (highway, city, parking)
2. **Show lane change arrows** - Visualize allowed/prohibited lane changes
3. **Highlight relevant lanelets** - Bold the lanes vehicle was in
4. **Add traffic light positions** - Show traffic lights from lanelet2 regulatory elements
5. **Show stop lines** - Mark where stop lines are located

## Files Summary

```
autoware_offline_evaluation_tools/
├── config/offline_evaluation.param.yaml         # Added map_path parameter
├── src/
│   ├── or_scene_evaluator.hpp                   # Added map_path member + setter
│   ├── or_scene_evaluator.cpp                   # Implemented setter, added to JSON
│   └── node.cpp                                 # Wired parameter
└── scripts/
    ├── generate_or_visualization.py             # Added map loading + plotting
    ├── run_evaluation.sh                        # Added map_path argument
    ├── run_dlr_and_evaluate.sh                  # Added map_path argument
    └── README.md                                # Updated documentation
```

**Total Changes:** ~80 lines of code across 7 files.
