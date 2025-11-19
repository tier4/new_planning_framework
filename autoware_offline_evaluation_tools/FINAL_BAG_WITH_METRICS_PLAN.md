# Plan: Final Bag with All Data + Multi-Run Metrics

## Current Problem

**Issue 1: Evaluation output bag missing original data**
- Has metrics: `/or_scene/ade`, `/or_scene/fde`, etc.
- Missing: LIVE trajectories, GT, map, control_mode
- Can't be used standalone

**Issue 2: Metric topics use generic names**
- Current: `/or_scene/ade` (conflicts when merging multiple evaluations)
- Needed: `/model_v1/or_scene/ade`, `/model_v2/or_scene/ade`

**Issue 3: No combined final bag**
- DLR result_bag: Has LIVE runs + GT + map
- Evaluation bags: Have metrics (separate per run)
- Want: Single bag with EVERYTHING

## Goal

**Create final mcap containing:**
- All LIVE trajectory topics (`/prefix1/planning/...`, `/prefix2/planning/...`)
- GT trajectory (`/ground_truth/trajectory`)
- Map topics (`/map/vector_map_marker`, etc.)
- Control mode (`/vehicle/status/control_mode`)
- Kinematic state (`/localization/kinematic_state`)
- **ALL evaluation metrics with prefixes** (`/prefix1/or_scene/ade`, `/prefix2/or_scene/ade`)

## Proposed Solutions

### Solution A: Merge Bags Approach

**Steps:**
1. Evaluate each run → separate evaluation bags
2. Merge all evaluation bags + DLR result_bag
3. Use `ros2 bag merge` or custom Python script

**Pros:**
- No C++ code changes
- Uses existing tools

**Cons:**
- Topic name conflicts (need renaming during merge)
- Complex merging logic
- 3-4 minutes overhead per merge

### Solution B: Prefix Metric Topics (Recommended)

**Steps:**
1. Add `metric_topic_prefix` parameter to ORSceneEvaluator
2. Modify `get_result_topics()` to use prefix
3. Modify `bag_writer.write()` calls to use prefixed names
4. Create final bag merge script

**Implementation:**

**C++ Changes:**
```cpp
// or_scene_evaluator.hpp
std::string metric_topic_prefix_;  // Add member variable

void set_metric_topic_prefix(const std::string & prefix);

// or_scene_evaluator.cpp
std::vector<std::pair<std::string, std::string>> ORSceneEvaluator::get_result_topics() {
  std::string prefix = metric_topic_prefix_.empty() ? "" : "/" + metric_topic_prefix_;
  return {
    {prefix + "/or_scene/event_markers", "..."},
    {prefix + "/or_scene/ade", "..."},
    {prefix + "/or_scene/fde", "..."},
    // ...
  };
}

void save_event_metrics_to_bag(...) {
  std::string prefix = metric_topic_prefix_.empty() ? "" : "/" + metric_topic_prefix_;
  bag_writer.write(ade_msg, prefix + "/or_scene/ade", traj_time);
  bag_writer.write(fde_msg, prefix + "/or_scene/fde", traj_time);
  // ...
}
```

**Result:**
- `model_v1` prefix → `/model_v1/or_scene/ade`
- `model_v2` prefix → `/model_v2/or_scene/ade`
- No conflicts!

### Solution C: Final Bag Merger Script

**After all evaluations, create combined bag:**

**Python Script:** `scripts/create_final_combined_bag.py`

```python
def create_final_bag(dlr_result_bag, evaluation_bags, output_bag):
    """
    Merge DLR result bag + all evaluation output bags

    Args:
        dlr_result_bag: Has LIVE runs + GT + map + control_mode
        evaluation_bags: List of evaluation output bags (one per run)
        output_bag: Final combined output
    """

    # Copy all topics from DLR result bag
    # For each evaluation bag:
    #   - Rename metric topics with run prefix
    #   - Copy to final bag
    # Result: Everything in one bag
```

**Usage:**
```bash
python3 scripts/create_final_combined_bag.py \
  --dlr-bag ~/output/run3/result_bag/result_bag_0.mcap \
  --eval-bags ~/eval/run1/evaluation_output.bag,~/eval/run2/evaluation_output.bag \
  --prefixes model_v1,model_v2 \
  --output ~/final_bag.mcap
```

## Recommended Implementation Plan

### Phase 1: Add Metric Topic Prefix (C++)

**Files to modify:**
- `src/or_scene_evaluator.hpp` - Add `metric_topic_prefix_` member
- `src/or_scene_evaluator.cpp` - Use prefix in topic names
- `src/node.cpp` - Wire parameter

**Estimate:** 30 minutes

### Phase 2: Create Final Bag Merger (Python)

**New script:** `scripts/create_final_combined_bag.py`

**Logic:**
```python
1. Read DLR result_bag (has LIVE + GT + map + control_mode)
2. Copy all topics to output

3. For each evaluation bag:
   a. Read metric topics
   b. Rename with run prefix if specified
   c. Append to output

4. Reindex
```

**Estimate:** 45 minutes

### Phase 3: Integrate with Multi-Run Evaluator

**Modify:** `scripts/multi_run_evaluator.py`

**Add final step:**
```python
# After all evaluations complete
print("\nCreating final combined bag...")
create_final_bag(
    dlr_bag=args.final_bag,
    eval_bags=all_evaluation_bags,
    prefixes=all_prefixes,
    output=args.output_dir + "/final_combined.mcap"
)
```

**Estimate:** 15 minutes

## Alternative: Skip Evaluation Output Bag

**Simpler approach:**
- Don't use evaluation output bag at all
- Only use JSON for metrics
- Keep DLR result_bag as final output
- Metrics available in JSON, original data in bag

**Pros:**
- No merging needed
- DLR result_bag already has all original data
- Simpler workflow

**Cons:**
- No time-series metrics in bag (only JSON summary)
- Can't replay metrics in rviz

## Recommendation

**Implement Solution B + C:**
1. Add metric topic prefix to evaluator (allows multi-run metrics in separate bags)
2. Create final bag merger script (combines DLR + all evaluations)
3. Make it optional (user choice: separate bags vs combined)

**Total effort:** ~90 minutes

Should I proceed with implementation?
