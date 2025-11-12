#!/bin/bash
# Script to run OR scene evaluation on DLR results
# Usage: ./run_evaluation.sh <result_bag_path> <input_bag_path> <trajectory_topic> <output_dir> [map_path]

set -e

if [ $# -lt 4 ]; then
    echo "Usage: $0 <result_bag_path> <input_bag_path> <trajectory_topic> <output_dir> [map_path]"
    echo ""
    echo "Arguments:"
    echo "  result_bag_path   - Path to DLR result_bag_0.mcap (LIVE trajectories)"
    echo "  input_bag_path    - Path to input bag directory (for OR extraction)"
    echo "  trajectory_topic  - Trajectory topic name to evaluate"
    echo "  output_dir        - Directory for outputs (JSON, images, logs)"
    echo "  map_path          - Optional: Path to lanelet2_map.osm for lane visualization"
    echo ""
    echo "Example:"
    echo "  $0 \\"
    echo "    /path/to/result_bag/result_bag_0.mcap \\"
    echo "    /path/to/input_bag \\"
    echo "    /planning/trajectory_generator/diffusion_planner_node/output/trajectory \\"
    echo "    /path/to/output_dir \\"
    echo "    /path/to/map/lanelet2_map.osm"
    exit 1
fi

RESULT_BAG_PATH="$1"
INPUT_BAG_PATH="$2"
TRAJECTORY_TOPIC="$3"
OUTPUT_DIR="$4"
MAP_PATH="${5:-}"

if [ ! -f "${RESULT_BAG_PATH}" ]; then
    echo "Error: Result bag not found: ${RESULT_BAG_PATH}"
    exit 1
fi

if [ ! -d "${INPUT_BAG_PATH}" ]; then
    echo "Error: Input bag directory not found: ${INPUT_BAG_PATH}"
    exit 1
fi

mkdir -p "${OUTPUT_DIR}"
DEBUG_DIR="${OUTPUT_DIR}/or_debug_images"
JSON_PATH="${OUTPUT_DIR}/or_results.json"
EVAL_BAG_PATH="${OUTPUT_DIR}/evaluation_output.bag"

echo "=========================================="
echo "OR Scene Evaluation"
echo "=========================================="
echo "Result bag (LIVE): ${RESULT_BAG_PATH}"
echo "Input bag (OR events): ${INPUT_BAG_PATH}"
echo "Trajectory topic: ${TRAJECTORY_TOPIC}"
echo "Output directory: ${OUTPUT_DIR}"
echo ""

source ~/pilot-auto/install/setup.bash

rm -rf "${DEBUG_DIR}"
rm -rf "${EVAL_BAG_PATH}"

# Build ROS 2 parameters
ROS_PARAMS="-p bag_path:=${RESULT_BAG_PATH} \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=${TRAJECTORY_TOPIC} \
  -p or_scene_evaluation.input_bag_path:=${INPUT_BAG_PATH} \
  -p or_scene_evaluation.enable_debug_visualization:=true \
  -p or_scene_evaluation.debug_output_dir:=${DEBUG_DIR} \
  -p json_output_path:=${JSON_PATH} \
  -p evaluation_output_bag_path:=${EVAL_BAG_PATH}"

# Add map path if provided
if [ -n "${MAP_PATH}" ] && [ -f "${MAP_PATH}" ]; then
    echo "  Map file: ${MAP_PATH}"
    ROS_PARAMS="${ROS_PARAMS} -p or_scene_evaluation.map_path:=${MAP_PATH}"
fi

ros2 run autoware_offline_evaluation_tools offline_evaluator_node --ros-args ${ROS_PARAMS}

echo ""
echo "=========================================="
echo "Evaluation Complete!"
echo "=========================================="
echo "Results saved to:"
echo "  - JSON: ${JSON_PATH}"
echo "  - Images: ${DEBUG_DIR}/"
echo "  - Evaluation bag: ${EVAL_BAG_PATH}/"
echo ""

if [ -f "${JSON_PATH}" ]; then
    echo "Summary:"
    cat ${JSON_PATH} | python3 -c "import sys, json; data=json.load(sys.stdin); print(f\"  Mean ADE: {data['summary']['ade']['mean']:.3f}m (±{data['summary']['ade']['std']:.3f}m)\"); print(f\"  Mean FDE: {data['summary']['fde']['mean']:.3f}m (±{data['summary']['fde']['std']:.3f}m)\"); print(f\"  Total OR events: {data['summary']['total_or_events']}\"); print(f\"  Events with predictions: {data['summary']['events_with_valid_predictions']}\")" 2>/dev/null || echo "  (JSON parsing failed, check file manually)"
fi
