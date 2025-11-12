#!/bin/bash
set -e

if [ $# -lt 3 ]; then
    echo "Usage: $0 <scenario_path> <trajectory_topic> <output_dir> [map_path]"
    echo ""
    echo "Arguments:"
    echo "  scenario_path     - Path to T4 dataset scenario.yaml"
    echo "  trajectory_topic  - Trajectory topic to evaluate"
    echo "  output_dir        - Base directory for all outputs"
    echo "  map_path          - Optional: Path to lanelet2_map.osm"
    exit 1
fi

SCENARIO_PATH="$1"
TRAJECTORY_TOPIC="$2"
OUTPUT_BASE_DIR="$3"
MAP_PATH="${4:-}"

if [ ! -f "${SCENARIO_PATH}" ]; then
    echo "Error: Scenario file not found: ${SCENARIO_PATH}"
    exit 1
fi

DATASET_DIR=$(dirname "${SCENARIO_PATH}")
INPUT_BAG_DIR="${DATASET_DIR}/input_bag"

if [ ! -d "${INPUT_BAG_DIR}" ]; then
    echo "Error: Input bag directory not found: ${INPUT_BAG_DIR}"
    exit 1
fi

mkdir -p "${OUTPUT_BASE_DIR}"

echo "=========================================="
echo "DLR + Evaluation Pipeline"
echo "=========================================="
echo "Dataset: ${DATASET_DIR}"
echo "Scenario: ${SCENARIO_PATH}"
echo "Trajectory topic: ${TRAJECTORY_TOPIC}"
echo "Output: ${OUTPUT_BASE_DIR}"
if [ -n "${MAP_PATH}" ]; then
    echo "Map file: ${MAP_PATH}"
fi
echo ""

source ~/pilot-auto/install/setup.bash

echo "Step 1: Cleaning up previous outputs..."
rm -rf "${DATASET_DIR}/out"
rm -rf "${OUTPUT_BASE_DIR}"
mkdir -p "${OUTPUT_BASE_DIR}"
echo "  ✓ Cleanup complete"

echo ""
echo "Step 2: Running DLR simulation..."
echo "  This will take several minutes..."
echo ""

ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${SCENARIO_PATH}

echo ""
echo "=========================================="
echo "DLR Complete - Starting Evaluation"
echo "=========================================="

RESULT_BAG="${DATASET_DIR}/out/latest/result_bag/result_bag_0.mcap"
if [ ! -f "${RESULT_BAG}" ]; then
    echo "Error: Result bag not found at ${RESULT_BAG}"
    exit 1
fi

echo "Result bag: ${RESULT_BAG}"
echo ""
echo "Step 3: Running OR scene evaluation..."

DEBUG_DIR="${OUTPUT_BASE_DIR}/or_debug_images"
JSON_PATH="${OUTPUT_BASE_DIR}/or_results.json"
EVAL_BAG_PATH="${OUTPUT_BASE_DIR}/evaluation_output.bag"

ROS_PARAMS="-p bag_path:=${RESULT_BAG} \
  -p evaluation.mode:=or_scene \
  -p trajectory_topic:=${TRAJECTORY_TOPIC} \
  -p or_scene_evaluation.input_bag_path:=${INPUT_BAG_DIR} \
  -p or_scene_evaluation.enable_debug_visualization:=true \
  -p or_scene_evaluation.debug_output_dir:=${DEBUG_DIR} \
  -p json_output_path:=${JSON_PATH} \
  -p evaluation_output_bag_path:=${EVAL_BAG_PATH}"

if [ -n "${MAP_PATH}" ] && [ -f "${MAP_PATH}" ]; then
    echo "Including map in visualizations: ${MAP_PATH}"
    ROS_PARAMS="${ROS_PARAMS} -p or_scene_evaluation.map_path:=${MAP_PATH}"
fi

ros2 run autoware_offline_evaluation_tools offline_evaluator_node --ros-args ${ROS_PARAMS}

echo ""
echo "=========================================="
echo "Pipeline Complete!"
echo "=========================================="
echo ""
echo "Results:"
echo "  - DLR output: ${DATASET_DIR}/out/latest/"
echo "  - JSON: ${JSON_PATH}"
echo "  - Visualizations: ${DEBUG_DIR}/"
echo ""

if [ -f "${JSON_PATH}" ]; then
    echo "Metrics Summary:"
    cat ${JSON_PATH} | python3 -c "import sys, json; data=json.load(sys.stdin); print(f\"  Mean ADE: {data['summary']['ade']['mean']:.3f}m (±{data['summary']['ade']['std']:.3f}m)\"); print(f\"  Mean FDE: {data['summary']['fde']['mean']:.3f}m (±{data['summary']['fde']['std']:.3f}m)\"); print(f\"  Total OR events: {data['summary']['total_or_events']}\"); print(f\"  Events with predictions: {data['summary']['events_with_valid_predictions']}\")" 2>/dev/null || cat ${JSON_PATH} | grep -A 5 '"summary"' | head -10
fi
