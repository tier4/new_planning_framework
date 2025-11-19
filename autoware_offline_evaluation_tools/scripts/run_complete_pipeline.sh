#!/bin/bash
# Complete DLR + Evaluation Pipeline with Multi-Run Support
# Usage: ./run_complete_pipeline.sh <scenario_path> <output_base_dir> <live_prefix> [previous_prefixes] [input_bag_override]

set -e

if [ $# -lt 3 ]; then
    echo "Usage: $0 <scenario_path> <output_base_dir> <live_prefix> [previous_prefixes] [input_bag_override]"
    echo ""
    echo "Arguments:"
    echo "  scenario_path      - Path to T4 dataset scenario.yaml"
    echo "  output_base_dir    - Base directory for outputs"
    echo "  live_prefix        - Prefix for this run (e.g., 'model_v1.0', 'baseline')"
    echo "  previous_prefixes  - Optional: Comma-separated previous run prefixes (for iteration 2+)"
    echo "  input_bag_override - Optional: Use specific bag instead of scenario's input_bag"
    echo ""
    echo "Examples:"
    echo "  # First run"
    echo "  $0 /path/scenario.yaml /path/output model_v1.0"
    echo ""
    echo "  # Second run (uses first result)"
    echo "  $0 /path/scenario.yaml /path/output model_v2.0 model_v1.0 /path/output/run1/result_bag"
    exit 1
fi

SCENARIO_PATH="$1"
OUTPUT_BASE_DIR="$2"
LIVE_PREFIX="$3"
PREVIOUS_PREFIXES="${4:-}"
INPUT_BAG_OVERRIDE="${5:-}"

if [ ! -f "${SCENARIO_PATH}" ]; then
    echo "Error: Scenario file not found: ${SCENARIO_PATH}"
    exit 1
fi

DATASET_DIR=$(dirname "${SCENARIO_PATH}")
MAP_PATH="${DATASET_DIR}/map/lanelet2_map.osm"
ORIGINAL_INPUT_BAG="${DATASET_DIR}/input_bag"

# Determine output directory for this run
RUN_OUTPUT="${OUTPUT_BASE_DIR}/${LIVE_PREFIX}"
mkdir -p "${RUN_OUTPUT}"

echo "=========================================="
echo "Complete DLR + Evaluation Pipeline"
echo "=========================================="
echo "Scenario: ${SCENARIO_PATH}"
echo "Output: ${RUN_OUTPUT}"
echo "Live prefix: ${LIVE_PREFIX}"
if [ -n "${PREVIOUS_PREFIXES}" ]; then
    echo "Previous runs: ${PREVIOUS_PREFIXES}"
fi
if [ -n "${INPUT_BAG_OVERRIDE}" ]; then
    echo "Input bag: ${INPUT_BAG_OVERRIDE}"
fi
echo ""

source ~/pilot-auto/install/setup.bash

# Step 1: Run DLR
echo "=========================================="
echo "Step 1: Running DLR Simulation"
echo "=========================================="

DLR_CMD="ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${SCENARIO_PATH} \
  output_dir:=${RUN_OUTPUT}/dlr_output \
  live_topic_prefix:=${LIVE_PREFIX}"

if [ -n "${PREVIOUS_PREFIXES}" ]; then
    DLR_CMD="${DLR_CMD} previous_live_prefixes:=${PREVIOUS_PREFIXES}"
fi

if [ -n "${INPUT_BAG_OVERRIDE}" ]; then
    DLR_CMD="${DLR_CMD} input_bag:=${INPUT_BAG_OVERRIDE}"
fi

echo "Running: ${DLR_CMD}"
echo ""

eval ${DLR_CMD}

echo ""
echo "=========================================="
echo "Step 2: Verifying DLR Output"
echo "=========================================="

RESULT_BAG="${RUN_OUTPUT}/dlr_output/out/latest/result_bag/result_bag_0.mcap"
if [ ! -f "${RESULT_BAG}" ]; then
    echo "Error: Result bag not found at ${RESULT_BAG}"
    exit 1
fi

echo "Result bag: ${RESULT_BAG}"
echo "Checking topics..."
ros2 bag info "${RESULT_BAG}" | grep -E "${LIVE_PREFIX}|kinematic_state" | head -5
echo ""

# Step 2: Run Evaluation
echo "=========================================="
echo "Step 3: Running OR Scene Evaluation"
echo "=========================================="

EVAL_OUTPUT="${RUN_OUTPUT}/evaluation"
mkdir -p "${EVAL_OUTPUT}"

TRAJECTORY_TOPIC="/${LIVE_PREFIX}/planning/trajectory_generator/diffusion_planner_node/output/trajectory"

echo "Evaluating topic: ${TRAJECTORY_TOPIC}"
echo "Output: ${EVAL_OUTPUT}"
echo ""

bash "$(dirname "$0")/run_evaluation.sh" \
  "${RESULT_BAG}" \
  "${ORIGINAL_INPUT_BAG}" \
  "${TRAJECTORY_TOPIC}" \
  "${EVAL_OUTPUT}" \
  "${MAP_PATH}"

echo ""
echo "=========================================="
echo "Pipeline Complete for: ${LIVE_PREFIX}"
echo "=========================================="
echo ""
echo "Outputs:"
echo "  DLR result bag: ${RESULT_BAG}"
echo "  Evaluation JSON: ${EVAL_OUTPUT}/or_results.json"
echo "  Visualizations: ${EVAL_OUTPUT}/or_debug_images/"
echo ""

if [ -f "${EVAL_OUTPUT}/or_results.json" ]; then
    echo "Metrics:"
    cat "${EVAL_OUTPUT}/or_results.json" | python3 -c "import sys, json; d=json.load(sys.stdin); print(f\"  Mean ADE: {d['summary']['ade']['mean']:.3f}m (±{d['summary']['ade']['std']:.3f}m)\"); print(f\"  Mean FDE: {d['summary']['fde']['mean']:.3f}m (±{d['summary']['fde']['std']:.3f}m)\")" 2>/dev/null || echo "  (See JSON for results)"
fi

echo ""
echo "For next iteration, use:"
echo "  $0 ${SCENARIO_PATH} ${OUTPUT_BASE_DIR} <new_prefix> ${LIVE_PREFIX}$([ -n \"${PREVIOUS_PREFIXES}\" ] && echo \",${PREVIOUS_PREFIXES}\" || echo \"\") ${RESULT_BAG%/*}"
