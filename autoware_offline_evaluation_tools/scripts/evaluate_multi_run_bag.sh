#!/bin/bash
# Evaluate all LIVE runs from a config file
# Usage: ./evaluate_multi_run_bag.sh <final_bag> <input_bag> <eval_config.yaml> <output_dir> <map_path>

set -e

if [ $# -lt 4 ]; then
    echo "Usage: $0 <final_bag> <input_bag> <eval_config.yaml> <output_dir> [map_path]"
    echo ""
    echo "Arguments:"
    echo "  final_bag     - Final result_bag with all LIVE trajectories"
    echo "  input_bag     - Original input bag (for OR event extraction)"
    echo "  eval_config   - YAML config file listing runs to evaluate"
    echo "  output_dir    - Directory for evaluation outputs"
    echo "  map_path      - Optional: Path to lanelet2_map.osm"
    echo ""
    echo "Config file format (eval_config.yaml):"
    echo "  base_trajectory_topic: /planning/.../trajectory"
    echo "  runs:"
    echo "    - prefix: model_v1.0_epoch50"
    echo "      description: 'First model'"
    echo "    - prefix: model_v2.0_epoch100"
    echo "      description: 'Improved model'"
    exit 1
fi

FINAL_BAG="$1"
INPUT_BAG="$2"
EVAL_CONFIG="$3"
OUTPUT_DIR="$4"
MAP_PATH="${5:-}"

if [ ! -f "${FINAL_BAG}" ]; then
    echo "Error: Final bag not found: ${FINAL_BAG}"
    exit 1
fi

if [ ! -d "${INPUT_BAG}" ]; then
    echo "Error: Input bag not found: ${INPUT_BAG}"
    exit 1
fi

if [ ! -f "${EVAL_CONFIG}" ]; then
    echo "Error: Config file not found: ${EVAL_CONFIG}"
    exit 1
fi

mkdir -p "${OUTPUT_DIR}"

echo "=========================================="
echo "Multi-Run Evaluation"
echo "=========================================="
echo "Final bag: ${FINAL_BAG}"
echo "Input bag: ${INPUT_BAG}"
echo "Config: ${EVAL_CONFIG}"
echo "Output: ${OUTPUT_DIR}"
echo ""

source ~/pilot-auto/install/setup.bash

# Run Python evaluator
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/multi_run_evaluator.py" \
  --final-bag "${FINAL_BAG}" \
  --input-bag "${INPUT_BAG}" \
  --config "${EVAL_CONFIG}" \
  --output-dir "${OUTPUT_DIR}" \
  --map-path "${MAP_PATH}"
