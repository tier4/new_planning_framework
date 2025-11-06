#!/bin/bash
# Script to run DLR (Driving Log Replayer) simulation
# Usage: ./run_dlr.sh <scenario_path>

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <scenario_path>"
    echo ""
    echo "Example:"
    echo "  $0 /path/to/t4_dataset/scenario.yaml"
    exit 1
fi

SCENARIO_PATH="$1"

if [ ! -f "${SCENARIO_PATH}" ]; then
    echo "Error: Scenario file not found: ${SCENARIO_PATH}"
    exit 1
fi

echo "=========================================="
echo "Running DLR Simulation"
echo "=========================================="
echo "Scenario: ${SCENARIO_PATH}"
echo ""

source ~/pilot-auto/install/setup.bash

ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py \
  scenario_path:=${SCENARIO_PATH}

echo ""
echo "DLR simulation complete!"
