#!/bin/bash
#
# Prepare OR Segment Bags (without DLR)
#
# Automates:
# 1. Detecting OR events from rosbag
# 2. Cutting rosbag into OR segments (with route injection)
# 3. Adding ground truth trajectories to each segment
#
# Output: Self-contained segment bags ready for analysis or DLR processing
#

set -e  # Exit on error
set -o pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
die() {
    echo -e "${RED}ERROR: $1${NC}" >&2
    exit 1
}

info() {
    echo -e "${GREEN}$1${NC}"
}

warn() {
    echo -e "${YELLOW}WARNING: $1${NC}"
}

# Parse arguments
usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Required:
  --input INPUT_BAG     Path to input rosbag (directory or .mcap file)
  --output OUTPUT_DIR   Output directory for OR segments

Optional:
  --before-or SECONDS   Seconds before OR to include in segment (default: 5)
  --after-or SECONDS    Seconds after OR to include in segment (default: 10)
  --min-after SECONDS   Minimum seconds after OR required (default: 8)
  --gt-horizon SECONDS  Ground truth trajectory horizon (default: 8.0)
  --gt-resolution SEC   Ground truth trajectory resolution (default: 0.1)
  --help                Show this help message

Description:
  Detects override (OR) events in a rosbag and creates self-contained segment
  bags around each OR event. Each segment includes:
    - Original sensor/perception/localization data
    - Injected route message (for planning node initialization)
    - Ground truth trajectory (generated from kinematic states)

  Output structure:
    OUTPUT_DIR/
    ├── or_events.json           # OR event metadata
    ├── route_message.bin        # Extracted route for injection
    └── segments/
        ├── or_event_0_with_gt/  # Segment bag for OR event #0
        ├── or_event_1_with_gt/  # Segment bag for OR event #1
        └── ...

Example:
  $0 --input ~/rosbags/recording.db3 --output ~/or_segments
  $0 --input ~/t4_dataset/input_bag --output ~/or_segments --before-or 10
EOF
}

# Default values
BEFORE_OR=5
AFTER_OR=10
MIN_AFTER=8
GT_HORIZON=8.0
GT_RESOLUTION=0.1

# Parse command line
while [[ $# -gt 0 ]]; do
    case $1 in
        --input)
            INPUT_BAG="$2"
            shift 2
            ;;
        --output)
            OUTPUT="$2"
            shift 2
            ;;
        --before-or)
            BEFORE_OR="$2"
            shift 2
            ;;
        --after-or)
            AFTER_OR="$2"
            shift 2
            ;;
        --min-after)
            MIN_AFTER="$2"
            shift 2
            ;;
        --gt-horizon)
            GT_HORIZON="$2"
            shift 2
            ;;
        --gt-resolution)
            GT_RESOLUTION="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Validate required arguments
[[ -z "$INPUT_BAG" ]] && die "Input bag not specified. Use --input"
[[ -z "$OUTPUT" ]] && die "Output directory not specified. Use --output"

info "=========================================="
info "Prepare OR Segment Bags"
info "=========================================="
echo ""
echo "Input: $INPUT_BAG"
echo "Output: $OUTPUT"
echo "Time window: OR - ${BEFORE_OR}s to OR + ${AFTER_OR}s (min ${MIN_AFTER}s after required)"
echo "GT horizon: ${GT_HORIZON}s"
echo "GT resolution: ${GT_RESOLUTION}s"
echo ""

# PHASE 1: Validation
info "Phase 1: Validating inputs..."

# Check input bag exists
if [[ ! -e "$INPUT_BAG" ]]; then
    die "Input bag not found: $INPUT_BAG"
fi

# Ensure ROS 2 environment is sourced
if ! command -v ros2 &> /dev/null; then
    die "ros2 command not found. Please source your ROS 2 workspace first."
fi

# Handle multiple bags in directory (merge if needed)
if [[ -d "$INPUT_BAG" ]]; then
    # Check if it's a bag directory itself (has .db3 or .mcap files)
    db3_count=$(find "$INPUT_BAG" -maxdepth 1 -name "*.db3" 2>/dev/null | wc -l)
    mcap_count=$(find "$INPUT_BAG" -maxdepth 1 -name "*.mcap" 2>/dev/null | wc -l)
    yaml_count=$(find "$INPUT_BAG" -maxdepth 1 -name "metadata.yaml" 2>/dev/null | wc -l)

    if [[ $db3_count -gt 0 ]] || [[ $mcap_count -gt 0 ]] || [[ $yaml_count -gt 0 ]]; then
        # It's a bag directory itself
        PROCESSED_INPUT="$INPUT_BAG"
        info "  ✓ Found rosbag directory"

        # Reindex if needed
        python3 "$SCRIPT_DIR/reindex_bag_if_needed.py" "$PROCESSED_INPUT" || die "Failed to reindex input bag"
    else
        # Check for subdirectories (multiple bags to merge)
        bag_count=$(find "$INPUT_BAG" -mindepth 1 -maxdepth 1 -type d | wc -l)

        if [[ $bag_count -eq 0 ]]; then
            die "No rosbag files or directories found in: $INPUT_BAG"
        elif [[ $bag_count -eq 1 ]]; then
            # Single subdirectory
            PROCESSED_INPUT=$(find "$INPUT_BAG" -mindepth 1 -maxdepth 1 -type d | head -1)
            info "  ✓ Found single rosbag subdirectory"

            # Reindex if needed
            python3 "$SCRIPT_DIR/reindex_bag_if_needed.py" "$PROCESSED_INPUT" || die "Failed to reindex input bag"
        else
            # Multiple bags - need to merge
            info "  Found $bag_count rosbags, merging..."

            # Create output directory for merged bag
            mkdir -p "$OUTPUT"
            PROCESSED_INPUT="$OUTPUT/merged_input_bag"

            # Source ros2bag_extensions if available
            if [[ -f ~/ros2_ws/install/setup.bash ]]; then
                source ~/ros2_ws/install/setup.bash
                info "  ✓ Sourced ros2bag_extensions"
            else
                warn "ros2bag_extensions not found at ~/ros2_ws/install/setup.bash"
                warn "Multi-bag merging may not work. Install with:"
                warn "  cd ~/ros2_ws/src && git clone https://github.com/tier4/ros2bag_extensions.git"
            fi

            # Reindex all input bags first
            for bag_dir in $(find "$INPUT_BAG" -mindepth 1 -maxdepth 1 -type d | sort); do
                python3 "$SCRIPT_DIR/reindex_bag_if_needed.py" "$bag_dir" || warn "Failed to reindex $bag_dir"
            done

            # Get all bag directories sorted
            bag_dirs=$(find "$INPUT_BAG" -mindepth 1 -maxdepth 1 -type d | sort)

            # Merge bags
            rm -rf "$PROCESSED_INPUT"
            ros2 bag merge -o "$PROCESSED_INPUT" $bag_dirs || die "Failed to merge rosbags"

            info "  ✓ Rosbags merged: $PROCESSED_INPUT"

            # Re-source pilot-auto if it exists (ros2_ws may have overridden it)
            if [[ -f ~/pilot-auto/install/setup.bash ]]; then
                source ~/pilot-auto/install/setup.bash
            fi
        fi
    fi
else
    # It's a file (likely .mcap)
    PROCESSED_INPUT="$INPUT_BAG"
    info "  ✓ Found rosbag file"
fi

# Extract date from input bag (for output structure)
# Try to extract from filename timestamp or use current date
INPUT_BASENAME=$(basename "$PROCESSED_INPUT")
if [[ "$INPUT_BASENAME" =~ ([0-9]{4}-[0-9]{2}-[0-9]{2}) ]]; then
    BAG_DATE="${BASH_REMATCH[1]}"
else
    BAG_DATE=$(date +%Y-%m-%d)
fi
info "  ✓ Using date: $BAG_DATE"

# Create output directory with date structure
mkdir -p "$OUTPUT/$BAG_DATE"
info "  ✓ Output directory: $OUTPUT/$BAG_DATE"

echo ""

# PHASE 2: Detect OR events and extract route
info "Phase 2: Detecting OR events and extracting route..."

python3 "$SCRIPT_DIR/detect_or_and_route.py" \
    --input "$PROCESSED_INPUT" \
    --output "$OUTPUT/$BAG_DATE/or_events.json" \
    --route-output "$OUTPUT/$BAG_DATE/route_message.bin" \
    --before "$BEFORE_OR" \
    --after "$AFTER_OR" \
    --min-after "$MIN_AFTER" \
    || die "OR detection failed"

# Read number of OR events
OR_COUNT=$(jq '.or_events | length' "$OUTPUT/$BAG_DATE/or_events.json")
info "✓ Detected $OR_COUNT OR events"

if [[ $OR_COUNT -eq 0 ]]; then
    die "No OR events found in bag. Nothing to process."
fi

echo ""

# PHASE 3: Cut segments with route injection
info "Phase 3: Cutting OR segments..."

for i in $(seq 0 $((OR_COUNT - 1))); do
    event_info=$(jq ".or_events[$i]" "$OUTPUT/$BAG_DATE/or_events.json")
    start_ns=$(echo "$event_info" | jq -r '.segment_start_ns')
    end_ns=$(echo "$event_info" | jq -r '.segment_end_ns')
    or_time=$(echo "$event_info" | jq -r '.or_timestamp_sec')

    # Create time-based directory name: or_event_X_tYYYYMMDD_HHMMSS
    or_time_int=$(printf "%.0f" "$or_time")
    time_str=$(date -d "@$or_time_int" +"%Y%m%d_%H%M%S" 2>/dev/null || echo "$(printf '%010d' $i)")
    segment_time_name="or_event_${i}_t${time_str}"
    segment_dir="$OUTPUT/$BAG_DATE/$segment_time_name"

    mkdir -p "$segment_dir"

    echo "Cutting OR event #$i (OR at t=${or_time}s)..."

    python3 "$SCRIPT_DIR/cut_or_segment_with_route.py" \
        --input "$PROCESSED_INPUT" \
        --output "$segment_dir" \
        --start "$start_ns" \
        --end "$end_ns" \
        --route-data "$OUTPUT/$BAG_DATE/route_message.bin" \
        || die "Failed to cut segment $i"

    # Reindex segment to ensure metadata.yaml exists
    python3 "$SCRIPT_DIR/reindex_bag_if_needed.py" "$segment_dir" || warn "Failed to reindex segment $i"

    info "  ✓ Segment $i created: $segment_time_name"
done

echo ""

# PHASE 4: Add GT to each segment
info "Phase 4: Adding ground truth to segments..."

for i in $(seq 0 $((OR_COUNT - 1))); do
    event_info=$(jq ".or_events[$i]" "$OUTPUT/$BAG_DATE/or_events.json")
    or_time=$(echo "$event_info" | jq -r '.or_timestamp_sec')
    or_time_int=$(printf "%.0f" "$or_time")
    time_str=$(date -d "@$or_time_int" +"%Y%m%d_%H%M%S" 2>/dev/null || echo "$(printf '%010d' $i)")
    segment_time_name="or_event_${i}_t${time_str}"
    segment_dir="$OUTPUT/$BAG_DATE/$segment_time_name"
    segment_with_gt="${segment_dir}_with_gt"

    echo "Adding GT to segment $i..."

    python3 "$SCRIPT_DIR/add_gt_trajectory_to_bag.py" \
        --input "$segment_dir" \
        --output "$segment_with_gt" \
        --horizon "$GT_HORIZON" \
        --resolution "$GT_RESOLUTION" \
        || die "Failed to add GT to segment $i"

    # Reindex segment with GT to ensure metadata.yaml exists
    python3 "$SCRIPT_DIR/reindex_bag_if_needed.py" "$segment_with_gt" || warn "Failed to reindex segment $i with GT"

    info "  ✓ GT added to segment $i"
done

echo ""

# PHASE 5: Summary
info "=========================================="
info "COMPLETE!"
info "=========================================="
echo ""
echo "Prepared $OR_COUNT OR segment bags:"
echo ""

for i in $(seq 0 $((OR_COUNT - 1))); do
    event_info=$(jq ".or_events[$i]" "$OUTPUT/$BAG_DATE/or_events.json")
    or_time=$(echo "$event_info" | jq -r '.or_timestamp_sec')
    duration=$(echo "$event_info" | jq -r '.segment_duration_sec')
    or_time_int=$(printf "%.0f" "$or_time")
    time_str=$(date -d "@$or_time_int" +"%Y%m%d_%H%M%S" 2>/dev/null || echo "$(printf '%010d' $i)")
    segment_time_name="or_event_${i}_t${time_str}"
    segment_with_gt="$OUTPUT/$BAG_DATE/${segment_time_name}_with_gt"

    echo "OR Event #$i:"
    echo "  OR timestamp: t=${or_time}s"
    echo "  Segment duration: ${duration}s"
    echo "  Directory: $BAG_DATE/${segment_time_name}_with_gt"

    # Show bag size
    if [[ -d "$segment_with_gt" ]]; then
        size=$(du -sh "$segment_with_gt" 2>/dev/null | cut -f1)
        echo "  Size: $size"

        # Verify metadata.yaml exists
        if [[ -f "$segment_with_gt/metadata.yaml" ]]; then
            echo "  ✓ metadata.yaml present"
        else
            warn "metadata.yaml missing in $segment_with_gt"
        fi
    fi

    echo ""
done

echo "Output structure (compatible with parse_rosbag_for_directory.py):"
echo "  $OUTPUT/"
echo "  └── $BAG_DATE/"
echo "      ├── or_events.json"
echo "      ├── route_message.bin"
echo "      ├── or_event_0_tYYYYMMDD_HHMMSS_with_gt/"
echo "      │   └── metadata.yaml"
echo "      ├── or_event_1_tYYYYMMDD_HHMMSS_with_gt/"
echo "      │   └── metadata.yaml"
echo "      └── ..."
echo ""

info "These segment bags are ready for:"
info "  - Processing with parse_rosbag_for_directory.py"
info "  - Direct analysis/evaluation"
info "  - DLR processing"
info "  - Manual inspection with rviz/foxglove"
echo ""

info "To process with parse_rosbag_for_directory.py:"
echo "  python3 ~/Diffusion-Planner/ros_scripts/parse_rosbag_for_directory.py \\"
echo "    $OUTPUT \\"
echo "    --save_root /path/to/output"
echo ""
info "=========================================="
