#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="${1:-sim}"

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  echo "Usage: $0 [sim|realsense|realsense:hsv_tuner|realsense:sam2] [options...]"
  echo ""
  echo "  Mode:"
  echo "    sim                  - Gazebo simulation (default)"
  echo "    realsense            - RealSense camera only (HSV segmentation)"
  echo "    realsense:hsv_tuner  - RealSense + HSV tuner GUI"
  echo "    realsense:sam2       - RealSense + SAM2 segmentation"
  echo ""
  echo "  Examples:"
  echo "    $0 sim                        # Simulation with NVIDIA GPU"
  echo "    $0 realsense                  # RealSense test with NVIDIA GPU"
  echo "    $0 realsense:hsv_tuner        # RealSense + HSV tuner"
  echo "    $0 realsense:sam2 -d          # RealSense + SAM2 (detached)"
  exit 0
fi

# Parse mode and segmentation
IFS=':' read -r BASE_MODE SEGMENTATION <<< "$MODE"
SEGMENTATION="${SEGMENTATION:-hsv}"

if [[ "$BASE_MODE" == "sim" ]]; then
  echo "=== Starting simulation with NVIDIA GPU support ==="
  docker compose -f docker-compose.yml -f docker-compose.nvidia.yml up "${@:2}"

elif [[ "$BASE_MODE" == "realsense" ]]; then
  echo "=== Starting RealSense test (segmentation=${SEGMENTATION}) with NVIDIA GPU ==="
  SEGMENTATION="$SEGMENTATION" \
    docker compose \
      -f docker-compose.realsense.yml \
      -f docker-compose.realsense.nvidia.yml \
      up "${@:2}"

else
  echo "Error: Unknown mode '${BASE_MODE}'. Use 'sim' or 'realsense'."
  exit 1
fi
