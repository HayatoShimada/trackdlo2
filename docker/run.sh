#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

GPU="${1:-amd}"
MODE="${2:-sim}"

if [[ "$GPU" != "amd" && "$GPU" != "nvidia" ]]; then
  echo "Usage: $0 [amd|nvidia] [sim|realsense] [options...]"
  echo ""
  echo "  GPU:"
  echo "    amd    - Use AMD GPU (default)"
  echo "    nvidia - Use NVIDIA GPU"
  echo ""
  echo "  Mode:"
  echo "    sim                  - Gazebo simulation (default)"
  echo "    realsense            - RealSense camera only (HSV segmentation)"
  echo "    realsense:hsv_tuner  - RealSense + HSV tuner GUI"
  echo "    realsense:sam2       - RealSense + SAM2 segmentation"
  echo ""
  echo "  Examples:"
  echo "    $0 amd                        # Simulation with AMD GPU"
  echo "    $0 nvidia realsense           # RealSense test with NVIDIA GPU"
  echo "    $0 amd realsense:hsv_tuner    # RealSense + HSV tuner"
  echo "    $0 nvidia realsense:sam2 -d   # RealSense + SAM2 (detached)"
  exit 1
fi

# Parse mode and segmentation
IFS=':' read -r BASE_MODE SEGMENTATION <<< "$MODE"
SEGMENTATION="${SEGMENTATION:-hsv}"

if [[ "$BASE_MODE" == "sim" ]]; then
  echo "=== Starting simulation with ${GPU} GPU support ==="
  docker compose -f docker-compose.yml -f "docker-compose.${GPU}.yml" up "${@:3}"

elif [[ "$BASE_MODE" == "realsense" ]]; then
  echo "=== Starting RealSense test (segmentation=${SEGMENTATION}) with ${GPU} GPU ==="
  SEGMENTATION="$SEGMENTATION" \
    docker compose \
      -f docker-compose.realsense.yml \
      -f "docker-compose.realsense.${GPU}.yml" \
      up "${@:3}"

else
  echo "Error: Unknown mode '${BASE_MODE}'. Use 'sim' or 'realsense'."
  exit 1
fi
