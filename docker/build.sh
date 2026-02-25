#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="${1:-all}"

echo "=== Step 1: Building base image ==="
docker compose build trackdlo-base

if [[ "$TARGET" == "all" || "$TARGET" == "sim" ]]; then
  echo "=== Building simulation images ==="
  docker compose build trackdlo-gazebo trackdlo-perception trackdlo-moveit
fi

if [[ "$TARGET" == "all" || "$TARGET" == "realsense" ]]; then
  echo "=== Building RealSense image ==="
  docker compose -f docker-compose.realsense.yml build trackdlo-realsense
fi

if [[ "$TARGET" == "realsense-sam2" ]]; then
  echo "=== Building RealSense image with SAM2 (CPU) ==="
  docker compose -f docker-compose.realsense.yml build \
    --build-arg INSTALL_SAM2=cpu trackdlo-realsense
fi

if [[ "$TARGET" == "realsense-sam2-cuda" ]]; then
  echo "=== Building RealSense image with SAM2 (CUDA) ==="
  docker compose -f docker-compose.realsense.yml build \
    --build-arg INSTALL_SAM2=cuda trackdlo-realsense
fi

echo "=== Build complete ==="
