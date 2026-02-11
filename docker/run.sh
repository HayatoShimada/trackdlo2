#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

GPU="${1:-amd}"

if [[ "$GPU" != "amd" && "$GPU" != "nvidia" ]]; then
  echo "Usage: $0 [amd|nvidia] [docker compose up options...]"
  echo "  amd    - Use AMD GPU (default)"
  echo "  nvidia - Use NVIDIA GPU"
  exit 1
fi

echo "=== Starting with ${GPU} GPU support ==="
docker compose -f docker-compose.yml -f "docker-compose.${GPU}.yml" up "${@:2}"
