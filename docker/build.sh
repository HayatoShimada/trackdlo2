#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Step 1/2: Building base image ==="
docker compose build trackdlo-base

echo "=== Step 2/2: Building all services ==="
docker compose build trackdlo-gazebo trackdlo-perception trackdlo-moveit trackdlo-viz

echo "=== Build complete ==="
