#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <preset.yaml>"
  echo "example: $0 pallas_core_ouster.yaml"
  exit 1
fi

PRESET="$1"

if [[ ! -f install/setup.bash ]]; then
  echo "[pallas] workspace not built yet, running colcon build..."
  uv run pallas-dev build
fi

uv run pallas-dev ros-check "${PRESET}"
exec uv run pallas-dev launch-live "${PRESET}" --skip-ros-check
