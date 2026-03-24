#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
IMAGE_TAG="${IMAGE_TAG:-pallas-smoke:${ROS_DISTRO}}"

docker build --build-arg ROS_DISTRO="${ROS_DISTRO}" -t "${IMAGE_TAG}" .

docker run --rm \
  -e ROS_DISTRO="${ROS_DISTRO}" \
  "${IMAGE_TAG}" \
  bash -lc '
    source /opt/ros/$ROS_DISTRO/setup.bash &&
    python3 scripts/make_demo_package.py package --demo ouster-core-demo --overwrite &&
    export PALLAS_DEMO_SOURCE_DIR=/workspace/artifacts/demo_assets &&
    uv run pallas-dev doctor &&
    uv run pallas-dev preset-check &&
    uv run pallas-dev demo-fetch ouster-core-demo &&
    uv run pytest &&
    uv run ruff check python tests &&
    uv run pallas-dev build &&
    uv run pallas-dev demo-replay ouster-core-demo --dry-run &&
    uv run python scripts/demo_benchmark.py ouster-core-demo --run-replay &&
    uv run pallas-dev test &&
    colcon test-result --verbose
  '
