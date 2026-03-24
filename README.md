# project_pallas_ros2

`PALLAS` is a clean-room commercial-friendly rebuild of a continuous-time LiDAR-inertial
odometry stack for ROS2 inside the ANIMA ecosystem.

Recommended expansion:

`PALLAS` = `Probabilistic Alignment for LiDAR Localization And State-fusion`

## Why This Exists

The upstream repository that motivated this effort is under a custom non-commercial license.
This repository does not copy that implementation. It is a fresh codebase built from a
requirements-level understanding of the problem:

- ROS2 LiDAR + IMU ingestion
- high-rate IMU propagation
- local odometry output
- rolling local map maintenance
- modular interfaces so we can evolve toward continuous-time spline optimization

## Repo Layout

```text
project_pallas_ros2/
  pyproject.toml
  config/
    colcon.defaults.yaml
  docs/
    architecture.md
    clean_room.md
  python/
    pallas_dev/
  ros2_ws/
    src/
      anima_pallas_ros2/
  scripts/
  tests/
```

`uv` manages the developer environment, validation tooling, and wrappers.
`colcon` manages the ROS2 package build under `ros2_ws/src`.

## Quick Start

```bash
cd /Users/ilessio/Development/AIFLOWLABS/projects/ROS2/project_pallas_ros2
uv sync --group dev
uv run pallas-dev doctor

source /opt/ros/$ROS_DISTRO/setup.bash
export COLCON_DEFAULTS_FILE=$PWD/config/colcon.defaults.yaml
colcon build --base-paths ros2_ws/src
source ros2_ws/install/setup.bash
ros2 launch anima_pallas_ros2 pallas.launch.py
```

## Current State

This first pass is intentionally a modular baseline:

- original repo and package naming removed
- commercial-friendly Apache-2.0 license
- no `Sophus`, `yaml-cpp`, or `pcl` dependency in the core baseline
- generic `PointCloud2` parser with per-point time-field support
- IMU stationary initialization + propagation
- rolling voxelized local map

The next iteration should focus on:

1. spline trajectory state
2. scan-to-plane residuals
3. IMU residual linearization
4. solver and benchmark harnesses
