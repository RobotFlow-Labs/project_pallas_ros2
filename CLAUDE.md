# PALLAS ROS2 — Project Instructions

## Overview
PALLAS (Probabilistic Alignment for LiDAR Localization And State-fusion) is an ANIMA ROS2 module for LiDAR-inertial odometry and local mapping. Apache 2.0 licensed open-source product by RobotFlow Labs.

## Architecture
```
python/pallas_dev/       # CLI tooling (Typer + Rich)
  cli.py                 # Main CLI app — preset, demo, build, doctor commands
  presets.py             # Sensor preset discovery & validation
  rosenv.py              # ROS2 environment detection & command wrapping
  demo.py                # Demo asset fetch/replay/benchmark
ros2_ws/src/anima_pallas_ros2/
  include/               # C++17 headers (types, bootstrap, ingest, tracker, sampler, volume, spline)
  src/                   # C++ implementations — two runtimes: core (lean) + ct (spline-backed)
  config/                # 19 YAML sensor presets (9 vendors x 2 profiles)
  launch/                # ROS2 Python launch files
tests/                   # pytest suite (22 tests)
scripts/                 # Shell utilities, Docker smoke, Gazebo test scripts
docs/                    # Architecture, presets, quickstart, Gazebo, Apple Silicon docs
```

## Key patterns
- **Two runtimes**: `pallas_core_runtime` (production) and `pallas_ct_runtime` (research/continuous-time)
- **Preset-driven**: sensor configs follow `pallas_<profile>_<vendor>.yaml` naming
- **Demo-driven onboarding**: synthetic bags for evaluation without hardware
- **Dependency-minimal C++**: only Eigen3 + ROS2 msgs (no Ceres/GTSAM)

## Dev commands
```bash
uv sync --group dev          # Install Python deps
uv run pytest                # Run tests
uv run ruff check python tests  # Lint
uv run ruff format --check python tests  # Format check
uv run pallas-dev build      # Build ROS2 workspace
uv run pallas-dev preset-check  # Validate all presets
uv run pallas-dev doctor     # Environment diagnostic
./scripts/docker_smoke.sh    # Full CI simulation (Humble + Jazzy)
```

## Versions
- Python: >=3.10, <3.13 (uv managed)
- C++: C++17
- ROS2: Humble / Jazzy
- Build: hatchling (Python), ament_cmake (ROS2)

## Conventions
- `rg` over `grep` in all shell commands
- Line length: 100 (ruff)
- Target Python version: 3.10
- Compiler flags: `-Wall -Wextra -Wpedantic -O3`
- Package manager: `uv` (never pip directly)

## CI
GitHub Actions: Python tooling (pytest, ruff, preset-check) + Docker smoke (Humble/Jazzy matrix)

## Repo
- Origin: `github.com/RobotFlow-Labs/project_pallas_ros2`
- Default branch: `develop`
- License: Apache 2.0
