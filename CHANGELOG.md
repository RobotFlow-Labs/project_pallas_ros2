# Changelog

All notable changes to PALLAS will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Thread-safe callback group for multi-threaded executor compatibility
- Zip Slip protection for demo archive extraction
- HTTPS enforcement and download size limits for demo fetches
- Negative parameter validation with warnings in ROS2 node
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` for `PoseState` struct
- `builtin_interfaces` and `eigen` as explicit package dependencies
- Quaternion hemisphere check for shortest-path slerp interpolation
- `SECURITY.md`, `CHANGELOG.md`, `CODE_OF_CONDUCT.md`

### Fixed
- CT runtime config clamped before core construction (was post-construction)
- Subprocess leak in `run_demo_replay` on exception paths
- `seen_topics` now correctly updated during verification polling
- `config-check` no longer crashes on non-dict YAML files
- CMake no longer hardcodes `-O3`, allowing proper debug builds
- Hardcoded developer path in `gazebo_lidar_test.sh` replaced with env var
- Redundant `CullExpired` call removed from `SurfelVolume::Integrate`
- `format` parameter renamed to avoid shadowing Python builtin

### Removed
- Unused `pydantic` dependency from `pyproject.toml`

## [0.1.0] - 2025-05-01

### Added
- Clean-room PALLAS ROS2 repository bootstrap
- Core runtime: lean LiDAR-inertial odometry with Eigen3
- CT runtime: continuous-time spline-backed smoothing
- 19 sensor presets across 9 LiDAR vendors (Core + CT)
- `pallas-dev` CLI: preset management, demo fetch/replay, doctor, build
- Docker smoke tests for ROS2 Humble and Jazzy
- GitHub Actions CI pipeline
- Demo asset system with synthetic Ouster bag
- Documentation: architecture, presets, quickstart, Gazebo, Apple Silicon
