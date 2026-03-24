# PALLAS Architecture

## Design Goals

- Commercially usable from day one
- Faster inner loop than the upstream single-package layout
- Easy to benchmark, profile, and refactor
- Small dependency surface
- Clear seam between ROS glue and estimation core

## Current Modules

### `pallas_dev`

Python developer CLI managed by `uv`.

- environment checks
- config validation
- thin wrappers around `colcon`

### `anima_pallas_ros2`

ROS2 package containing the estimation baseline.

- `cloud_ingest`: robust extraction from `PointCloud2`
- `attitude_bootstrap`: stationary IMU alignment and seed-state estimation
- `strapdown_tracker`: bias-aware propagation and interpolation
- `scan_sampler`: voxel downsampling and normal estimation
- `surfel_volume`: rolling local surfel map
- `core_runtime`: lean realtime odometry and mapping path
- `ct_runtime`: spline-backed research path layered on the core runtime
- `runtime_node`: shared ROS2 node wrapper for both runtime profiles

## Planned Evolution

1. replace spline smoothing with full scan/IMU residual optimization
2. add correspondence search and residual linearization around the surfel volume
3. add solver loops, benchmark harnesses, and bag replay workflows
4. keep Apple Silicon and Jetson deployment on the same portable core
