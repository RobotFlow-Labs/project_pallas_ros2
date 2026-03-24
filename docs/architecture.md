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

- `point_parser`: robust extraction from `PointCloud2`
- `imu_buffer`: IMU history and stationary bootstrap
- `motion_integrator`: bias-aware strapdown propagation
- `local_map`: rolling voxel map
- `pipeline`: sensor fusion orchestration
- `pallas_node`: ROS2 node and publishers

## Planned Evolution

1. replace strapdown-only scan pose with spline control points
2. split local map into correspondence and normal estimation layers
3. add Gauss-Newton solver over spline + bias states
4. add bag replay and metric reports under `uv run pallas-dev`
