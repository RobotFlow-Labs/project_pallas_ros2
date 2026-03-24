# project_pallas_ros2

`PALLAS` is an ANIMA ROS2 module for LiDAR-inertial odometry and local mapping.

Recommended expansion:

`PALLAS` = `Probabilistic Alignment for LiDAR Localization And State-fusion`

## Overview

PALLAS provides a modular estimation stack for robots that need:

- LiDAR and IMU ingestion in ROS2
- realtime odometry from high-rate inertial propagation
- scan alignment into a locally consistent frame
- rolling local map maintenance
- a research path toward continuous-time trajectory modeling

The package is built around a portable C++/Eigen core, with ROS2 used as the integration layer for topics, TF, launch, and deployment.

## Functional Scope

PALLAS currently implements these runtime capabilities:

- `PointCloud2` ingestion with flexible field-name handling for `x`, `y`, `z`, `intensity`, `ring` or `line`, and per-point time
- range clipping and relative-time normalization during scan ingest
- stationary IMU bootstrap to estimate initial attitude and sensor biases
- bias-aware strapdown propagation for pose and velocity tracking
- voxelized scan sampling
- local normal estimation from neighborhood covariance
- rolling surfel-map integration for local mapping
- ROS2 pose, odometry, aligned scan, and map publication
- TF publication from odometry frame to base frame

## Runtime Profiles

PALLAS exposes two explicit runtime profiles.

### PALLAS Core

Realtime runtime for deployment-oriented robotics workloads.

- portable C++/Eigen implementation
- lower complexity and smaller dependency surface
- scan ingest, IMU bootstrap, strapdown tracking, scan sampling, and local surfel-map maintenance
- intended for live robot execution and benchmarking

Default launch:

```bash
ros2 launch anima_pallas_ros2 pallas_core.launch.py
```

Primary config:

`ros2_ws/src/anima_pallas_ros2/config/pallas_core.yaml`

### PALLAS CT

Research runtime for continuous-time trajectory experimentation.

- includes the Core runtime path
- adds spline-backed output smoothing through the continuous-time layer
- intended for research iteration, evaluation, and model refinement

Launch:

```bash
ros2 launch anima_pallas_ros2 pallas_ct.launch.py
```

Primary config:

`ros2_ws/src/anima_pallas_ros2/config/pallas_ct.yaml`

## Sensor Preset Pack

PALLAS now ships with vendor-oriented config presets so integrators can start
from the topic and frame conventions used by common ROS2 LiDAR drivers instead
of rewriting parameters from scratch.

Included Core and CT presets:

- `generic`: `pallas_core.yaml`, `pallas_ct.yaml`
- `Unitree`: `pallas_core_unitree.yaml`, `pallas_ct_unitree.yaml`
- `Livox`: `pallas_core_livox.yaml`, `pallas_ct_livox.yaml`
- `Ouster`: `pallas_core_ouster.yaml`, `pallas_ct_ouster.yaml`
- `Hesai`: `pallas_core_hesai.yaml`, `pallas_ct_hesai.yaml`
- `RoboSense`: `pallas_core_robosense.yaml`, `pallas_ct_robosense.yaml`
- `RoboSense alias`: `pallas_core_rslidar.yaml`, `pallas_ct_rslidar.yaml`
- `Velodyne`: `pallas_core_velodyne.yaml`, `pallas_ct_velodyne.yaml`

Use any preset through the same launch file:

```bash
ros2 launch anima_pallas_ros2 pallas_core.launch.py config_name:=pallas_core_unitree.yaml
ros2 launch anima_pallas_ros2 pallas_ct.launch.py config_name:=pallas_ct_ouster.yaml
```

Preset intent:

- `Unitree`: starts from `unilidar/cloud` and `unilidar/imu`
- `Livox`: starts from `livox/lidar` and `livox/imu`
- `Ouster`: starts from `/ouster/points` and `/ouster/imu`
- `Hesai`: starts from `/lidar_points` and `/lidar_imu`
- `RoboSense`: starts from `/rslidar_points` and `/rslidar_imu_data`
- `Velodyne`: starts from `/velodyne_points` and expects an external IMU on `/imu/data`

These presets are integration starting points, not a substitute for calibration.
For any sensor that publishes LiDAR and IMU in different frames, you still need
to set `sensor_to_body_translation` and `sensor_to_body_rpy` to your measured
LiDAR-to-IMU extrinsics.

## ROS2 Interfaces

Both runtime profiles expose the same functional interface pattern with profile-specific topic prefixes.

Inputs:

- `pointcloud_topic`
- `imu_topic`

Outputs:

- `pose_topic`
- `odom_topic`
- `aligned_scan_topic`
- `map_topic`

Frames:

- `odom_frame`
- `base_frame`

The node also publishes the odometry-to-base transform through TF.

## Configuration Surface

The runtime is parameterized through ROS2 parameters.

Core estimation parameters:

- `lidar_type`
- `stationary_init_sec`
- `gravity_mps2`
- `min_range_m`
- `max_range_m`

Scan processing parameters:

- `scan_voxel_size_m`
- `scan_normal_radius_m`
- `scan_min_points_for_normal`

Map parameters:

- `map_voxel_size_m`
- `map_max_surfels`
- `map_max_age_sec`
- `map_publish_period_scans`

Continuous-time parameters:

- `ct_min_control_points`
- `ct_max_control_points`

Extrinsics:

- `sensor_to_body_translation`
- `sensor_to_body_rpy`

## Platform Support

The core runtime is intentionally portable.

- primary implementation: C++17 + Eigen + ROS2
- suitable target classes: Jetson, Apple Silicon environments with ROS2, and standard x86 Linux systems
- MLX is not required for the current odometry stack
- MLX remains a valid future path for optional learned or perception-heavy accelerators on Apple Silicon

## Repository Layout

```text
project_pallas_ros2/
  pyproject.toml
  uv.lock
  config/
    colcon.defaults.yaml
  docs/
    apple_silicon.md
    architecture.md
    clean_room.md
    runtime_profiles.md
  python/
    pallas_dev/
  ros2_ws/
    src/
      anima_pallas_ros2/
  scripts/
  tests/
```

## Developer Workflow

`uv` manages the Python developer environment and helper tooling. `colcon` builds the ROS2 workspace.

Bootstrap the environment:

```bash
cd /Users/ilessio/Development/AIFLOWLABS/projects/ROS2/project_pallas_ros2
uv sync --group dev
uv run pallas-dev doctor
```

Build the ROS2 package:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
export COLCON_DEFAULTS_FILE=$PWD/config/colcon.defaults.yaml
colcon build --base-paths ros2_ws/src
source ros2_ws/install/setup.bash
```

Run the Core profile:

```bash
ros2 launch anima_pallas_ros2 pallas_core.launch.py
```

Run the CT profile:

```bash
ros2 launch anima_pallas_ros2 pallas_ct.launch.py
```

Run a vendor preset:

```bash
ros2 launch anima_pallas_ros2 pallas_core.launch.py config_name:=pallas_core_unitree.yaml
ros2 launch anima_pallas_ros2 pallas_core.launch.py config_name:=pallas_core_velodyne.yaml
```

## Validation

Repository-level validation available through the current tooling includes:

- `uv run pytest`
- `uv run ruff check python tests`
- launch-file syntax checks through Python compilation

Full ROS2 validation should additionally include:

- `colcon build`
- node startup checks
- bag replay on representative LiDAR/IMU data
- topic, TF, and map-output verification

## Near-Term Engineering Priorities

The current stack is a strong baseline, but the next implementation passes should focus on:

1. replacing CT output smoothing with full continuous-time residual optimization
2. adding scan-to-surface residuals over the surfel map
3. introducing IMU residual linearization and solver loops
4. adding repeatable benchmarking and bag-replay metrics
