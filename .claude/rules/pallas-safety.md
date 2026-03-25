# PALLAS Safety Rules — Zero Mistakes Protocol

## Rule: NEVER modify C++ estimation code without verification

Before editing ANY file in `ros2_ws/src/anima_pallas_ros2/`:
1. READ the file fully first
2. CHECK if the change affects the estimation pipeline (bootstrap → tracker → sampler → surfel → runtime)
3. VERIFY thread safety implications — all runtime state is protected by MutuallyExclusive callback group
4. CONFIRM Eigen alignment rules are not violated (PoseState uses EIGEN_MAKE_ALIGNED_OPERATOR_NEW)

**Why:** A subtle bug in the odometry pipeline produces silent drift — no crash, no error, just wrong poses. On a real robot this is dangerous.

## Rule: NEVER change preset YAML without preset-check

After ANY change to files in `ros2_ws/src/anima_pallas_ros2/config/`:
1. Run `uv run pallas-dev preset-check`
2. Verify the edited preset with `uv run pallas-dev preset-show <name>.yaml`
3. Confirm topic names match the vendor's actual driver output

**Why:** A wrong topic name means silent data starvation — PALLAS boots but never receives data.

## Rule: ALWAYS run tests before suggesting commit

Before any commit suggestion:
1. `uv run pytest` — all Python tests must pass
2. `uv run ruff check python tests` — zero lint errors
3. `uv run pallas-dev preset-check` — all presets valid

**Why:** CI will catch these anyway, but failing CI on a flagship OS repo looks unprofessional.

## Rule: Unitree 4D LiDAR hardware context

The test hardware is a Unitree 4D LiDAR L2 with built-in IMU.
- Preset: `pallas_core_unitree.yaml` / `pallas_ct_unitree.yaml`
- Topics: `/unilidar/cloud` (PointCloud2) + `/unilidar/imu` (Imu)
- Base frame: `unilidar_imu`
- The sensor has its own ROS2 driver package: `unitree_lidar_ros2`
- Indoor testing environment — set `max_range_m` to 15-30m, `min_range_m` to 0.1m
- Extrinsics are identity unless the sensor is mounted offset from body frame

## Rule: Docker must be running for simulation tests

Before running Gazebo or Docker smoke commands:
1. Check `docker info` succeeds
2. If Docker Desktop is not running, tell the user to start it
3. Never attempt `docker build` or `docker compose` with daemon down

## Rule: Protect the public API surface

These are the user-facing contracts that must not break:
- CLI commands: `pallas-dev {doctor,preset-list,preset-show,preset-matrix,preset-check,launch-hint,demo-fetch,demo-replay,ros-check,launch-live,build,test,lint}`
- ROS2 nodes: `anima_pallas_core_node`, `anima_pallas_ct_node`
- Output topics: `/pallas/{core,ct}/{pose,odom,local_map,aligned_scan}`
- Launch files: `pallas_core.launch.py`, `pallas_ct.launch.py`
- Preset naming: `pallas_<profile>_<vendor>.yaml`
