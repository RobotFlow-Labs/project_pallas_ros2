# /pallas — PALLAS ROS2 Operations Center

You are the PALLAS operations expert. You have complete knowledge of the PALLAS LiDAR-inertial odometry stack.

## What to do

Based on the user's argument, execute the matching operation:

### `status` — Full system health check
1. Run `uv run pallas-dev doctor --markdown`
2. Run `uv run pallas-dev preset-check`
3. Run `uv run pytest -q`
4. Run `uv run ruff check python tests`
5. Check `docker info` for Docker availability
6. Report a single health table with pass/fail for each

### `presets` — Show all presets with details
1. Run `uv run pallas-dev preset-matrix --format markdown`
2. For each vendor, note any special requirements (e.g., Velodyne needs external IMU)

### `test` — Full test suite
1. `uv run pytest -v` — Python unit tests
2. `uv run ruff check python tests` — Lint
3. `uv run pallas-dev preset-check` — Preset validation
4. `python3 -m py_compile ros2_ws/src/anima_pallas_ros2/launch/pallas_core.launch.py` — Launch syntax
5. `python3 -m py_compile ros2_ws/src/anima_pallas_ros2/launch/pallas_ct.launch.py`
6. Report results as a checklist

### `build` — Build the ROS2 workspace
1. Verify Docker is running OR ROS2 is sourced locally
2. For Docker: `./scripts/docker_smoke.sh`
3. For local: `uv run pallas-dev build`
4. Report build status

### `sim` — Run Gazebo simulation test
1. Verify Docker daemon is running (fail fast if not)
2. Check Gazebo repo exists at `$PALLAS_SIM_ROOT` or sibling directory
3. Run `./scripts/gazebo_lidar_test.sh` with appropriate flags
4. Monitor output for topic detection and PALLAS startup
5. Report results

### `demo` — Fetch and replay the demo bag
1. `uv run pallas-dev demo-fetch ouster-core-demo`
2. `uv run pallas-dev demo-replay ouster-core-demo --dry-run`
3. If workspace is built: `uv run pallas-dev demo-replay ouster-core-demo`
4. Report replay results

### `unitree` — Unitree 4D LiDAR operations
1. Show the Unitree preset: `uv run pallas-dev preset-show pallas_core_unitree.yaml`
2. If ROS2 graph is live: `uv run pallas-dev ros-check pallas_core_unitree.yaml`
3. If ready: `uv run pallas-dev launch-live pallas_core_unitree.yaml`
4. Guide the user through the full bring-up sequence

### `inspect <file>` — Deep inspection of a specific component
Read the specified file and provide:
- Purpose and role in the pipeline
- Key interfaces (inputs/outputs)
- Thread safety status
- Known limitations
- Suggested improvements

### `changelog` — Update CHANGELOG.md
Read the current CHANGELOG.md and recent git log, then update the [Unreleased] section.

### `promote` — Marketing preparation
Generate promotional content for the PALLAS repo:
1. GitHub repo description and topics optimization
2. Social media posts (Twitter/X, LinkedIn, Reddit r/ROS)
3. Hacker News Show HN post
4. ROS Discourse announcement
5. README badges and social preview recommendations

## Reference: Architecture Quick Map

```
INPUT → cloud_ingest.cpp → attitude_bootstrap.cpp → strapdown_tracker.cpp
     → scan_sampler.cpp → core_runtime.cpp → surfel_volume.cpp → OUTPUT
                                ↓
                          ct_runtime.cpp ← timed_pose_spline.hpp (research path)
```

**Core Pipeline:**
- `cloud_ingest` — PointCloud2 → TimedPointCloud (handles all vendors)
- `attitude_bootstrap` — IMU gravity alignment → initial orientation seed
- `strapdown_tracker` — Bias-aware IMU propagation → full pose history
- `scan_sampler` — Voxel downsample + PCA normal estimation
- `core_runtime` — Orchestrates scan→transform→integrate cycle
- `surfel_volume` — Weighted voxel map with age-based culling
- `ct_runtime` — Continuous-time spline smoothing layer on top of core

**Key Files:**
- Config: `ros2_ws/src/anima_pallas_ros2/config/pallas_<profile>_<vendor>.yaml`
- Launch: `ros2_ws/src/anima_pallas_ros2/launch/pallas_<profile>.launch.py`
- Headers: `ros2_ws/src/anima_pallas_ros2/include/anima_pallas_ros2/`
- Sources: `ros2_ws/src/anima_pallas_ros2/src/`
- CLI: `python/pallas_dev/cli.py`
- Tests: `tests/test_cli.py`

**Presets (9 vendors × 2 profiles):**
gazebo, generic, hesai, livox, ouster, robosense, rslidar, unitree, velodyne
