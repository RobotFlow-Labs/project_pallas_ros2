# Sensor Presets

PALLAS ships with a LiDAR preset pack so evaluators can start from known ROS2
topic and frame conventions instead of hand-editing YAML on day one.

## Support Matrix

| Vendor preset | Runtime profiles | Point cloud topic | IMU topic | Base frame | Notes |
| --- | --- | --- | --- | --- | --- |
| `generic` | Core, CT | `/points_raw` | `/imu/data` | `imu` | Bring your own driver mapping. |
| `Unitree` | Core, CT | `/unilidar/cloud` | `/unilidar/imu` | `unilidar_imu` | Set measured LiDAR-to-IMU extrinsics. |
| `Livox` | Core, CT | `/livox/lidar` | `/livox/imu` | `livox_frame` | Verify frame names on your robot. |
| `Ouster` | Core, CT | `/ouster/points` | `/ouster/imu` | `os_imu` | Confirm driver frame wiring. |
| `Hesai` | Core, CT | `/lidar_points` | `/lidar_imu` | `hesai_lidar` | Confirm driver topic names and extrinsics. |
| `RoboSense` | Core, CT | `/rslidar_points` | `/rslidar_imu_data` | `rslidar` | Matches the common `rslidar_sdk` conventions. |
| `RoboSense alias` | Core, CT | `/rslidar_points` | `/rslidar_imu_data` | `rslidar` | Alternate preset name for teams using `rslidar` naming. |
| `Velodyne` | Core, CT | `/velodyne_points` | `/imu/data` | `imu` | Requires an external IMU plus measured extrinsics. |

## CLI Workflow

List the shipped presets:

```bash
uv run pallas-dev preset-list
uv run pallas-dev preset-list --profile core
uv run pallas-dev preset-matrix --format markdown
```

Inspect one preset in detail:

```bash
uv run pallas-dev preset-show pallas_core_ouster.yaml
```

Print the exact launch command for a preset:

```bash
uv run pallas-dev launch-hint pallas_ct_unitree.yaml
```

Validate the live ROS graph before starting PALLAS:

```bash
uv run pallas-dev ros-check pallas_core_ouster.yaml
```

Start the runtime directly from the preset after the workspace has been built:

```bash
uv run pallas-dev launch-live pallas_core_ouster.yaml
```

Validate the full preset pack:

```bash
uv run pallas-dev preset-check
```

## Launch Pattern

Every shipped sensor preset goes through the same launch entrypoint:

```bash
ros2 launch anima_pallas_ros2 pallas_core.launch.py config_name:=pallas_core_unitree.yaml
ros2 launch anima_pallas_ros2 pallas_ct.launch.py config_name:=pallas_ct_ouster.yaml
```

## Operational Expectations

- These presets are starting points for integration, not final calibration.
- Any sensor with different LiDAR and IMU frames still needs measured
  `sensor_to_body_translation` and `sensor_to_body_rpy`.
- Velodyne presets assume a separate IMU already exists on `/imu/data`.
- The output point cloud path preserves `ring` or `line` style channel metadata
  so downstream stacks that depend on scan ordering can stay intact.
- For a first live run with a newly connected sensor, follow
  [`docs/first_lidar_test.md`](first_lidar_test.md).
