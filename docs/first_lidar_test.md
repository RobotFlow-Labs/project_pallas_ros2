# First LiDAR Test

This is the shortest path from "the driver is publishing" to "PALLAS is
running live against the sensor."

If you do not have hardware connected yet, start with
[`docs/demo_quickstart.md`](demo_quickstart.md) first. That path exercises the
same shipped Ouster/Core preset against a maintained bag replay.

If you want a real ROS2 graph without touching hardware, use
[`docs/gazebo_simulation.md`](gazebo_simulation.md). That path connects PALLAS
to the shared ANIMA Gazebo repo and exposes `/anima/lidar/points` plus
`/anima/imu/data`.

## 1. Pick the closest shipped preset

List the presets:

```bash
uv run pallas-dev preset-list
```

Inspect the one that matches your driver:

```bash
uv run pallas-dev preset-show pallas_core_ouster.yaml
```

## 2. Start the vendor driver first

Bring up the LiDAR and IMU driver stack exactly as the vendor expects. PALLAS
assumes the sensor topics already exist before it starts.

Examples:

- Ouster: expect `/ouster/points` and `/ouster/imu`
- Livox: expect `/livox/lidar` and `/livox/imu`
- Velodyne: expect `/velodyne_points` plus an external IMU on `/imu/data`

## 3. Check the ROS graph against the preset

Run the graph check before launching PALLAS:

```bash
uv run pallas-dev ros-check pallas_core_ouster.yaml
```

That validates:

- the expected point cloud topic exists
- the expected IMU topic exists
- the discovered ROS message types match the preset
- each input topic has at least one publisher

If the topics do not line up, either switch to a different shipped preset or
copy the nearest config and adjust the topic names and extrinsics.

## 4. Build once, then start the live runtime

First build:

```bash
uv run pallas-dev build
```

Then launch the runtime from the preset:

```bash
uv run pallas-dev launch-live pallas_core_ouster.yaml
```

Or use the one-command helper:

```bash
./scripts/live_lidar_test.sh pallas_core_ouster.yaml
```

## 5. Confirm PALLAS outputs

After launch, verify the output topics from a second terminal:

```bash
ros2 topic echo /pallas/core/pose --once
ros2 topic echo /pallas/core/odom --once
ros2 topic list | grep /pallas/core
```

For CT presets, swap the topic prefix to `/pallas/ct/...`.

## Docker vs Live Sensor Use

`./scripts/docker_smoke.sh` is the fast evaluation path for build and CI-style
validation on laptops and Apple machines.

A live sensor-connected run is different: it needs a real ROS2 graph, a driver
publishing LiDAR and IMU data, and a local workspace build of `anima_pallas_ros2`.
For that path, prefer native ROS2 on the host or a ROS environment where the
driver and PALLAS participate in the same DDS domain.
