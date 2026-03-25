# /pallas-unitree — Unitree 4D LiDAR Bring-Up Assistant

Guide the user through the complete Unitree 4D LiDAR L2 bring-up with PALLAS.

## Step-by-step checklist:

### 1. Driver Setup
- Verify `unitree_lidar_ros2` package is installed
- Check `ros2 pkg list | grep unitree`
- If missing, guide installation from source

### 2. Sensor Connection
- Verify USB/Ethernet connection to the LiDAR
- Check `ros2 topic list` for `/unilidar/cloud` and `/unilidar/imu`
- If topics missing, help launch the driver node

### 3. Data Validation
- `ros2 topic hz /unilidar/cloud` — expect 10-20 Hz
- `ros2 topic hz /unilidar/imu` — expect 100-400 Hz
- `ros2 topic echo /unilidar/imu --once` — verify gravity vector magnitude

### 4. PALLAS Preset Check
- `uv run pallas-dev preset-show pallas_core_unitree.yaml`
- `uv run pallas-dev ros-check pallas_core_unitree.yaml`
- If topics don't match, help adjust the preset

### 5. Build & Launch
- `uv run pallas-dev build`
- `uv run pallas-dev launch-live pallas_core_unitree.yaml`

### 6. Verify Output
- Check output topics appear: `/pallas/core/{pose,odom,local_map,aligned_scan}`
- `ros2 topic hz /pallas/core/odom` — should match LiDAR rate
- Optionally open RViz2 with PointCloud2 + Odometry displays

### 7. Record Results
- Save a test bag: `ros2 bag record -o unitree_indoor_test /unilidar/cloud /unilidar/imu /pallas/core/pose /pallas/core/odom /pallas/core/local_map`
- Run `uv run pallas-dev doctor --markdown --preset pallas_core_unitree.yaml`
- Capture RViz screenshot for docs

### Indoor Testing Notes
- `max_range_m: 15` may be better for small rooms (default is 200)
- Watch for glass reflections — they produce phantom points
- Keep the sensor stationary for first 2-3 seconds (IMU bootstrap needs stillness)
- The Unitree L2 has a 360° FoV — expect dense clouds (~50K pts/scan)
