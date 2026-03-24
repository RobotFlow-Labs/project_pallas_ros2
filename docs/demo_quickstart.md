# Demo Quickstart

This is the shortest path from clean checkout to a repeatable PALLAS run without
connecting a live LiDAR.

## 1. Prepare the repo

```bash
uv sync --group dev
uv run pallas-dev build
```

If you only want to validate the stack from Docker first:

```bash
./scripts/docker_smoke.sh
```

## 2. Fetch the canonical demo

```bash
uv run pallas-dev demo-fetch ouster-core-demo
```

That resolves one maintained demo package:

- preset: `pallas_core_ouster.yaml`
- inputs: `/ouster/points` and `/ouster/imu`
- outputs: `/pallas/core/pose`, `/pallas/core/odom`, `/pallas/core/aligned_scan`, `/pallas/core/local_map`

The package includes:

- a small ROS2 bag
- a shipped RViz profile
- stable output paths for benchmark summaries and screenshots

## 3. Replay the demo

```bash
uv run pallas-dev demo-replay ouster-core-demo
```

The command prints:

- the exact `ros2 bag play ...` command
- the exact `ros2 run anima_pallas_ros2 ...` command
- the exact RViz command for the shipped profile
- the expected output topics

For a command-only check:

```bash
uv run pallas-dev demo-replay ouster-core-demo --dry-run
```

## 4. Open RViz

Use the printed path or run it directly:

```bash
rviz2 -d artifacts/demos/pallas-demo-ouster-core/rviz/pallas_demo.rviz
```

## 5. Capture proof or benchmark output

Generate the canonical summary bundle:

```bash
uv run python scripts/demo_benchmark.py ouster-core-demo --run-replay
```

That writes:

- `artifacts/benchmarks/ouster-core-demo/summary.json`
- `artifacts/benchmarks/ouster-core-demo/summary.md`

## Moving to Live Hardware

Once the demo path works, switch to a real driver graph:

1. pick the nearest preset with `uv run pallas-dev preset-list`
2. validate the ROS graph with `uv run pallas-dev ros-check <preset>`
3. run the live launch with `uv run pallas-dev launch-live <preset>`

The live bring-up path is documented in [`docs/first_lidar_test.md`](first_lidar_test.md).
