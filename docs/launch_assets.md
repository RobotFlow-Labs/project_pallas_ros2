# Launch Assets

PALLAS uses one maintained demo flow to generate launch-ready repo assets. Keep
the source of truth narrow: one demo, one RViz profile, one benchmark summary.

## Required Assets Per Release

- hero visual: `docs/media/pallas-hero.png`
- social preview: `docs/media/pallas-social-preview.png`
- map screenshot 1: `docs/media/pallas-demo-map-1.png`
- map screenshot 2: `docs/media/pallas-demo-map-2.png`
- RViz screenshot: `docs/media/pallas-demo-rviz.png`
- canonical demo rosbag archive: GitHub Release asset `pallas-demo-ouster-core.zip`
- pinned release note: generated from the template below

Current PNG assets are based on screenshots captured from Unitree LiDAR output.
`pallas-demo-rviz.png` is a designed RViz-style composite generated from
`docs/media/pallas-demo-rviz.html`; keep that source note in release copy until a
native RViz capture replaces it.

## Demo Asset Workflow

Record and package the maintained demo from a sourced ROS environment:

```bash
python3 scripts/make_demo_package.py package --demo ouster-core-demo --overwrite
```

Fetch and replay the same asset through the public CLI:

```bash
uv run pallas-dev demo-fetch ouster-core-demo
uv run pallas-dev demo-replay ouster-core-demo
```

Generate the benchmark bundle:

```bash
uv run python scripts/demo_benchmark.py ouster-core-demo --run-replay
```

## Release Note Template

```md
# PALLAS Demo Assets

- Canonical demo: `ouster-core-demo`
- Preset: `pallas_core_ouster.yaml`
- Quickstart:
  - `uv run pallas-dev demo-fetch ouster-core-demo`
  - `uv run pallas-dev demo-replay ouster-core-demo`
- Summary:
  - see `artifacts/benchmarks/ouster-core-demo/summary.md`
- Recommended screenshots:
  - local map
  - second map angle
  - RViz profile
```

## Maintenance Rules

- do not add multiple demo bags unless one demo can no longer cover onboarding
- keep screenshot filenames stable so README links do not churn
- regenerate visuals from the same replay flow used in smoke checks
- keep benchmark claims limited to what `scripts/demo_benchmark.py` emits
