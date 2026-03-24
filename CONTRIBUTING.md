# Contributing

The lowest-friction way to contribute to PALLAS is to add or improve one sensor
preset.

## Add a Sensor Preset

1. Copy the nearest file under `ros2_ws/src/anima_pallas_ros2/config/`.
2. Keep the naming pattern `pallas_<profile>_<vendor>.yaml`.
3. Only change topic names, frames, notes, and measured extrinsics needed for that driver.
4. Validate the preset pack:

```bash
uv run pallas-dev preset-check
uv run pallas-dev preset-show <new-preset>.yaml
uv run pallas-dev preset-matrix --format markdown
```

## Demo and Docs Checks

Before opening a PR, run:

```bash
uv run pytest
uv run ruff check python tests
./scripts/docker_smoke.sh
```

If your change affects onboarding, also validate:

```bash
uv run pallas-dev doctor --markdown --preset pallas_core_ouster.yaml
uv run pallas-dev demo-replay ouster-core-demo --dry-run
```

## PR Expectations

- keep README claims tied to repeatable commands
- prefer extending preset metadata over hardcoding vendor-specific branches
- use one issue or PR per user-visible improvement
- if you add a new preset, mention the driver package and expected topic names in the PR body
