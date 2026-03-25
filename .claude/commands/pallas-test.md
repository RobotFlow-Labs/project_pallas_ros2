# /pallas-test — Run the full PALLAS validation suite

Run all validation steps in sequence and report a checklist:

1. `uv run pytest -v` — Python unit tests
2. `uv run ruff check python tests` — Lint check
3. `uv run pallas-dev preset-check` — All 19 presets valid
4. `python3 -m py_compile ros2_ws/src/anima_pallas_ros2/launch/pallas_core.launch.py` — Launch syntax
5. `python3 -m py_compile ros2_ws/src/anima_pallas_ros2/launch/pallas_ct.launch.py` — Launch syntax
6. Check git status for uncommitted changes

Report results as a markdown checklist. If anything fails, stop and diagnose.
