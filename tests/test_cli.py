from pathlib import Path
import subprocess
import zipfile

from typer.testing import CliRunner

import pallas_dev.cli as cli_module
import pallas_dev.demo as demo_module
from pallas_dev.cli import app
from pallas_dev import rosenv


def test_paths_command() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["paths"])
    assert result.exit_code == 0
    assert "repo_root" in result.stdout


def test_config_check(tmp_path: Path) -> None:
    config = tmp_path / "sample.yaml"
    config.write_text("root:\n  child: 1\n", encoding="utf-8")

    runner = CliRunner()
    result = runner.invoke(app, ["config-check", str(config)])
    assert result.exit_code == 0
    assert "root" in result.stdout


def test_preset_list_core() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["preset-list", "--profile", "core"])
    assert result.exit_code == 0
    assert "pallas_core_unitree.yaml" in result.stdout
    assert "pallas_ct_unitree.yaml" not in result.stdout


def test_preset_show_vendor_details() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["preset-show", "pallas_ct_ouster.yaml"])
    assert result.exit_code == 0
    assert "/ouster/points" in result.stdout
    assert "pallas_ct.launch.py" in result.stdout


def test_preset_matrix_markdown() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["preset-matrix", "--format", "markdown"])
    assert result.exit_code == 0
    assert "| Vendor | Core | CT |" in result.stdout
    assert "| Ouster | yes | yes |" in result.stdout


def test_launch_hint_for_velodyne_mentions_external_imu() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["launch-hint", "pallas_core_velodyne.yaml"])
    assert result.exit_code == 0
    assert "config_name:=pallas_core_velodyne.yaml" in result.stdout
    assert "external IMU" in result.stdout


def test_preset_check_reports_success() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["preset-check"])
    assert result.exit_code == 0
    assert "Validated" in result.stdout


def test_ros_setup_prefers_explicit_env(monkeypatch, tmp_path: Path) -> None:
    setup = tmp_path / "setup.bash"
    setup.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    monkeypatch.setenv("ROS_SETUP", str(setup))
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    assert rosenv.ros_setup() == setup


def test_doctor_markdown_includes_launch_hints(monkeypatch) -> None:
    monkeypatch.setattr(cli_module, "ensure_tool", lambda tool: f"/usr/bin/{tool}")
    runner = CliRunner()
    result = runner.invoke(app, ["doctor", "--markdown", "--preset", "pallas_core_ouster.yaml"])
    assert result.exit_code == 0
    assert "# PALLAS Doctor" in result.stdout
    assert "uv run pallas-dev ros-check pallas_core_ouster.yaml" in result.stdout
    assert "uv run pallas-dev launch-live pallas_core_ouster.yaml" in result.stdout


def _write_demo_archive(source_dir: Path) -> None:
    demo = demo_module.get_demo("ouster-core-demo")
    archive_path = source_dir / demo.asset_name
    with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(f"{demo.package_dir}/manifest.txt", "name=ouster-core-demo\n")
        archive.writestr(
            f"{demo.package_dir}/{demo.bag_dir}/metadata.yaml",
            "rosbag2_bagfile_information: {}\n",
        )
        archive.writestr(f"{demo.package_dir}/{demo.rviz_relpath}", "Fixed Frame: odom\n")


def test_demo_fetch_uses_local_asset_source(monkeypatch, tmp_path: Path) -> None:
    source_dir = tmp_path / "demo-source"
    source_dir.mkdir()
    _write_demo_archive(source_dir)
    cache_root = tmp_path / "demo-cache"

    monkeypatch.setenv("PALLAS_DEMO_SOURCE_DIR", str(source_dir))
    monkeypatch.setattr(demo_module, "demo_cache_root", lambda: cache_root)

    runner = CliRunner()
    result = runner.invoke(app, ["demo-fetch", "ouster-core-demo"])
    assert result.exit_code == 0
    assert "PALLAS Demo Ready: Ouster Core Demo" in result.stdout
    assert "pallas-demo-ouster-core.zip" in result.stdout


def test_demo_replay_dry_run_prints_commands(monkeypatch, tmp_path: Path) -> None:
    source_dir = tmp_path / "demo-source"
    source_dir.mkdir()
    _write_demo_archive(source_dir)
    cache_root = tmp_path / "demo-cache"

    monkeypatch.setenv("PALLAS_DEMO_SOURCE_DIR", str(source_dir))
    monkeypatch.setattr(demo_module, "demo_cache_root", lambda: cache_root)

    runner = CliRunner()
    result = runner.invoke(app, ["demo-replay", "ouster-core-demo", "--dry-run"])
    assert result.exit_code == 0
    assert "ros2 bag play" in result.stdout
    assert "rviz2 -d" in result.stdout
    assert "/pallas/core/local_map" in result.stdout


def test_ros_check_reports_matching_topics(monkeypatch) -> None:
    def fake_run_capture(cmd, *, extra_env=None, source_ros=False, timeout=None):
        assert source_ros is True
        if cmd == ["ros2", "topic", "list", "--types"]:
            return subprocess.CompletedProcess(
                cmd,
                0,
                "/ouster/points [sensor_msgs/msg/PointCloud2]\n"
                "/ouster/imu [sensor_msgs/msg/Imu]\n",
                "",
            )
        if cmd == ["ros2", "topic", "info", "-v", "/ouster/points"]:
            return subprocess.CompletedProcess(
                cmd, 0, "Type: sensor_msgs/msg/PointCloud2\nPublisher count: 1\n", ""
            )
        if cmd == ["ros2", "topic", "info", "-v", "/ouster/imu"]:
            return subprocess.CompletedProcess(
                cmd, 0, "Type: sensor_msgs/msg/Imu\nPublisher count: 1\n", ""
            )
        raise AssertionError(f"unexpected command: {cmd}")

    monkeypatch.setattr(cli_module, "run_capture", fake_run_capture)
    monkeypatch.setattr(cli_module, "workspace_setup", lambda: Path("/tmp/install/setup.bash"))

    runner = CliRunner()
    result = runner.invoke(app, ["ros-check", "pallas_core_ouster.yaml"])
    assert result.exit_code == 0
    assert "ROS graph matches pallas_core_ouster.yaml" in result.stdout
    assert "launch-live pallas_core_ouster.yaml" in result.stdout


def test_ros_check_fails_when_topic_is_missing(monkeypatch) -> None:
    def fake_run_capture(cmd, *, extra_env=None, source_ros=False, timeout=None):
        assert source_ros is True
        if cmd == ["ros2", "topic", "list", "--types"]:
            return subprocess.CompletedProcess(cmd, 0, "/ouster/imu [sensor_msgs/msg/Imu]\n", "")
        if cmd == ["ros2", "topic", "info", "-v", "/ouster/imu"]:
            return subprocess.CompletedProcess(
                cmd, 0, "Type: sensor_msgs/msg/Imu\nPublisher count: 1\n", ""
            )
        raise AssertionError(f"unexpected command: {cmd}")

    monkeypatch.setattr(cli_module, "run_capture", fake_run_capture)

    runner = CliRunner()
    result = runner.invoke(app, ["ros-check", "pallas_core_ouster.yaml"])
    assert result.exit_code == 1
    assert "missing" in result.stdout


def test_launch_live_requires_built_workspace(monkeypatch) -> None:
    monkeypatch.setattr(cli_module, "workspace_setup", lambda: None)

    runner = CliRunner()
    result = runner.invoke(app, ["launch-live", "pallas_core_ouster.yaml", "--skip-ros-check"])
    assert result.exit_code == 1
    assert "Workspace is not built yet" in result.stdout


def test_launch_live_runs_ros2_node(monkeypatch, tmp_path: Path) -> None:
    recorded: dict[str, object] = {}

    def fake_run(cmd, *, extra_env=None, source_ros=False):
        recorded["cmd"] = cmd
        recorded["source_ros"] = source_ros
        return 0

    monkeypatch.setattr(cli_module, "workspace_setup", lambda: tmp_path / "setup.bash")
    monkeypatch.setattr(cli_module, "run", fake_run)

    runner = CliRunner()
    result = runner.invoke(app, ["launch-live", "pallas_core_ouster.yaml", "--skip-ros-check"])
    assert result.exit_code == 0
    assert recorded["source_ros"] is True
    assert recorded["cmd"][:4] == ["ros2", "run", "anima_pallas_ros2", "anima_pallas_core_node"]
