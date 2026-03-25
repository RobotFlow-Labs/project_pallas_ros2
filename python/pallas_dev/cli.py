"""CLI application for PALLAS developer workflows (presets, demos, builds)."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

import typer
import yaml
from rich.console import Console
from rich.table import Table

from .demo import (
    expected_output_topics,
    fetch_demo,
    get_demo,
    run_demo_replay,
    shipped_demos,
)
from .presets import get_preset, shipped_presets
from .rosenv import (
    colcon_defaults,
    ensure_tool,
    repo_root,
    ros_setup,
    ros_workspace,
    run,
    run_capture,
    workspace_setup,
)

app = typer.Typer(no_args_is_help=True)
console = Console()
_INPUT_TOPIC_EXPECTATIONS = (
    ("pointcloud", "pointcloud_topic", "sensor_msgs/msg/PointCloud2"),
    ("imu", "imu_topic", "sensor_msgs/msg/Imu"),
)


def _normalize_profile(profile: str) -> str | None:
    normalized = profile.strip().lower()
    if normalized == "all":
        return None
    if normalized not in {"core", "ct"}:
        raise typer.BadParameter("profile must be one of: all, core, ct")
    return normalized


def _parse_topic_inventory(output: str) -> dict[str, str]:
    inventory: dict[str, str] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(maxsplit=1)
        topic = parts[0]
        type_info = parts[1].strip() if len(parts) > 1 else "<unknown>"
        inventory[topic] = type_info
    return inventory


def _ros_capture(cmd: list[str], *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    try:
        return run_capture(cmd, source_ros=True, timeout=timeout)
    except RuntimeError as exc:
        raise typer.BadParameter(str(exc)) from exc
    except subprocess.TimeoutExpired as exc:
        raise typer.BadParameter(
            f"ROS command timed out after {timeout or 0:.1f}s: {' '.join(cmd)}"
        ) from exc


def _print_ros_failure(action: str, result: subprocess.CompletedProcess[str]) -> None:
    console.print(f"[red]{action} failed[/red]")
    stderr = result.stderr.strip()
    stdout = result.stdout.strip()
    if stderr:
        console.print(stderr)
    elif stdout:
        console.print(stdout)


def _extract_count(output: str, label: str) -> str:
    prefix = f"{label}:"
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith(prefix):
            return line.split(":", 1)[1].strip()
    return "?"


def _topic_inventory_without_types(output: str) -> dict[str, str]:
    inventory: dict[str, str] = {}
    for raw_line in output.splitlines():
        topic = raw_line.strip()
        if topic:
            inventory[topic] = "<unknown>"
    return inventory


def _collect_topic_inventory(timeout_sec: float) -> dict[str, str]:
    list_commands = (
        ("ros2 topic list --types", ["ros2", "topic", "list", "--types"], _parse_topic_inventory),
        ("ros2 topic list -t", ["ros2", "topic", "list", "-t"], _parse_topic_inventory),
        ("ros2 topic list", ["ros2", "topic", "list"], _topic_inventory_without_types),
    )

    last_result: subprocess.CompletedProcess[str] | None = None
    last_action = "ros2 topic list"
    for action, cmd, parser in list_commands:
        result = _ros_capture(cmd, timeout=timeout_sec)
        if result.returncode == 0:
            return parser(result.stdout)
        last_result = result
        last_action = action

    if last_result is not None:
        _print_ros_failure(last_action, last_result)
        raise typer.Exit(last_result.returncode or 1)

    raise typer.Exit(1)


def _render_ros_check(name: str, timeout_sec: float) -> tuple[bool, str]:
    try:
        preset = get_preset(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    inventory = _collect_topic_inventory(timeout_sec)

    table = Table(title=f"ROS Graph Check: {preset.name}")
    table.add_column("Stream")
    table.add_column("Topic", overflow="fold")
    table.add_column("Expected Type")
    table.add_column("Seen Type", overflow="fold")
    table.add_column("Publishers")
    table.add_column("Status")

    ok = True
    for label, attr_name, expected_type in _INPUT_TOPIC_EXPECTATIONS:
        topic = getattr(preset, attr_name)
        seen_type = inventory.get(topic)
        publishers = "0"

        if seen_type is None:
            status = "[red]missing[/red]"
            ok = False
        else:
            topic_info = _ros_capture(["ros2", "topic", "info", "-v", topic], timeout=timeout_sec)
            if topic_info.returncode == 0:
                publishers = _extract_count(topic_info.stdout, "Publisher count")
                info_type = _extract_count(topic_info.stdout, "Type")
                if info_type != "?":
                    seen_type = info_type
            else:
                publishers = "?"

            if expected_type not in seen_type:
                status = "[red]type mismatch[/red]"
                ok = False
            elif publishers.isdigit() and int(publishers) == 0:
                status = "[red]no publishers[/red]"
                ok = False
            else:
                status = "[green]ok[/green]"

        table.add_row(
            label,
            topic,
            expected_type,
            seen_type or "<missing>",
            publishers,
            status,
        )

    console.print(table)
    return ok, preset.name


def _doctor_rows() -> list[tuple[str, str]]:
    presets = shipped_presets()
    demos = shipped_demos()
    rows = [
        ("repo_root", str(repo_root())),
        ("ros_workspace", str(ros_workspace())),
        ("colcon_defaults", str(colcon_defaults())),
        ("ROS_DISTRO", os.environ.get("ROS_DISTRO", "<unset>")),
        ("ros_setup", str(ros_setup()) if ros_setup() else "<not found>"),
        ("workspace_setup", str(workspace_setup()) if workspace_setup() else "<not built>"),
        ("preset_count", str(len(presets))),
        ("core_presets", str(sum(1 for preset in presets if preset.profile == "core"))),
        ("ct_presets", str(sum(1 for preset in presets if preset.profile == "ct"))),
        ("demo_count", str(len(demos))),
    ]
    for tool in ("uv", "colcon", "git"):
        try:
            rows.append((tool, ensure_tool(tool)))
        except RuntimeError as exc:
            rows.append((tool, str(exc)))
    return rows


def _render_markdown_table(title: str, rows: list[tuple[str, str]]) -> str:
    lines = [f"## {title}", "", "| Check | Value |", "| --- | --- |"]
    for label, value in rows:
        safe_value = value.replace("|", "\\|")
        lines.append(f"| `{label}` | `{safe_value}` |")
    return "\n".join(lines)


def _preset_matrix_rows() -> list[tuple[str, str, str, str, str, str]]:
    grouped: dict[str, dict[str, str]] = {}
    details: dict[str, dict[str, str]] = {}
    for preset in shipped_presets():
        grouped.setdefault(preset.vendor_label, {})[preset.profile] = "yes"
        details.setdefault(
            preset.vendor_label,
            {
                "cloud": preset.pointcloud_topic,
                "imu": preset.imu_topic,
                "frame": preset.base_frame,
            },
        )

    rows: list[tuple[str, str, str, str, str, str]] = []
    for vendor in sorted(grouped):
        info = grouped[vendor]
        meta = details[vendor]
        rows.append(
            (
                vendor,
                info.get("core", "no"),
                info.get("ct", "no"),
                meta["cloud"],
                meta["imu"],
                meta["frame"],
            )
        )
    return rows


@app.command()
def doctor(
    markdown: bool = typer.Option(
        False,
        "--markdown",
        help="Emit copy-pasteable Markdown for GitHub issues and support threads.",
    ),
    preset: str | None = typer.Option(
        None,
        "--preset",
        help="Include preset-specific launch hints in the report.",
    ),
) -> None:
    rows = _doctor_rows()
    if not markdown:
        table = Table(title="PALLAS Doctor")
        table.add_column("Check")
        table.add_column("Value")
        for label, value in rows:
            table.add_row(label, value)
        console.print(table)
        if preset:
            preset_show(preset)
        return

    lines = ["# PALLAS Doctor", "", _render_markdown_table("Environment", rows)]
    if preset:
        try:
            selected = get_preset(preset)
        except KeyError as exc:
            raise typer.BadParameter(exc.args[0]) from exc

        lines.extend(
            [
                "",
                "## Launch Hints",
                "",
                f"- preset: `{selected.name}`",
                f"- pointcloud_topic: `{selected.pointcloud_topic}`",
                f"- imu_topic: `{selected.imu_topic}`",
                f"- ros-check: `uv run pallas-dev ros-check {selected.name}`",
                f"- launch-live: `uv run pallas-dev launch-live {selected.name}`",
            ]
        )

    typer.echo("\n".join(lines))


@app.command("config-check")
def config_check(path: Path) -> None:
    with path.open("r", encoding="utf-8") as handle:
        parsed = yaml.safe_load(handle)
    if not isinstance(parsed, dict):
        console.print({"path": str(path), "top_level_keys": [], "warning": "Not a YAML mapping"})
        return
    console.print({"path": str(path), "top_level_keys": sorted(parsed.keys())})


@app.command("preset-list")
def preset_list(
    profile: str = typer.Option("all", "--profile", "-p", help="Filter by runtime profile."),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Show topic and frame details."),
) -> None:
    selected = shipped_presets(profile=_normalize_profile(profile))

    table = Table(title="PALLAS LiDAR Presets")
    table.add_column("Preset", overflow="fold")
    table.add_column("Profile")
    table.add_column("Vendor", overflow="fold")
    table.add_column("LiDAR Type")
    if verbose:
        table.add_column("Cloud Topic", overflow="fold")
        table.add_column("IMU Topic", overflow="fold")
        table.add_column("Base Frame", overflow="fold")

    for preset in selected:
        row = [
            preset.name,
            preset.profile,
            preset.vendor_label,
            preset.lidar_type,
        ]
        if verbose:
            row.extend([preset.pointcloud_topic, preset.imu_topic, preset.base_frame])
        table.add_row(*row)

    console.print(table)


@app.command("preset-matrix")
def preset_matrix(
    output_format: str = typer.Option("table", "--format", help="Output format: table or markdown."),
) -> None:
    rows = _preset_matrix_rows()
    if output_format not in {"table", "markdown"}:
        raise typer.BadParameter("format must be one of: table, markdown")

    if output_format == "markdown":
        lines = [
            "| Vendor | Core | CT | Cloud Topic | IMU Topic | Base Frame |",
            "| --- | --- | --- | --- | --- | --- |",
        ]
        for row in rows:
            lines.append("| " + " | ".join(row) + " |")
        typer.echo("\n".join(lines))
        return

    table = Table(title="PALLAS Sensor Support Matrix")
    table.add_column("Vendor", overflow="fold")
    table.add_column("Core")
    table.add_column("CT")
    table.add_column("Cloud Topic", overflow="fold")
    table.add_column("IMU Topic", overflow="fold")
    table.add_column("Base Frame", overflow="fold")
    for row in rows:
        table.add_row(*row)
    console.print(table)


@app.command("preset-show")
def preset_show(name: str) -> None:
    try:
        preset = get_preset(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    table = Table(title=f"PALLAS Preset: {preset.name}")
    table.add_column("Field")
    table.add_column("Value")
    table.add_row("profile", preset.profile)
    table.add_row("vendor", preset.vendor_label)
    table.add_row("lidar_type", preset.lidar_type)
    table.add_row("config_path", str(preset.path))
    table.add_row("pointcloud_topic", preset.pointcloud_topic)
    table.add_row("imu_topic", preset.imu_topic)
    table.add_row("pose_topic", preset.pose_topic)
    table.add_row("odom_topic", preset.odom_topic)
    table.add_row("map_topic", preset.map_topic)
    table.add_row("aligned_scan_topic", preset.aligned_scan_topic)
    table.add_row("odom_frame", preset.odom_frame)
    table.add_row("base_frame", preset.base_frame)
    table.add_row("launch", preset.launch_command)
    table.add_row("ros2_run", preset.ros2_run_command)
    table.add_row("ros_check", f"uv run pallas-dev ros-check {preset.name}")
    table.add_row("live_launch", preset.live_launch_command)
    table.add_row("note", preset.note)
    console.print(table)


@app.command("launch-hint")
def launch_hint(name: str) -> None:
    try:
        preset = get_preset(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    console.print(f"[bold]{preset.launch_command}[/bold]")
    console.print(f"[bold]{preset.live_launch_command}[/bold]")
    console.print(f"Check inputs first: uv run pallas-dev ros-check {preset.name}")
    console.print(preset.note)


@app.command("demo-fetch")
def demo_fetch(
    name: str,
    force: bool = typer.Option(False, "--force", help="Re-download and re-extract the demo package."),
) -> None:
    try:
        demo = get_demo(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    try:
        paths = fetch_demo(demo.name, force=force)
    except Exception as exc:
        raise typer.BadParameter(str(exc)) from exc

    table = Table(title=f"PALLAS Demo Ready: {demo.title}")
    table.add_column("Field")
    table.add_column("Value", overflow="fold")
    table.add_row("demo", demo.name)
    table.add_row("preset", demo.preset.name)
    table.add_row("archive", str(paths.archive_path))
    table.add_row("bag_path", str(paths.bag_path))
    table.add_row("rviz_config", str(paths.rviz_config))
    table.add_row("release_url", demo.release_url)
    console.print(table)
    console.print(f"Next: [bold]uv run pallas-dev build[/bold] then [bold]uv run pallas-dev demo-replay {demo.name}[/bold]")


@app.command("demo-replay")
def demo_replay(
    name: str,
    dry_run: bool = typer.Option(False, "--dry-run", help="Print the exact commands without running them."),
    rate: float = typer.Option(1.0, "--rate", min=0.1, help="ros2 bag playback rate."),
    node_startup_sec: float = typer.Option(
        1.5,
        "--node-startup-sec",
        min=0.5,
        help="How long to wait before starting bag playback.",
    ),
    topic_timeout_sec: float = typer.Option(
        8.0,
        "--topic-timeout-sec",
        min=2.0,
        help="How long to wait for expected output topics to appear.",
    ),
) -> None:
    try:
        demo = get_demo(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    try:
        paths = fetch_demo(demo.name)
    except Exception as exc:
        raise typer.BadParameter(str(exc)) from exc

    preset = demo.preset
    node_cmd = (
        "ros2 run anima_pallas_ros2 "
        f"{preset.node_name} --ros-args --params-file {preset.path}"
    )
    bag_cmd = f"ros2 bag play {paths.bag_path} --rate {rate}"

    table = Table(title=f"PALLAS Demo Replay: {demo.title}")
    table.add_column("Field")
    table.add_column("Value", overflow="fold")
    table.add_row("description", demo.description)
    table.add_row("preset", preset.name)
    table.add_row("bag_path", str(paths.bag_path))
    table.add_row("runtime_cmd", node_cmd)
    table.add_row("bag_cmd", bag_cmd)
    table.add_row("rviz_cmd", f"rviz2 -d {paths.rviz_config}")
    table.add_row("expected_topics", ", ".join(expected_output_topics(demo)))
    console.print(table)

    if dry_run:
        console.print("[green]Dry run complete.[/green]")
        return

    try:
        result = run_demo_replay(
            demo.name,
            rate=rate,
            node_startup_sec=node_startup_sec,
            topic_timeout_sec=topic_timeout_sec,
        )
    except Exception as exc:
        raise typer.BadParameter(str(exc)) from exc

    result_table = Table(title=f"Demo Replay Result: {demo.name}")
    result_table.add_column("Field")
    result_table.add_column("Value", overflow="fold")
    result_table.add_row("wall_runtime_sec", f"{result.wall_runtime_sec:.2f}")
    result_table.add_row("bag_returncode", str(result.bag_returncode))
    result_table.add_row("node_returncode", str(result.node_returncode))
    result_table.add_row("seen_topics", ", ".join(result.seen_topics) or "<none>")
    result_table.add_row("missing_topics", ", ".join(result.missing_topics) or "<none>")
    result_table.add_row("node_log", str(result.node_log))
    result_table.add_row("bag_log", str(result.bag_log))
    console.print(result_table)
    if not result.ok:
        raise typer.Exit(1)


@app.command("ros-check")
def ros_check(
    name: str,
    timeout_sec: float = typer.Option(
        5.0,
        "--timeout-sec",
        min=1.0,
        help="Timeout for ROS graph inspection commands.",
    ),
) -> None:
    ok, preset_name = _render_ros_check(name, timeout_sec)
    if not ok:
        console.print(f"[yellow]Use `uv run pallas-dev preset-show {preset_name}` to inspect the expected inputs.[/yellow]")
        raise typer.Exit(1)

    console.print(f"[green]ROS graph matches {preset_name}[/green]")
    if workspace_setup():
        console.print(f"Next: [bold]uv run pallas-dev launch-live {preset_name}[/bold]")
    else:
        console.print(
            "Next: [bold]uv run pallas-dev build[/bold] "
            f"then [bold]uv run pallas-dev launch-live {preset_name}[/bold]"
        )


@app.command("launch-live")
def launch_live(
    name: str,
    skip_ros_check: bool = typer.Option(
        False,
        "--skip-ros-check",
        help="Launch immediately without validating the input topics first.",
    ),
    timeout_sec: float = typer.Option(
        5.0,
        "--timeout-sec",
        min=1.0,
        help="Timeout for the optional ROS graph check.",
    ),
) -> None:
    try:
        preset = get_preset(name)
    except KeyError as exc:
        raise typer.BadParameter(exc.args[0]) from exc

    if not workspace_setup():
        console.print("[red]Workspace is not built yet.[/red]")
        console.print("Run [bold]uv run pallas-dev build[/bold] first.")
        raise typer.Exit(1)

    if not skip_ros_check:
        ok, _ = _render_ros_check(preset.name, timeout_sec)
        if not ok:
            raise typer.Exit(1)

    console.print(f"[bold]Starting {preset.profile.upper()} runtime for {preset.vendor_label}[/bold]")
    console.print(f"Inputs: {preset.pointcloud_topic} + {preset.imu_topic}")
    console.print(
        "Outputs: "
        f"{preset.pose_topic}, {preset.odom_topic}, {preset.aligned_scan_topic}, {preset.map_topic}"
    )
    cmd = [
        "ros2",
        "run",
        "anima_pallas_ros2",
        preset.node_name,
        "--ros-args",
        "--params-file",
        str(preset.path),
    ]
    raise typer.Exit(run(cmd, source_ros=True))


@app.command("preset-check")
def preset_check(
    profile: str = typer.Option("all", "--profile", "-p", help="Filter by runtime profile."),
) -> None:
    selected = shipped_presets(profile=_normalize_profile(profile))
    if not selected:
        raise typer.BadParameter("No presets matched the selected profile")

    table = Table(title="PALLAS Preset Validation")
    table.add_column("Preset", overflow="fold")
    table.add_column("Profile")
    table.add_column("Vendor", overflow="fold")
    table.add_column("Launch File", overflow="fold")

    for preset in selected:
        if not preset.launch_path.is_file():
            raise typer.BadParameter(f"Missing launch file for {preset.name}: {preset.launch_file}")
        table.add_row(
            preset.name,
            preset.profile,
            preset.vendor_label,
            preset.launch_file,
        )

    console.print(table)
    console.print(f"[green]Validated {len(selected)} shipped presets[/green]")


@app.command()
def paths() -> None:
    console.print(
        {
            "repo_root": str(repo_root()),
            "ros_workspace": str(ros_workspace()),
            "colcon_defaults": str(colcon_defaults()),
            "workspace_setup": str(workspace_setup()) if workspace_setup() else None,
        }
    )


@app.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def build(ctx: typer.Context) -> None:
    ensure_tool("colcon")
    cmd = ["colcon", "build", "--base-paths", "ros2_ws/src", *ctx.args]
    raise typer.Exit(run(cmd, source_ros=True))


@app.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def test(ctx: typer.Context) -> None:
    ensure_tool("colcon")
    cmd = ["colcon", "test", "--base-paths", "ros2_ws/src", *ctx.args]
    raise typer.Exit(run(cmd, source_ros=True))


@app.command()
def lint() -> None:
    rc = run(["ruff", "check", "python", "tests"])
    if rc != 0:
        raise typer.Exit(rc)
    console.print("[green]ruff passed[/green]")


if __name__ == "__main__":
    app()
