from __future__ import annotations

import os
import shlex
import shutil
import subprocess
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ros_workspace() -> Path:
    return repo_root() / "ros2_ws"


def colcon_defaults() -> Path:
    return repo_root() / "config" / "colcon.defaults.yaml"


def workspace_setup() -> Path | None:
    candidate = repo_root() / "install" / "setup.bash"
    return candidate if candidate.is_file() else None


def ros_setup() -> Path | None:
    explicit = os.environ.get("ROS_SETUP")
    if explicit:
        path = Path(explicit)
        if path.is_file():
            return path

    distro = os.environ.get("ROS_DISTRO")
    if distro:
        candidate = Path("/opt/ros") / distro / "setup.bash"
        if candidate.is_file():
            return candidate

    ros_root = Path("/opt/ros")
    if not ros_root.is_dir():
        return None

    candidates = sorted(ros_root.glob("*/setup.bash"))
    return candidates[-1] if candidates else None


def ensure_tool(name: str) -> str:
    path = shutil.which(name)
    if not path:
        raise RuntimeError(f"Required tool not found on PATH: {name}")
    return path


def command_env(extra_env: dict[str, str] | None = None) -> dict[str, str]:
    env = os.environ.copy()
    env["COLCON_DEFAULTS_FILE"] = str(colcon_defaults())
    if extra_env:
        env.update(extra_env)
    return env


def prepare_command(cmd: list[str], *, source_ros: bool) -> list[str]:
    if not source_ros:
        return cmd

    setup = ros_setup()
    if not setup:
        raise RuntimeError(
            "ROS setup.bash not found. Set ROS_DISTRO/ROS_SETUP or install ROS under /opt/ros."
        )

    sources = [f"source {shlex.quote(str(setup))}"]
    overlay = workspace_setup()
    if overlay:
        sources.append(f"source {shlex.quote(str(overlay))}")
    sources.append(shlex.join(cmd))
    return ["bash", "-lc", " && ".join(sources)]


def run(
    cmd: list[str],
    *,
    extra_env: dict[str, str] | None = None,
    source_ros: bool = False,
) -> int:
    result = subprocess.run(
        prepare_command(cmd, source_ros=source_ros),
        cwd=repo_root(),
        env=command_env(extra_env),
        check=False,
    )
    return result.returncode


def run_capture(
    cmd: list[str],
    *,
    extra_env: dict[str, str] | None = None,
    source_ros: bool = False,
    timeout: float | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        prepare_command(cmd, source_ros=source_ros),
        cwd=repo_root(),
        env=command_env(extra_env),
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
