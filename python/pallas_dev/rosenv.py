from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ros_workspace() -> Path:
    return repo_root() / "ros2_ws"


def colcon_defaults() -> Path:
    return repo_root() / "config" / "colcon.defaults.yaml"


def ensure_tool(name: str) -> str:
    path = shutil.which(name)
    if not path:
        raise RuntimeError(f"Required tool not found on PATH: {name}")
    return path


def run(cmd: list[str], *, extra_env: dict[str, str] | None = None) -> int:
    env = os.environ.copy()
    env["COLCON_DEFAULTS_FILE"] = str(colcon_defaults())
    if extra_env:
        env.update(extra_env)
    result = subprocess.run(cmd, cwd=repo_root(), env=env, check=False)
    return result.returncode
