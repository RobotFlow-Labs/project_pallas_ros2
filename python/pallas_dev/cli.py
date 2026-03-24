from __future__ import annotations

import os
from pathlib import Path

import typer
import yaml
from rich.console import Console
from rich.table import Table

from .rosenv import colcon_defaults, ensure_tool, repo_root, ros_workspace, run

app = typer.Typer(no_args_is_help=True)
console = Console()


@app.command()
def doctor() -> None:
    table = Table(title="PALLAS Doctor")
    table.add_column("Check")
    table.add_column("Value")

    table.add_row("repo_root", str(repo_root()))
    table.add_row("ros_workspace", str(ros_workspace()))
    table.add_row("colcon_defaults", str(colcon_defaults()))
    table.add_row("ROS_DISTRO", os.environ.get("ROS_DISTRO", "<unset>"))

    for tool in ("uv", "colcon", "git"):
        try:
            table.add_row(tool, ensure_tool(tool))
        except RuntimeError as exc:
            table.add_row(tool, str(exc))

    console.print(table)


@app.command("config-check")
def config_check(path: Path) -> None:
    with path.open("r", encoding="utf-8") as handle:
        parsed = yaml.safe_load(handle)
    console.print({"path": str(path), "top_level_keys": sorted(parsed.keys()) if parsed else []})


@app.command()
def paths() -> None:
    console.print(
        {
            "repo_root": str(repo_root()),
            "ros_workspace": str(ros_workspace()),
            "colcon_defaults": str(colcon_defaults()),
        }
    )


@app.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def build(ctx: typer.Context) -> None:
    ensure_tool("colcon")
    cmd = ["colcon", "build", "--base-paths", "ros2_ws/src", *ctx.args]
    raise typer.Exit(run(cmd))


@app.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def test(ctx: typer.Context) -> None:
    ensure_tool("colcon")
    cmd = ["colcon", "test", "--base-paths", "ros2_ws/src", *ctx.args]
    raise typer.Exit(run(cmd))


@app.command()
def lint() -> None:
    rc = run(["ruff", "check", "python", "tests"])
    if rc != 0:
        raise typer.Exit(rc)
    console.print("[green]ruff passed[/green]")


if __name__ == "__main__":
    app()
