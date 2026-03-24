from pathlib import Path

from typer.testing import CliRunner

from pallas_dev.cli import app


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
