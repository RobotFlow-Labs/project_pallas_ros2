"""Demo asset lifecycle: fetch, extract, replay, and benchmark ROS2 bags."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import time
import urllib.request
import zipfile

from .presets import Preset, get_preset
from .rosenv import command_env, prepare_command, repo_root, run_capture, workspace_setup


@dataclass(frozen=True)
class DemoAsset:
    name: str
    title: str
    description: str
    preset_name: str
    release_tag: str
    asset_name: str
    package_dir: str
    bag_dir: str
    rviz_relpath: str
    screenshot_relpaths: tuple[str, ...]
    hero_asset_relpath: str

    @property
    def preset(self) -> Preset:
        return get_preset(self.preset_name)

    @property
    def release_url(self) -> str:
        base = os.environ.get("PALLAS_DEMO_BASE_URL")
        if base:
            return f"{base.rstrip('/')}/{self.asset_name}"
        return (
            "https://github.com/RobotFlow-Labs/project_pallas_ros2/releases/download/"
            f"{self.release_tag}/{self.asset_name}"
        )


@dataclass(frozen=True)
class DemoPaths:
    demo: DemoAsset
    archive_path: Path
    install_root: Path
    package_root: Path
    bag_path: Path
    rviz_config: Path
    manifest_path: Path
    replay_log_dir: Path

    @property
    def screenshots(self) -> tuple[Path, ...]:
        root = repo_root()
        return tuple(root / relpath for relpath in self.demo.screenshot_relpaths)

    @property
    def hero_asset(self) -> Path:
        return repo_root() / self.demo.hero_asset_relpath


@dataclass(frozen=True)
class ReplayResult:
    ok: bool
    node_log: Path
    bag_log: Path
    seen_topics: tuple[str, ...]
    missing_topics: tuple[str, ...]
    wall_runtime_sec: float
    bag_returncode: int
    node_returncode: int


_DEMO_ASSETS = {
    "ouster-core-demo": DemoAsset(
        name="ouster-core-demo",
        title="Ouster Core Demo",
        description=(
            "Synthetic ROS2 bag for the shipped Ouster Core preset. "
            "Used for onboarding, screenshots, and replay smoke checks."
        ),
        preset_name="pallas_core_ouster.yaml",
        release_tag="demo-assets-v1",
        asset_name="pallas-demo-ouster-core.zip",
        package_dir="pallas-demo-ouster-core",
        bag_dir="bag/pallas_ouster_core_demo",
        rviz_relpath="rviz/pallas_demo.rviz",
        screenshot_relpaths=(
            "docs/media/pallas-demo-map-1.svg",
            "docs/media/pallas-demo-map-2.svg",
            "docs/media/pallas-demo-rviz.svg",
        ),
        hero_asset_relpath="docs/media/pallas-hero.svg",
    ),
}


def demo_cache_root() -> Path:
    return repo_root() / "artifacts" / "demos"


def demo_source_root() -> Path:
    return repo_root() / "artifacts" / "demo_assets"


def shipped_demos() -> list[DemoAsset]:
    return list(_DEMO_ASSETS.values())


def get_demo(name: str) -> DemoAsset:
    try:
        return _DEMO_ASSETS[name]
    except KeyError as exc:
        known = ", ".join(sorted(_DEMO_ASSETS))
        raise KeyError(f"Unknown demo: {name}. Known demos: {known}") from exc


def _archive_path(demo: DemoAsset) -> Path:
    return demo_cache_root() / "archives" / demo.asset_name


def _install_root(demo: DemoAsset) -> Path:
    return demo_cache_root() / demo.package_dir


def _demo_paths(demo: DemoAsset) -> DemoPaths:
    install_root = _install_root(demo)
    package_root = install_root
    return DemoPaths(
        demo=demo,
        archive_path=_archive_path(demo),
        install_root=install_root,
        package_root=package_root,
        bag_path=package_root / demo.bag_dir,
        rviz_config=package_root / demo.rviz_relpath,
        manifest_path=package_root / "manifest.txt",
        replay_log_dir=package_root / "logs",
    )


def expected_output_topics(demo: DemoAsset) -> tuple[str, ...]:
    preset = demo.preset
    return (
        preset.pose_topic,
        preset.odom_topic,
        preset.aligned_scan_topic,
        preset.map_topic,
    )


def demo_paths(name: str) -> DemoPaths:
    return _demo_paths(get_demo(name))


def _existing_install_is_complete(paths: DemoPaths) -> bool:
    return paths.package_root.is_dir() and paths.bag_path.exists() and paths.rviz_config.is_file()


def _local_asset_override(demo: DemoAsset) -> Path | None:
    source_dir = os.environ.get("PALLAS_DEMO_SOURCE_DIR")
    if source_dir:
        candidate = Path(source_dir) / demo.asset_name
        if candidate.is_file():
            return candidate

    candidate = demo_source_root() / demo.asset_name
    return candidate if candidate.is_file() else None


_MAX_DOWNLOAD_BYTES = 500 * 1024 * 1024  # 500 MB safety cap


def _safe_extractall(archive: zipfile.ZipFile, target: Path) -> None:
    resolved_target = target.resolve()
    for member in archive.namelist():
        member_path = (resolved_target / member).resolve()
        if not member_path.is_relative_to(resolved_target):
            raise RuntimeError(f"Zip entry escapes target directory: {member}")
    archive.extractall(target)


def _download_archive(demo: DemoAsset, archive_path: Path) -> None:
    url = demo.release_url
    if not url.startswith("https://"):
        raise RuntimeError(f"Refusing non-HTTPS download URL: {url}")
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(url) as response:
        data = response.read(_MAX_DOWNLOAD_BYTES + 1)
        if len(data) > _MAX_DOWNLOAD_BYTES:
            raise RuntimeError(f"Download exceeds {_MAX_DOWNLOAD_BYTES} byte limit")
        archive_path.write_bytes(data)


def fetch_demo(name: str, *, force: bool = False) -> DemoPaths:
    demo = get_demo(name)
    paths = _demo_paths(demo)

    if force and paths.install_root.exists():
        shutil.rmtree(paths.install_root)
    if force and paths.archive_path.exists():
        paths.archive_path.unlink()

    if _existing_install_is_complete(paths):
        return paths

    paths.archive_path.parent.mkdir(parents=True, exist_ok=True)
    local_override = _local_asset_override(demo)
    if local_override:
        shutil.copyfile(local_override, paths.archive_path)
    elif not paths.archive_path.is_file():
        _download_archive(demo, paths.archive_path)

    if paths.install_root.exists():
        shutil.rmtree(paths.install_root)
    paths.install_root.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(paths.archive_path) as archive:
        _safe_extractall(archive, demo_cache_root())

    if not _existing_install_is_complete(paths):
        raise RuntimeError(
            f"Demo archive for {demo.name} is incomplete. "
            f"Expected bag at {paths.bag_path} and RViz config at {paths.rviz_config}."
        )
    return paths


def bag_info(paths: DemoPaths, *, timeout_sec: float = 10.0) -> str:
    result = run_capture(
        ["ros2", "bag", "info", str(paths.bag_path)],
        source_ros=True,
        timeout=timeout_sec,
    )
    if result.returncode != 0:
        stderr = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(stderr or f"ros2 bag info failed for {paths.bag_path}")
    return result.stdout.strip()


def bag_duration_seconds(info_text: str) -> float | None:
    for line in info_text.splitlines():
        stripped = line.strip()
        if stripped.lower().startswith("duration:"):
            value = stripped.split(":", 1)[1].strip()
            if value.endswith("s"):
                value = value[:-1].strip()
            try:
                return float(value)
            except ValueError:
                return None
    return None


def _topic_inventory(timeout_sec: float) -> set[str]:
    result = run_capture(["ros2", "topic", "list"], source_ros=True, timeout=timeout_sec)
    if result.returncode != 0:
        stderr = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(stderr or "ros2 topic list failed")
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def run_demo_replay(
    name: str,
    *,
    rate: float = 1.0,
    node_startup_sec: float = 1.5,
    topic_timeout_sec: float = 8.0,
    verify_topics: bool = True,
) -> ReplayResult:
    paths = fetch_demo(name)
    preset = paths.demo.preset
    overlay = workspace_setup()
    if not overlay:
        raise RuntimeError("Workspace is not built yet. Run `uv run pallas-dev build` first.")

    paths.replay_log_dir.mkdir(parents=True, exist_ok=True)
    node_log = paths.replay_log_dir / "pallas_node.log"
    bag_log = paths.replay_log_dir / "bag_play.log"

    node_cmd = [
        "ros2",
        "run",
        "anima_pallas_ros2",
        preset.node_name,
        "--ros-args",
        "--params-file",
        str(preset.path),
    ]
    bag_cmd = ["ros2", "bag", "play", str(paths.bag_path), "--rate", str(rate)]

    start = time.perf_counter()
    node_proc: subprocess.Popen[str] | None = None
    bag_proc: subprocess.Popen[str] | None = None
    missing_topics = expected_output_topics(paths.demo)
    seen_topics: tuple[str, ...] = ()

    bag_returncode = -1
    node_returncode = -1
    try:
        with node_log.open("w", encoding="utf-8") as node_handle, bag_log.open(
            "w", encoding="utf-8"
        ) as bag_handle:
            node_proc = subprocess.Popen(
                prepare_command(node_cmd, source_ros=True),
                cwd=repo_root(),
                env=command_env(),
                stdout=node_handle,
                stderr=subprocess.STDOUT,
                text=True,
            )
            time.sleep(node_startup_sec)

            bag_proc = subprocess.Popen(
                prepare_command(bag_cmd, source_ros=True),
                cwd=repo_root(),
                env=command_env(),
                stdout=bag_handle,
                stderr=subprocess.STDOUT,
                text=True,
            )

            if verify_topics:
                per_call_timeout = min(3.0, topic_timeout_sec)
                deadline = time.monotonic() + topic_timeout_sec
                expected = set(expected_output_topics(paths.demo))
                while time.monotonic() < deadline:
                    seen = _topic_inventory(per_call_timeout)
                    found = expected & seen
                    missing = expected - seen
                    seen_topics = tuple(sorted(found))
                    if not missing:
                        missing_topics = ()
                        break
                    missing_topics = tuple(sorted(missing))
                    time.sleep(0.5)

            bag_returncode = bag_proc.wait(timeout=max(10.0, topic_timeout_sec + 5.0))
    finally:
        for proc in (bag_proc, node_proc):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        if node_proc is not None and node_proc.returncode is not None:
            node_returncode = node_proc.returncode

    wall_runtime = time.perf_counter() - start
    ok = bag_returncode == 0 and (not verify_topics or not missing_topics)
    return ReplayResult(
        ok=ok,
        node_log=node_log,
        bag_log=bag_log,
        seen_topics=seen_topics,
        missing_topics=missing_topics,
        wall_runtime_sec=wall_runtime,
        bag_returncode=bag_returncode,
        node_returncode=node_returncode,
    )
