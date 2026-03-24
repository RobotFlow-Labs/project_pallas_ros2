from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from .rosenv import repo_root

_PRESET_PATTERN = re.compile(r"^pallas_(core|ct)(?:_([a-z0-9]+))?\.yaml$")
_EXPECTED_NODE_NAME = {
    "core": "anima_pallas_core_node",
    "ct": "anima_pallas_ct_node",
}
_REQUIRED_KEYS = (
    "lidar_type",
    "pointcloud_topic",
    "imu_topic",
    "pose_topic",
    "odom_topic",
    "map_topic",
    "aligned_scan_topic",
    "odom_frame",
    "base_frame",
)
_VENDOR_LABELS = {
    "generic": "Generic",
    "unitree": "Unitree",
    "livox": "Livox",
    "ouster": "Ouster",
    "hesai": "Hesai",
    "robosense": "RoboSense",
    "rslidar": "RoboSense/rslidar",
    "velodyne": "Velodyne",
}
_VENDOR_NOTES = {
    "generic": "Bring your own topics, frames, and range limits.",
    "unitree": "Set measured LiDAR-to-IMU extrinsics before field use.",
    "livox": "Verify Livox frame wiring and measured extrinsics.",
    "ouster": "Verify the Ouster driver frame mapping against your install.",
    "hesai": "Confirm Hesai topic names and measured extrinsics.",
    "robosense": "Check RoboSense frame names and measured extrinsics.",
    "rslidar": "Alias of the RoboSense defaults used by rslidar_sdk.",
    "velodyne": "Requires an external IMU on /imu/data plus measured extrinsics.",
}


@dataclass(frozen=True)
class Preset:
    name: str
    profile: str
    vendor_key: str
    vendor_label: str
    lidar_type: str
    path: Path
    node_name: str
    pointcloud_topic: str
    imu_topic: str
    pose_topic: str
    odom_topic: str
    map_topic: str
    aligned_scan_topic: str
    odom_frame: str
    base_frame: str
    note: str

    @property
    def launch_file(self) -> str:
        return f"pallas_{self.profile}.launch.py"

    @property
    def launch_path(self) -> Path:
        return (
            repo_root()
            / "ros2_ws"
            / "src"
            / "anima_pallas_ros2"
            / "launch"
            / self.launch_file
        )

    @property
    def launch_command(self) -> str:
        return (
            "ros2 launch anima_pallas_ros2 "
            f"{self.launch_file} config_name:={self.name}"
        )

    @property
    def ros2_run_command(self) -> str:
        return (
            f"ros2 run anima_pallas_ros2 {self.node_name} "
            f"--ros-args --params-file {self.path}"
        )

    @property
    def live_launch_command(self) -> str:
        return f"uv run pallas-dev launch-live {self.name}"


def preset_config_dir() -> Path:
    return repo_root() / "ros2_ws" / "src" / "anima_pallas_ros2" / "config"


def load_preset(path: Path) -> Preset:
    match = _PRESET_PATTERN.match(path.name)
    if not match:
        raise ValueError(f"Unsupported preset file name: {path.name}")

    profile = match.group(1)
    vendor_key = match.group(2) or "generic"
    vendor_label = _VENDOR_LABELS.get(vendor_key, vendor_key.title())
    expected_node_name = _EXPECTED_NODE_NAME[profile]

    with path.open("r", encoding="utf-8") as handle:
        parsed = yaml.safe_load(handle)

    if not isinstance(parsed, dict) or len(parsed) != 1:
        raise ValueError(f"{path.name} must contain exactly one top-level ROS node block")

    node_name = next(iter(parsed))
    if node_name != expected_node_name:
        raise ValueError(
            f"{path.name} uses node {node_name}, expected {expected_node_name} for {profile}"
        )

    node_block = parsed[node_name]
    if not isinstance(node_block, dict):
        raise ValueError(f"{path.name} node block must be a mapping")

    params = node_block.get("ros__parameters")
    if not isinstance(params, dict):
        raise ValueError(f"{path.name} is missing ros__parameters")

    missing_keys = [key for key in _REQUIRED_KEYS if key not in params]
    if missing_keys:
        missing = ", ".join(missing_keys)
        raise ValueError(f"{path.name} is missing required parameters: {missing}")

    return Preset(
        name=path.name,
        profile=profile,
        vendor_key=vendor_key,
        vendor_label=vendor_label,
        lidar_type=str(params["lidar_type"]),
        path=path,
        node_name=node_name,
        pointcloud_topic=str(params["pointcloud_topic"]),
        imu_topic=str(params["imu_topic"]),
        pose_topic=str(params["pose_topic"]),
        odom_topic=str(params["odom_topic"]),
        map_topic=str(params["map_topic"]),
        aligned_scan_topic=str(params["aligned_scan_topic"]),
        odom_frame=str(params["odom_frame"]),
        base_frame=str(params["base_frame"]),
        note=_VENDOR_NOTES.get(vendor_key, "Check topics, frames, and extrinsics."),
    )


def shipped_presets(*, profile: str | None = None) -> list[Preset]:
    presets: list[Preset] = []
    for path in sorted(preset_config_dir().glob("pallas_*.yaml")):
        if path.name == "pallas.yaml":
            continue
        preset = load_preset(path)
        if profile and preset.profile != profile:
            continue
        presets.append(preset)
    return presets


def get_preset(name: str) -> Preset:
    preset_name = name if name.endswith(".yaml") else f"{name}.yaml"
    path = preset_config_dir() / preset_name
    if not path.is_file():
        raise KeyError(f"Unknown preset: {name}")
    return load_preset(path)
