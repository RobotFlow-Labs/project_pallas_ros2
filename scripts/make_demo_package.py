#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import zipfile


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from pallas_dev.demo import demo_source_root, get_demo  # noqa: E402


def build_point_cloud(
    stamp,
    *,
    frame_id: str = "os_sensor",
    points_per_ring: int = 12,
    rings: int = 8,
):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
    ]

    raw = bytearray()
    total = points_per_ring * rings
    for ring in range(rings):
        for column in range(points_per_ring):
            phase = (2.0 * math.pi * column) / points_per_ring
            radius = 5.0 + 0.05 * ring
            x = radius
            y = math.cos(phase) * 0.6
            z = -0.25 + 0.07 * ring + math.sin(phase) * 0.05
            intensity = 30.0 + ring
            relative_time = float(column) / float(total)
            raw.extend(struct.pack("<ffffHf", x, y, z, intensity, ring, relative_time))

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = total
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 22
    msg.row_step = msg.point_step * total
    msg.is_dense = True
    msg.data = bytes(raw)
    return msg


def publish_fixture(duration_sec: float) -> None:
    import rclpy
    from sensor_msgs.msg import Imu, PointCloud2

    rclpy.init()
    node = rclpy.create_node("pallas_demo_fixture_publisher")
    cloud_pub = node.create_publisher(PointCloud2, "/ouster/points", 10)
    imu_pub = node.create_publisher(Imu, "/ouster/imu", 50)

    start = node.get_clock().now()
    end_time = start.nanoseconds + int(duration_sec * 1e9)
    next_cloud = start.nanoseconds
    cloud_period_ns = int(0.1 * 1e9)
    imu_period_ns = int(0.01 * 1e9)
    next_imu = start.nanoseconds

    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end_time:
            now = node.get_clock().now()
            if now.nanoseconds >= next_cloud:
                cloud_pub.publish(build_point_cloud(now.to_msg()))
                next_cloud += cloud_period_ns
            if now.nanoseconds >= next_imu:
                imu = Imu()
                imu.header.stamp = now.to_msg()
                imu.header.frame_id = "os_imu"
                imu.linear_acceleration.z = 9.80665
                imu.angular_velocity.z = 0.01
                cloud_phase = (now.nanoseconds - start.nanoseconds) / 1e9
                imu.angular_velocity.x = math.sin(cloud_phase) * 0.005
                imu.angular_velocity.y = math.cos(cloud_phase) * 0.005
                imu_pub.publish(imu)
                next_imu += imu_period_ns
            rclpy.spin_once(node, timeout_sec=0.005)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def package_demo(demo_name: str, output_dir: Path, overwrite: bool) -> Path:
    demo = get_demo(demo_name)
    output_dir.mkdir(parents=True, exist_ok=True)
    archive_path = output_dir / demo.asset_name
    if archive_path.exists() and not overwrite:
        raise FileExistsError(f"{archive_path} already exists. Pass --overwrite to replace it.")

    rviz_source = REPO_ROOT / "ros2_ws" / "src" / "anima_pallas_ros2" / "rviz" / "pallas_demo.rviz"
    if not rviz_source.is_file():
        raise FileNotFoundError(f"Missing RViz config: {rviz_source}")

    with tempfile.TemporaryDirectory(prefix="pallas-demo-") as tmp_dir:
        tmp_root = Path(tmp_dir)
        package_root = tmp_root / demo.package_dir
        bag_path = package_root / demo.bag_dir
        rviz_target = package_root / demo.rviz_relpath
        rviz_target.parent.mkdir(parents=True, exist_ok=True)
        bag_path.parent.mkdir(parents=True, exist_ok=True)

        record_cmd = ["ros2", "bag", "record", "/ouster/points", "/ouster/imu", "-o", str(bag_path)]
        record_proc = subprocess.Popen(
            record_cmd,
            cwd=REPO_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        try:
            time.sleep(2.0)
            publish_proc = subprocess.run(
                [sys.executable, str(Path(__file__)), "publish-fixture", "--duration-sec", "4.5"],
                cwd=REPO_ROOT,
                check=True,
            )
            _ = publish_proc
            time.sleep(1.0)
        finally:
            record_proc.send_signal(signal.SIGINT)
            record_proc.wait(timeout=15.0)

        shutil.copyfile(rviz_source, rviz_target)
        manifest = package_root / "manifest.txt"
        manifest.write_text(
            "\n".join(
                [
                    f"name={demo.name}",
                    f"title={demo.title}",
                    f"preset={demo.preset_name}",
                    f"bag={demo.bag_dir}",
                    f"rviz={demo.rviz_relpath}",
                ]
            )
            + "\n",
            encoding="utf-8",
        )

        if archive_path.exists():
            archive_path.unlink()
        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(package_root.rglob("*")):
                if path.is_dir():
                    continue
                archive.write(path, path.relative_to(tmp_root))

    return archive_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create or publish the canonical PALLAS demo package.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    package_cmd = subparsers.add_parser("package", help="Record and package the canonical demo rosbag.")
    package_cmd.add_argument("--demo", default="ouster-core-demo")
    package_cmd.add_argument("--output-dir", type=Path, default=demo_source_root())
    package_cmd.add_argument("--overwrite", action="store_true")

    publish_cmd = subparsers.add_parser("publish-fixture", help="Publish synthetic Ouster-style ROS2 messages.")
    publish_cmd.add_argument("--duration-sec", type=float, default=4.5)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "publish-fixture":
        publish_fixture(args.duration_sec)
        return 0
    if args.command == "package":
        archive = package_demo(args.demo, args.output_dir, args.overwrite)
        print(archive)
        return 0
    parser.error(f"unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
