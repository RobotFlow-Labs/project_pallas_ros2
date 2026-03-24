#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from pallas_dev.demo import (  # noqa: E402
    bag_duration_seconds,
    bag_info,
    demo_cache_root,
    expected_output_topics,
    fetch_demo,
    get_demo,
    run_demo_replay,
)


def render_markdown(summary: dict[str, object]) -> str:
    lines = [
        f"# Demo Benchmark: {summary['demo']}",
        "",
        f"- title: `{summary['title']}`",
        f"- preset: `{summary['preset']}`",
        f"- bag_path: `{summary['bag_path']}`",
        f"- rviz_config: `{summary['rviz_config']}`",
        f"- bag_duration_sec: `{summary['bag_duration_sec']}`",
        f"- wall_runtime_sec: `{summary['wall_runtime_sec']}`",
        f"- replay_ok: `{summary['replay_ok']}`",
        f"- hero_asset: `{summary['hero_asset']}`",
        "",
        "## Output Topics",
        "",
    ]
    for topic in summary["output_topics"]:
        lines.append(f"- `{topic}`")
    lines.extend(["", "## Screenshot Paths", ""])
    for path in summary["screenshot_paths"]:
        lines.append(f"- `{path}`")
    lines.extend(["", "## Bag Info", "", "```text", str(summary["bag_info"]), "```"])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run or summarize the canonical PALLAS demo replay.")
    parser.add_argument("demo", nargs="?", default="ouster-core-demo")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "artifacts" / "benchmarks")
    parser.add_argument("--run-replay", action="store_true")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    demo = get_demo(args.demo)
    paths = fetch_demo(demo.name)
    info = bag_info(paths)
    replay = None
    if args.run_replay:
        replay = run_demo_replay(demo.name, rate=args.rate)

    summary = {
        "demo": demo.name,
        "title": demo.title,
        "preset": demo.preset_name,
        "bag_path": str(paths.bag_path),
        "rviz_config": str(paths.rviz_config),
        "bag_duration_sec": bag_duration_seconds(info),
        "wall_runtime_sec": round(replay.wall_runtime_sec, 3) if replay else None,
        "replay_ok": replay.ok if replay else None,
        "node_log": str(replay.node_log) if replay else None,
        "bag_log": str(replay.bag_log) if replay else None,
        "output_topics": list(expected_output_topics(demo)),
        "hero_asset": str(paths.hero_asset),
        "screenshot_paths": [str(path) for path in paths.screenshots],
        "bag_info": info,
        "demo_cache_root": str(demo_cache_root()),
    }

    target_dir = args.output_dir / demo.name
    target_dir.mkdir(parents=True, exist_ok=True)
    json_path = target_dir / "summary.json"
    md_path = target_dir / "summary.md"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md_path.write_text(render_markdown(summary) + "\n", encoding="utf-8")
    print(md_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
