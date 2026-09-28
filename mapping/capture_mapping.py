#!/usr/bin/env python3
"""Record the original inputs without building a growing map during motion."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
from typing import Sequence


def record_command(bag_dir: Path, topic: str, odom_topic: str, ros2: str) -> list[str]:
    # Both official sensor publishers can be best effort. A reliable recorder
    # subscription would not match them. Keep a bounded DDS inbox plus rosbag's
    # bounded write cache; no cloud geometry is decoded here.
    qos_path = bag_dir.parent / "capture_qos.yaml"
    qos = {name: {"history": "keep_last", "depth": depth,
                  "reliability": "best_effort", "durability": "volatile"}
           for name, depth in ((topic, 100), (odom_topic, 2000))}
    # JSON is also YAML; JSON quoting keeps arbitrary ROS names out of syntax.
    qos_path.write_text(json.dumps(qos, indent=2) + "\n", encoding="utf-8")
    return [ros2, "bag", "record", "--storage", "sqlite3", "--output", str(bag_dir),
            "--max-cache-size", "67108864", "--qos-profile-overrides-path", str(qos_path),
            topic, odom_topic]


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", type=Path, required=True)
    parser.add_argument("--topic", default="/front_lidar")
    parser.add_argument("--odom-topic", default="/odom/mujoco_odom")
    args = parser.parse_args(argv)
    bag_dir = args.bag_dir.expanduser().resolve()
    ros2 = shutil.which("ros2")
    if ros2 is None:
        print("ros2 is unavailable; source the ROS environment first.", file=sys.stderr)
        return 1
    if bag_dir.exists():
        print(f"Refusing to overwrite an existing capture: {bag_dir}", file=sys.stderr)
        return 1
    if args.topic == args.odom_topic or any(not name.startswith("/") for name in (args.topic, args.odom_topic)):
        print("Use two distinct absolute ROS topic names.", file=sys.stderr)
        return 1
    try:
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
        command = record_command(bag_dir, args.topic, args.odom_topic, ros2)
        print(f"Capture only: {args.topic} + {args.odom_topic} -> {bag_dir}", flush=True)
        print("Move while scanning. Stop this capture before running finalize_pcd.sh.", flush=True)
        # Replace this PID rather than spawn a grandchild: the launcher's SIGINT
        # goes straight to rosbag so it flushes SQLite and writes metadata.yaml.
        os.execv(ros2, command)
    except OSError as exc:
        print(f"Cannot start raw capture: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
