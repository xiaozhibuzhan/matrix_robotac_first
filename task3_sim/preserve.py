"""Read-only integrity snapshot for existing competition code and assets.

This helper never writes inside a protected path. Huge recorded ROS bags are
tracked by size/mtime only; its report states that limitation explicitly.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys


DEFAULT_PATHS = (
    "src/robot_mujoco", "src/robot_mc", "deps", "config", "scripts",
    "navigation", "mapping", "maps", "scene_tools", "scene_updates",
    "run_sim.sh", "field_map.pcd", "README.md", "README_GPT.md",
    "TASK2_PLAN.md", "TASK3_PLAN.md", "PROJECT_STATUS.md", "CHANGELOG.md",
    "src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.umap",
    "src/UeSim/Linux/UeSim/Content/Maps/RobotAC/RobotAC_Level_1.uexp",
)
SKIP_PARTS = {".git", ".venv", "__pycache__", "build", "install", "log"}
METADATA_EXTENSIONS = {".db3", ".mcap"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inventory(root: Path, paths: list[str]) -> dict:
    root = root.resolve()
    files: dict[str, dict] = {}
    missing = []
    for relative in paths:
        target = (root / relative).resolve()
        if not target.is_relative_to(root):
            raise ValueError(f"Protected path escapes project: {relative}")
        if not target.exists():
            missing.append(relative)
            continue
        candidates = [target] if target.is_file() else sorted(target.rglob("*"))
        for path in candidates:
            if not path.is_file() or any(p in SKIP_PARTS for p in path.relative_to(root).parts):
                continue
            resolved = path.resolve()
            if not resolved.is_relative_to(root):
                raise ValueError(f"Protected symlink escapes project: {path}")
            relative_name = path.relative_to(root).as_posix()
            stat = path.stat()
            entry = {"size": stat.st_size}
            if path.suffix.lower() in METADATA_EXTENSIONS:
                entry.update(mode="metadata_only", mtime_ns=stat.st_mtime_ns)
            else:
                entry.update(mode="sha256", sha256=sha256(path))
            files[relative_name] = entry
    return {
        "format_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "project_root": str(root),
        "protected_paths": paths,
        "excluded_directory_names": sorted(SKIP_PARTS),
        "metadata_only_extensions": sorted(METADATA_EXTENSIONS),
        "missing_paths": missing,
        "files": files,
    }


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("snapshot", "verify"))
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--path", action="append", help="Override protected paths; repeat as needed")
    parser.add_argument("--report", type=Path, help="Optional JSON verification report")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    if args.action == "snapshot":
        result = inventory(root, args.path or list(DEFAULT_PATHS))
        write_json(args.manifest, result)
        files = result["files"]
        print(f"Snapshot: {len(files)} files, {sum(v['mode'] == 'sha256' for v in files.values())} SHA-256, "
              f"{sum(v['mode'] == 'metadata_only' for v in files.values())} metadata only")
        print(f"Manifest: {args.manifest.resolve()}")
        return 0
    before = json.loads(args.manifest.read_text(encoding="utf-8"))
    after = inventory(root, before["protected_paths"])
    old, new = before["files"], after["files"]
    changed = [p for p in sorted(old.keys() & new.keys()) if old[p] != new[p]]
    result = {
        "unchanged": not (changed or old.keys() - new.keys() or new.keys() - old.keys()),
        "changed": changed, "deleted": sorted(old.keys() - new.keys()),
        "added": sorted(new.keys() - old.keys()),
        "files_checked": len(old),
        "sha256_files": sum(v["mode"] == "sha256" for v in old.values()),
        "metadata_only_files": sum(v["mode"] == "metadata_only" for v in old.values()),
        "excluded_directory_names": before["excluded_directory_names"],
        "protected_paths": before["protected_paths"],
        "missing_paths": after["missing_paths"],
    }
    if args.report:
        write_json(args.report, result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["unchanged"] else 1


if __name__ == "__main__":
    sys.exit(main())
