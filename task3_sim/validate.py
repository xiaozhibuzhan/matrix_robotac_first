"""Repeatable actual-render integration checks, including no-fire failure cases."""

from __future__ import annotations

import argparse
import ast
import copy
from datetime import datetime
import json
from pathlib import Path
import platform
import sys
import uuid

from .__main__ import PACKAGE, PROJECT, parse_args, run, write_json
from .package_update import source_files


def scenarios(base):
    yield "default", copy.deepcopy(base), "SUCCESS", 10, 12.0
    left = copy.deepcopy(base)
    left["simulation"]["initial_rpy_rad"] = [0, 0.10, -0.12]
    yield "left_start", left, "SUCCESS", 6, 12.0
    oblique = copy.deepcopy(base)
    oblique["simulation"]["target_yaw_rad"] = 0.3
    oblique["simulation"]["target_center_world_m"] = [2.3, -0.2, 0.55]
    oblique["simulation"]["initial_rpy_rad"] = [0, 0.04, -0.13]
    yield "oblique_target", oblique, "SUCCESS", 6, 12.0
    blocked = copy.deepcopy(base)
    # A low obstacle blocks the physical beam while the elevated tag stays
    # visible. The referee must see the nearer surface, not the hidden target.
    blocked["simulation"]["obstacles"] = [{"position_m": [1.8, 0.18, 0.45], "half_size_m": [0.03, 0.06, 0.025]}]
    yield "beam_blocked", blocked, "MISSED", None, 12.0
    lost = copy.deepcopy(base)
    lost["simulation"]["target_center_world_m"] = [2.5, 4.0, 0.5]
    yield "tag_out_of_view", lost, "TIMEOUT", None, 1.0
    empty = copy.deepcopy(base)
    empty["simulation"]["ammo_count"] = 0
    yield "no_ammo", empty, "NO_AMMO", None, 12.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--model-root", type=Path, default=PROJECT/"src/robot_mujoco/zsibot_robots/xgb")
    args = parser.parse_args(argv)
    output = (args.output or PACKAGE/"runs"/f"validation_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    base = json.loads((PACKAGE/"config/single_target.json").read_text(encoding="utf-8"))
    results = []
    for name, config, expected, min_ring, limit in scenarios(base):
        config_path = output/f"{name}.json"
        write_json(config_path, config)
        _, run_path = run(parse_args(["--headless", "--model-root", str(args.model_root),
            "--config", str(config_path), "--output", str(output/name), "--max-seconds", str(limit)]))
        summary = json.loads((run_path/"summary.json").read_text(encoding="utf-8"))
        passed = summary["status"] == expected and summary.get("model_sources_unchanged", False)
        shot = summary.get("shot")
        if min_ring is not None:
            passed &= shot is not None and shot["ring"] >= min_ring and summary["ammo_remaining"] == 4
        if name == "beam_blocked":
            passed &= shot is not None and shot["reason"] == "occluded" and shot["ring"] == 3
        if name in ("tag_out_of_view", "no_ammo"):
            passed &= shot is None and summary["ammo_remaining"] == config["simulation"]["ammo_count"]
        results.append({"case": name, "passed": bool(passed), "status": summary["status"],
                        "ring": shot["ring"] if shot else None, "simulation_time_s": summary.get("simulation_time_s"),
                        "summary": str(run_path/"summary.json")})
        print(f"CASE {name}: {'PASS' if passed else 'FAIL'}", flush=True)
    source_paths = [p for p in source_files(PACKAGE) if p.suffix == ".py"]
    for path in source_paths:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3,10))
    report = {"all_passed": all(result["passed"] for result in results),
              "platform": platform.platform(), "python": platform.python_version(),
              "ubuntu_runtime_tested": sys.platform.startswith("linux"),
              "python310_grammar_files": len(source_paths), "cases": results}
    write_json(output/"validation.json", report)
    print(f"Validation report: {output/'validation.json'}")
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
