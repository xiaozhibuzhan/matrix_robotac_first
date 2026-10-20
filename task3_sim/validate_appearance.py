"""Controlled real-render photo-target regression; not a real-world guarantee."""

from __future__ import annotations

import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import platform
import sys
import uuid

from .__main__ import PACKAGE, PROJECT, parse_args, run, write_json


def scenarios(base):
    """Fixed acceptance settings; do not tune the controller for each case."""
    yield "photo_baseline", copy.deepcopy(base), "SUCCESS", 10, 0.01, 12.0
    for name, brightness, tile in (("background_dark", 0.45, [1, 1]),
                                   ("background_bright", 1.35, [1, 1]),
                                   ("background_tiled", 1.0, [3, 2])):
        config = copy.deepcopy(base)
        config["appearance"]["background"].update(brightness=brightness, tile=tile)
        yield name, config, "SUCCESS", 9, 0.02, 12.0
    for name, effects in (("sensor_dim", {"exposure": 0.65, "gamma": 1.1}),
                          ("sensor_bright", {"exposure": 1.3}),
                          ("sensor_blur", {"blur_sigma_px": 0.8}),
                          ("sensor_noise", {"noise_std_u8": 8.0, "seed": 42}),
                          ("sensor_combined", {"exposure": 0.8, "blur_sigma_px": 0.5,
                                               "noise_std_u8": 4.0, "seed": 43})):
        config = copy.deepcopy(base)
        config["appearance"]["camera_effects"] = effects
        yield name, config, "SUCCESS", 9, 0.02, 12.0
    left = copy.deepcopy(base)
    left["simulation"]["initial_rpy_rad"] = [0, 0.10, -0.12]
    yield "left_start", left, "SUCCESS", 9, 0.02, 12.0
    oblique = copy.deepcopy(base)
    oblique["simulation"].update(target_yaw_rad=0.3, target_center_world_m=[2.3, -0.2, 0.55],
                                  initial_rpy_rad=[0, 0.04, -0.13])
    yield "oblique_target", oblique, "SUCCESS", 9, 0.02, 12.0
    far = copy.deepcopy(base)
    far["simulation"]["target_center_world_m"] = [3.3, 0.25, 0.5]
    yield "farther_target", far, "SUCCESS", 9, 0.02, 12.0
    for name, expected in (("beam_blocked", "MISSED"), ("tag_out_of_view", "TIMEOUT"),
                            ("no_ammo", "NO_AMMO")):
        config = copy.deepcopy(base)
        if name == "beam_blocked":
            config["simulation"]["obstacles"] = [{"position_m": [1.8, 0.18, 0.45],
                                                       "half_size_m": [0.03, 0.06, 0.025]}]
        elif name == "tag_out_of_view":
            config["simulation"]["target_center_world_m"] = [2.5, 4.0, 0.5]
        else:
            config["simulation"]["ammo_count"] = 0
        yield name, config, expected, None, None, 1.0 if name == "tag_out_of_view" else 12.0


def assess(summary, process_code, config, expected, min_ring, max_miss):
    checks = {"status": summary.get("status") == expected,
              "exit_code": process_code == (0 if expected == "SUCCESS" else 2),
              "model_unchanged": summary.get("model_sources_unchanged") is True,
              "shutdown_completed": summary.get("shutdown_completed") is True,
              "perception_closed": summary.get("perception_closed") is True,
              "cleanup_errors": summary.get("cleanup_errors") == []}
    shot = summary.get("shot")
    if expected == "SUCCESS":
        actual = summary.get("diagnostics", {}).get("actual_miss_m")
        checks.update(ring=shot is not None and shot["ring"] >= min_ring,
                      finite_ammo=summary.get("ammo_remaining") == config["simulation"]["ammo_count"]-1,
                      actual_miss=actual is not None and actual <= max_miss)
    elif expected == "MISSED":
        checks.update(occluded=shot is not None and shot["reason"] == "occluded" and shot["ring"] == 3,
                      finite_ammo=summary.get("ammo_remaining") == config["simulation"]["ammo_count"]-1)
    else:
        checks.update(no_shot=shot is None,
                      finite_ammo=summary.get("ammo_remaining") == config["simulation"]["ammo_count"])
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PACKAGE/"config/photo_target.json")
    parser.add_argument("--model-root", type=Path, default=PROJECT/"src/robot_mujoco/zsibot_robots/xgb")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    output = (args.output or PACKAGE/"runs"/f"appearance_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory must be new or empty")
    output.mkdir(parents=True, exist_ok=True)
    base = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if base.get("appearance", {}).get("target", {}).get("kind") != "image":
        parser.error("appearance validation requires an image target config")
    results = []
    for name, config, expected, min_ring, max_miss, limit in scenarios(base):
        config_path = output/f"{name}.json"
        write_json(config_path, config)
        process_code, run_path = run(parse_args(["--headless", "--model-root", str(args.model_root),
            "--config", str(config_path), "--output", str(output/name), "--max-seconds", str(limit)]))
        summary = json.loads((run_path/"summary.json").read_text(encoding="utf-8"))
        checks = assess(summary, process_code, config, expected, min_ring, max_miss)
        events = [json.loads(line) for line in (run_path/"events.jsonl").read_text(encoding="utf-8").splitlines()] if (run_path/"events.jsonl").exists() else []
        detection_rate = sum(bool(event["detections"]) for event in events)/len(events) if events else 0
        result = {"case": name, "passed": all(checks.values()), "checks": checks,
                  "status": summary["status"], "ring": (summary.get("shot") or {}).get("ring"),
                  "actual_miss_m": summary.get("diagnostics", {}).get("actual_miss_m"),
                  "detection_frame_fraction": detection_rate,
                  "simulation_time_s": summary.get("simulation_time_s"),
                  "expected": {"status": expected, "min_ring": min_ring, "max_actual_miss_m": max_miss},
                  "summary": str(run_path/"summary.json")}
        results.append(result)
        # Save partial evidence as well if a later native process failure occurs.
        write_json(output/"validation.json", {"complete": False, "cases": results})
        print(f"APPEARANCE_CASE {name}: {'PASS' if result['passed'] else 'FAIL'}", flush=True)
    report = {"complete": True, "all_passed": all(result["passed"] for result in results),
              "platform": platform.platform(), "python": platform.python_version(),
              "ubuntu_runtime_tested": sys.platform.startswith("linux"),
              "scope": "15 fixed real-render scenarios; one artwork/background and deterministic noise seeds. Not statistical or real-world robustness proof.",
              "metric_scale": "Assumed tag black edge from config; target photo supplies no verified physical dimensions.",
              "cases": results}
    write_json(output/"validation.json", report)
    print(f"Appearance validation: {output/'validation.json'}", flush=True)
    return 0 if report["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
