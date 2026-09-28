"""Reproduce local voxel-fusion/output timings; this is not an Ubuntu ROS run.

Run from any directory with Python and NumPy:
    python mapping/tests/efficiency_benchmark.py

Only a JSON report is written, by default under diagnostics/efficiency_20260908.
The baseline is preserved Python source in that directory; no PCD is read or
modified. Timed work is accepted-scan fusion and map-array preparation, excluding
point decoding, filtering, ROS, RViz and file I/O. Results do not establish safe
driving speed or target-system throughput.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np


MAPPING = Path(__file__).resolve().parents[1]
DIAGNOSTICS = MAPPING / "diagnostics" / "efficiency_20260908"
sys.path.insert(0, str(MAPPING))
from voxel_fusion import VoxelFusion


def source_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def baseline_type(path: Path):
    specification = importlib.util.spec_from_file_location("efficiency_baseline_voxel_fusion", path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"Cannot import baseline from {path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module.VoxelFusion


def summary(values: list[float]) -> dict[str, float | list[float]]:
    return {"median_ms": float(np.median(values)), "p95_ms": float(np.percentile(values, 95)),
            "minimum_ms": min(values), "samples_ms": values}


def measure_case(size: int, scan_points: int, rounds: int, rng, baseline) -> dict:
    voxel = 0.05
    point_count = min(size, scan_points)
    initial = (rng.integers([-1000, -1000, -100], [1000, 1000, 100], size=(size, 3)) + 0.1) * voxel
    scans = [initial[:point_count] + rng.uniform(0, 0.035, (point_count, 3)) for _ in range(rounds)]
    # min_observations=1 bypasses initial confidence warm-up, isolating repeated
    # updates of a confirmed map. Per-scan means/max_weight retain real defaults.
    implementations = [baseline(voxel_size=voxel, min_observations=1),
                       VoxelFusion(voxel_size=voxel, min_observations=1)]
    for fusion in implementations:
        fusion.integrate(initial, 1.0)
    implementations[1].snapshot()
    timings = [{"integrate": [], "snapshot": [], "combined": []} for _ in implementations]
    for index, scan in enumerate(scans):
        changed = [None, None]
        arrays = [None, None]
        # Alternate order to avoid always charging one side for transient load.
        for which in ((0, 1) if index % 2 == 0 else (1, 0)):
            fusion = implementations[which]
            started = time.perf_counter()
            changed[which] = fusion.integrate(scan, 2.0 + index * 0.1)
            integrated = time.perf_counter()
            arrays[which] = (np.asarray(list(fusion.points.values()), dtype="<f4").reshape(-1, 3)
                             if which == 0 else fusion.snapshot())
            finished = time.perf_counter()
            timings[which]["integrate"].append((integrated - started) * 1000)
            timings[which]["snapshot"].append((finished - integrated) * 1000)
            timings[which]["combined"].append((finished - started) * 1000)
        # Exact means, changed-key order and output equivalence are outside the
        # timed region. A speed result is invalid if the geometry differs.
        if implementations[0].points != implementations[1].points or changed[0] != changed[1]:
            raise AssertionError(f"Baseline geometry/changed keys differ in frame {index}")
        np.testing.assert_array_equal(arrays[0], arrays[1])

    unchanged = implementations[1].snapshot()
    started = time.perf_counter()
    for _ in range(1000):
        if implementations[1].snapshot() is not unchanged:
            raise AssertionError("Unchanged snapshot was not cached")
    cached_ms = (time.perf_counter() - started) * 1000 / 1000
    result = {
        "initial_input_points": size, "confirmed_map_points": len(implementations[1].points),
        "scan_input_points": point_count, "rounds": rounds,
        "baseline": {name: summary(samples) for name, samples in timings[0].items()},
        "optimized": {name: summary(samples) for name, samples in timings[1].items()},
        "unchanged_snapshot_mean_ms": cached_ms,
        "geometry_and_changed_key_equivalence": "exact for every measured frame",
        "float32_snapshot_equivalence": "exact for every measured frame",
    }
    result["combined_median_speedup"] = (result["baseline"]["combined"]["median_ms"] /
                                            result["optimized"]["combined"]["median_ms"])
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sizes", type=int, nargs="+", default=[18000, 500000])
    parser.add_argument("--scan-points", type=int, default=18000)
    parser.add_argument("--rounds", type=int, default=20)
    parser.add_argument("--seed", type=int, default=728)
    parser.add_argument("--baseline", type=Path, default=DIAGNOSTICS / "baseline_voxel_fusion.py")
    parser.add_argument("--output", type=Path, default=DIAGNOSTICS / "voxel_efficiency_benchmark.json")
    args = parser.parse_args()
    if min(*args.sizes, args.scan_points, args.rounds) < 1:
        parser.error("sizes, scan-points and rounds must be positive")
    baseline = baseline_type(args.baseline)
    rng = np.random.default_rng(args.seed)
    report = {
        "experiment": "confirmed_voxel_fusion_and_float32_output",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "field_measurements": False,
        "environment": {"platform": platform.platform(), "python": sys.version,
                        "numpy": np.__version__, "processor": platform.processor(),
                        "logical_cpu_count": os.cpu_count()},
        "sources": {"baseline": str(args.baseline.resolve()),
                    "baseline_sha256": source_hash(args.baseline),
                    "optimized": str(MAPPING / "voxel_fusion.py"),
                    "optimized_sha256": source_hash(MAPPING / "voxel_fusion.py"),
                    "benchmark_sha256": source_hash(Path(__file__))},
        "parameters": {"seed": args.seed, "sizes": args.sizes, "rounds": args.rounds,
                       "scan_input_points": args.scan_points, "voxel_m": 0.05,
                       "min_observations": 1, "max_weight": 20,
                       "jitter_m": [0, 0.035], "alternating_measurement_order": True},
        "limitations": "Synthetic local CPU timings; excludes ROS, point filters, RViz and disk I/O. "
                       "Does not verify target Ubuntu performance or faster-motion map quality.",
        "cases": [],
    }
    for size in args.sizes:
        case = measure_case(size, args.scan_points, args.rounds, rng, baseline)
        report["cases"].append(case)
        print(json.dumps({"map_points": case["confirmed_map_points"],
                          "baseline_combined_median_ms": case["baseline"]["combined"]["median_ms"],
                          "optimized_combined_median_ms": case["optimized"]["combined"]["median_ms"],
                          "speedup": case["combined_median_speedup"]}), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Report: {args.output.resolve()}")


if __name__ == "__main__":
    main()
