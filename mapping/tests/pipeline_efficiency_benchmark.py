"""Compare the complete added mapper pipeline using real binary fake-ROS scans.

Run with Python, NumPy and SciPy from any directory. Baseline and current code
run in separate processes so their maps and GC policy cannot affect each other.
The default 180 scans represent 18 seconds at 10 Hz and include 1-second map
publication, 5-second diagnostics and 15-second PCD autosave. No ROS transport,
RViz rendering, live scheduling or target Ubuntu performance is simulated.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gc
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest import mock

# Match the added shell launcher before importing numerical libraries.
for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_name] = os.environ.get("MAPPING_BLAS_THREADS", "1")

import numpy as np
import scipy


MAPPING = Path(__file__).resolve().parents[1]
DIAGNOSTICS = MAPPING / "diagnostics" / "efficiency_20260909"
sys.path[:0] = [str(MAPPING), str(MAPPING / "tests")]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(values) -> dict:
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {"samples": 0}
    return {"samples": len(array), "median_ms": float(np.median(array)),
            "p95_ms": float(np.percentile(array, 95)), "max_ms": float(array.max()),
            "min_ms": float(array.min()), "total_ms": float(array.sum())}


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_mapper(which: str, baseline: Path):
    import runtime_gc  # Keep it outside fake ROS's temporary module registry.
    import test_field_mapper as fake_ros

    if which == "current":
        return fake_ros.load_mapper(), fake_ros
    replacements = {
        name: load_module(f"pipeline_baseline_{name}", baseline / f"baseline_{name}.py")
        for name in ("voxel_fusion", "point_filters", "ground_surface", "pose_buffer")
    }
    with mock.patch.dict(sys.modules, replacements), mock.patch.object(
            fake_ros, "SOURCE", baseline / "baseline_field_mapper_node.py"):
        return fake_ros.load_mapper(), fake_ros


def synthetic_scene(map_points: int, scan_points: int, seed: int):
    """Build measured wall/floor samples in unique 3 cm voxels, never fill a map."""
    rng = np.random.default_rng(seed)
    # Four 100k-point walls plus a 100k-point floor provide the default 500k
    # old map. All coordinates are voxel-interior measured synthetic samples.
    horizontal, vertical = np.meshgrid(np.arange(500), np.arange(200), indexing="ij")
    along = (horizontal.ravel() - 250 + 0.37) * 0.03
    height = (vertical.ravel() + 0.41) * 0.03
    walls = [np.column_stack((np.full(len(along), side * 15.013), along, height))
             for side in (-1, 1)]
    walls += [np.column_stack((along, np.full(len(along), side * 10.013), height))
              for side in (-1, 1)]
    floor = np.column_stack((along, (vertical.ravel() - 100 + 0.43) * 0.03,
                             np.full(len(along), 0.006)))
    initial = np.vstack((*walls, floor))
    if not 20000 <= map_points <= len(initial):
        raise ValueError("map-points must be between 20000 and 500000")
    initial = initial[:map_points].copy()
    ground_count = min(100, max(36, scan_points // 180))
    wall_count = scan_points - ground_count
    wall_indices = np.linspace(0, min(100000, len(initial)) - 1, wall_count, dtype=int)
    wall_scan = initial[wall_indices].copy()
    side = int(np.ceil(np.sqrt(ground_count)))
    gx, gy = np.meshgrid(np.arange(side), np.arange(side), indexing="ij")
    ground_scan = np.column_stack((2.01 + gx.ravel()[:ground_count] * 0.12,
                                  1.01 + gy.ravel()[:ground_count] * 0.12,
                                  np.full(ground_count, 0.006)))
    template = np.vstack((wall_scan, ground_scan))
    # Precompute repeatable input noise outside each timed frame; every scan
    # revisits real samples with submillimetre changes so averaging is exercised.
    return initial, template, rng


class DiscardPublisher:
    """Exercise message construction without retaining all large output frames."""

    def __init__(self):
        self.count = 0
        self.last_point_count = 0

    def publish(self, message):
        self.count += 1
        self.last_point_count = getattr(message, "width", 0)


def cloud_message(fake_ros, world: np.ndarray, seconds: float, sensor_z: float):
    # Match the actual logged 26-byte sensor layout, including unused fields.
    dtype = np.dtype({"names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
                      "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
                      "offsets": [0, 4, 8, 12, 16, 17, 18], "itemsize": 26})
    records = np.zeros(len(world), dtype=dtype)
    records["x"], records["y"], records["z"] = world[:, 0], world[:, 1], world[:, 2] - sensor_z
    records["timestamp"] = seconds
    message = fake_ros.PointCloud2(seconds=seconds)
    message.fields = [fake_ros.PointField(name, offset, datatype, 1) for name, offset, datatype in
                      (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7),
                       ("tag", 16, 2), ("line", 17, 2), ("timestamp", 18, 8))]
    message.width, message.height = len(records), 1
    message.point_step, message.row_step = 26, 26 * len(records)
    message.data = records.tobytes()
    return message


def initialize_map(node, initial: np.ndarray) -> None:
    # Preload a previously confirmed map without running radius/ground fitting
    # on an artificial 500k-point single scan. Preserve the default 3-scan gate.
    for stamp in (1.0, 1.1, 1.2):
        node.fusion.integrate(initial, stamp)
    changed = list(node.voxels)
    if hasattr(node, "update_display"):
        node.update_display(changed)
    else:
        # Same fixed-owner initialization as the baseline per-scan loop. This
        # is setup only; all measured updates execute the baseline node itself.
        centres = (np.asarray(changed, dtype=np.int64) + 0.5) * node.voxel_size
        surfaces = np.floor(centres / node.cell_size).astype(np.int64).tolist()
        displays = np.floor(centres / node.display_voxel).astype(np.int64).tolist()
        for key, surface, display in zip(changed, surfaces, displays):
            sample = node.voxels[key]
            surface_key, display_key = tuple(surface), tuple(display)
            if node.surface_owners.setdefault(surface_key, key) == key:
                node.surface_samples[surface_key] = sample
            cell = node.cells.setdefault(surface_key[:2], {"min_z": sample[2], "max_z": sample[2]})
            cell["min_z"], cell["max_z"] = min(cell["min_z"], sample[2]), max(cell["max_z"], sample[2])
            if node._display_owners.setdefault(display_key, key) == key:
                node._display_points[display_key] = sample
    node.dirty = True
    node.display_dirty = True
    node.publish_map()
    ok, reason = node.save_pcd()
    if not ok:
        raise RuntimeError(reason)


def sorted_dictionary(dictionary, dimensions=3):
    keys = np.asarray(list(dictionary), dtype=np.int64).reshape(-1, dimensions)
    values = np.asarray(list(dictionary.values()), dtype=np.float64).reshape(-1, 3)
    order = np.lexsort(tuple(keys[:, axis] for axis in range(dimensions - 1, -1, -1)))
    return keys[order], values[order]


def export_geometry(node, target: Path):
    arrays = {}
    for name, dictionary in (("fine", node.voxels), ("ground", node.ground_fusion.points),
                             ("display", node._display_points), ("surface", node.surface_samples)):
        arrays[name + "_keys"], arrays[name + "_xyz"] = sorted_dictionary(dictionary)
    cell_keys = sorted(node.cells)
    arrays["cell_keys"] = np.asarray(cell_keys, dtype=np.int64).reshape(-1, 2)
    arrays["cell_bounds"] = np.asarray([[node.cells[key]["min_z"], node.cells[key]["max_z"]]
                                       for key in cell_keys], dtype=np.float64).reshape(-1, 2)
    arrays["display_owner_keys"], arrays["display_owners"] = sorted_dictionary(node._display_owners)
    arrays["surface_owner_keys"], arrays["surface_owners"] = sorted_dictionary(node.surface_owners)
    np.savez(target, **arrays)


def worker(args) -> None:
    mapper, fake_ros = load_mapper(args.worker, args.baseline_dir)
    initial, template, rng = synthetic_scene(args.map_points, args.scan_points, args.seed)
    virtual_wall = [1000.0]
    # Autosave uses virtual sensor elapsed time; all processing and GC pause
    # durations continue to use the real monotonic/performance clock.
    mapper.time = SimpleNamespace(time=lambda: virtual_wall[0], monotonic=time.monotonic,
                                  perf_counter=time.perf_counter)
    options = mapper.build_parser().parse_args(["--pcd-file", str(args.artifact_dir / "map.pcd")])
    node = mapper.FieldMapperNode(options)
    publishers = {}
    for name in ("map", "ground", "current", "passable", "structure"):
        publishers[name] = DiscardPublisher()
        setattr(node, name + "_publisher", publishers[name])
    initialize_map(node, initial)
    del initial
    # A stationary 500 Hz pose stream brackets every scan with the unchanged
    # default motion window. This checks the regular callback/lookup path.
    odom_tick = -60
    while odom_tick <= 55:
        node.odom_callback(fake_ros.Odometry(100.0 + odom_tick * 0.002))
        odom_tick += 1
    gc.collect(2)
    gc.enable()
    control = mapper.ScheduledGC() if args.worker == "current" else None
    if control is not None:
        node.gc_maintenance = control
        control_module = sys.modules[type(control).__module__]
        virtual_gc = [control.started]
        control_module.time = SimpleNamespace(monotonic=lambda: virtual_gc[0], perf_counter=time.perf_counter)
    else:
        virtual_gc = [0.0]
    gc_started = {}
    gc_pauses = []
    active_phase = ["input_preparation"]

    def record_gc(phase, info):
        generation = info["generation"]
        if phase == "start":
            gc_started[generation] = (time.perf_counter(), active_phase[0])
        elif generation in gc_started:
            started, label = gc_started.pop(generation)
            gc_pauses.append({"generation": generation, "phase": label,
                              "duration_ms": (time.perf_counter() - started) * 1000,
                              "collected": info.get("collected", 0)})

    gc.callbacks.append(record_gc)
    rows = []
    for index in range(args.frames):
        elapsed = (index + 1) * 0.1
        seconds = 100.0 + index * 0.1
        world = template + rng.uniform(-0.0002, 0.0002, template.shape)
        message = cloud_message(fake_ros, world, seconds, options.sensor_z)
        virtual_wall[0] = 1000.0 + elapsed
        row = {"frame": index, "elapsed_sensor_s": elapsed}
        active_phase[0] = "intake"
        start = time.perf_counter()
        while odom_tick * 0.002 <= index * 0.1 + 0.1100001:
            node.odom_callback(fake_ros.Odometry(100.0 + odom_tick * 0.002))
            odom_tick += 1
        node.cloud_callback(message)
        intake_end = time.perf_counter()
        active_phase[0] = "processing"
        before_frames = node.frames
        node.process_pending()
        processing_end = time.perf_counter()
        if node.frames != before_frames + 1:
            raise AssertionError(f"Frame {index} was not fused: {dict(node.rejection_reasons)} / {node.last_reason}")
        row.update({"intake_ms": (intake_end - start) * 1000,
                    "processing_ms": (processing_end - intake_end) * 1000,
                    "stages_ms": dict(node.last_stage_ms)})
        for label, due, callback in (
                ("publish", (index + 1) % 10 == 0, node.publish_timer),
                ("status", (index + 1) % 50 == 0, node.report_status),
                ("autosave", (index + 1) % 10 == 0, node.autosave_timer)):
            active_phase[0] = label
            tick_start = time.perf_counter()
            if due:
                callback()
            row[label + "_ms"] = (time.perf_counter() - tick_start) * 1000 if due else 0.0
        active_phase[0] = "scheduled_gc"
        tick_start = time.perf_counter()
        if control is not None and (index + 1) % 10 == 0:
            virtual_gc[0] = control.started + elapsed
            control.tick()
        row["scheduled_gc_ms"] = (time.perf_counter() - tick_start) * 1000 if control is not None else 0.0
        row["total_ms"] = (time.perf_counter() - start) * 1000
        rows.append(row)
        active_phase[0] = "input_preparation"
    active_phase[0] = "final_save"
    started = time.perf_counter()
    ok, reason = node.save_pcd()
    final_save_ms = (time.perf_counter() - started) * 1000
    if not ok:
        raise RuntimeError(reason)
    gc.callbacks.remove(record_gc)
    if control is not None:
        control.close()
    export_geometry(node, args.artifact_dir / "geometry.npz")
    stage_names = sorted(set(k for row in rows for k in row["stages_ms"]))
    report = {
        "implementation": args.worker, "parameters": vars(options),
        "input_frames": args.frames, "processed_frames": node.frames,
        "final_pcd_points": node.map_point_count(), "fine_points": len(node.voxels),
        "ground_points": len(node.ground_fusion.points), "display_points": len(node._display_points),
        "pending_voxels": node.fusion.pending_count,
        "timings": {key: summary([row[key] for row in rows]) for key in
                    ("intake_ms", "processing_ms", "publish_ms", "status_ms", "autosave_ms", "scheduled_gc_ms", "total_ms")},
        "stages": {key: summary([row["stages_ms"][key] for row in rows if key in row["stages_ms"]])
                   for key in stage_names},
        "final_save_ms": final_save_ms, "gc_pauses": gc_pauses,
        "gc_pause_summary": summary([event["duration_ms"] for event in gc_pauses]),
        "gc_pauses_by_generation": {str(g): summary([event["duration_ms"] for event in gc_pauses
                                                      if event["generation"] == g]) for g in (0, 1, 2)},
        "output_publications_including_initial": {name: publisher.count for name, publisher in publishers.items()},
        "save_log_count_including_initial_and_final": sum("saved " in message for _, message in node.logger.entries) + 2,
        "frames": rows,
    }
    (args.artifact_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def compare_geometry(baseline: Path, current: Path) -> dict:
    result = {}
    with np.load(baseline / "geometry.npz") as old, np.load(current / "geometry.npz") as new:
        if set(old.files) != set(new.files):
            raise AssertionError("Geometry exports have different fields")
        for name in old.files:
            first, second = old[name], new[name]
            if first.shape != second.shape:
                raise AssertionError(f"{name}: shapes differ: {first.shape}, {second.shape}")
            exact = bool(np.array_equal(first, second))
            if name.endswith("keys") or name.endswith("owners"):
                if not exact:
                    raise AssertionError(f"{name}: map/bucket ownership differs")
            elif not np.allclose(first, second, rtol=0, atol=1e-9):
                raise AssertionError(f"{name}: geometry differs beyond 1e-9 m")
            result[name] = {"shape": list(first.shape), "exact": exact,
                            "max_absolute_difference": float(np.max(np.abs(first - second))) if first.size else 0.0}
    arrays = []
    for directory in (baseline, current):
        payload = (directory / "map.pcd").read_bytes().split(b"DATA binary\n", 1)[1]
        xyz = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
        order = np.lexsort((xyz[:, 2], xyz[:, 1], xyz[:, 0]))
        arrays.append(xyz[order])
    if arrays[0].shape != arrays[1].shape or not np.allclose(*arrays, rtol=0, atol=2e-6):
        raise AssertionError("Final float32 PCD differs beyond 2e-6 m")
    result["pcd"] = {"points": len(arrays[0]), "exact_after_coordinate_sort": bool(np.array_equal(*arrays)),
                     "max_absolute_difference_m": float(np.max(np.abs(arrays[0] - arrays[1]))),
                     "absolute_tolerance_m": 2e-6}
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=180)
    parser.add_argument("--map-points", type=int, default=500000)
    parser.add_argument("--scan-points", type=int, default=18000)
    parser.add_argument("--seed", type=int, default=909)
    parser.add_argument("--baseline-dir", type=Path, default=DIAGNOSTICS)
    parser.add_argument("--output", type=Path, default=DIAGNOSTICS / "pipeline_efficiency_benchmark.json")
    parser.add_argument("--worker", choices=("baseline", "current"), help=argparse.SUPPRESS)
    parser.add_argument("--artifact-dir", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.frames < 30 or not 1000 <= args.scan_points <= 50000:
        parser.error("Use at least 30 frames and 1000..50000 points per scan")
    if args.worker:
        worker(args)
        return
    source_paths = [Path(__file__), MAPPING / "tests" / "test_field_mapper.py"]
    source_paths += [MAPPING / f"{name}.py" for name in
                     ("field_mapper_node", "voxel_fusion", "point_filters", "ground_surface", "pose_buffer", "cloud_io", "runtime_gc")]
    source_paths += [args.baseline_dir / f"baseline_{name}.py" for name in
                     ("field_mapper_node", "voxel_fusion", "point_filters", "ground_surface", "pose_buffer")]
    sources = {str(path.resolve()): sha256(path) for path in source_paths}
    report = {
        "experiment": "binary_scan_full_mapper_pipeline_with_publishing_saving_and_cyclic_gc",
        "created_utc": datetime.now(timezone.utc).isoformat(), "field_measurements": False,
        "sources_sha256": sources,
        "environment": {"platform": platform.platform(), "python": sys.version, "numpy": np.__version__,
                        "scipy": scipy.__version__, "logical_cpu_count": os.cpu_count(),
                        "blas_threads": os.environ["OPENBLAS_NUM_THREADS"]},
        "parameters": {"seed": args.seed, "initial_map_points": args.map_points, "scan_points": args.scan_points,
                       "frames": args.frames, "sensor_hz": 10, "sensor_elapsed_seconds": args.frames * 0.1,
                       "publish_period_s": 1, "status_period_s": 5, "autosave_period_s": 15,
                       "gc_policy": {"baseline": "automatic", "current": "ScheduledGC.tick on virtual sensor time"},
                       "pose": "stationary, 500 Hz callback input", "input_layout": "26-byte binary PointCloud2 with XYZ/intensity/tag/line/timestamp"},
        "limitations": [
            "Synthetic local CPU/disk experiment; does not measure ROS serialization/transport or RViz rendering.",
            "Elapsed sensor time is virtual; processing runs as fast as possible, and input queue scheduling/drop behavior is not simulated.",
            "Map initialization, scan synthesis, and geometry validation are excluded; intake, filtering, fusion, output construction, periodic publish/save/status and scheduled GC are included.",
            "Publishers discard messages immediately; fake ROS message objects differ from generated ROS classes.",
            "The default 18 simulated seconds exercise 1-second and 15-second GC but not the 120-second full-generation maintenance. Use --frames 1200 to include it.",
            "No motion deskew or driving-speed accuracy conclusion can be drawn from stationary synthetic scans.",
            "Baseline runs before current in independent processes; source hashes must remain stable throughout the experiment.",
        ],
    }
    with tempfile.TemporaryDirectory(prefix="mapping_pipeline_benchmark_") as temporary:
        directories = {}
        for which in ("baseline", "current"):
            directory = Path(temporary) / which
            directory.mkdir()
            directories[which] = directory
            command = [sys.executable, "-B", str(Path(__file__).resolve()), "--worker", which,
                       "--artifact-dir", str(directory), "--baseline-dir", str(args.baseline_dir.resolve()),
                       "--frames", str(args.frames), "--map-points", str(args.map_points),
                       "--scan-points", str(args.scan_points), "--seed", str(args.seed)]
            print(f"Running {which}: {args.frames} x {args.scan_points} scans over {args.map_points} map points", flush=True)
            completed = subprocess.run(command, capture_output=True, text=True)
            if completed.returncode:
                raise RuntimeError(f"{which} worker failed:\n{completed.stdout}\n{completed.stderr}")
            report[which] = json.loads((directory / "report.json").read_text(encoding="utf-8"))
            print(json.dumps({"implementation": which, "processing": report[which]["timings"]["processing_ms"],
                              "total": report[which]["timings"]["total_ms"]}), flush=True)
        report["geometry_comparison"] = compare_geometry(directories["baseline"], directories["current"])
    if any(sha256(Path(path)) != digest for path, digest in sources.items()):
        raise RuntimeError("Source files changed during benchmark; rerun with stable files")
    report["total_median_speedup"] = (report["baseline"]["timings"]["total_ms"]["median_ms"] /
                                      report["current"]["timings"]["total_ms"]["median_ms"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Geometry verified. Report: {args.output.resolve()}", flush=True)


if __name__ == "__main__":
    main()
