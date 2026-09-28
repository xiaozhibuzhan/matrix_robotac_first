#!/usr/bin/env python3
"""Capture scans with the official RViz view, or run the online mapper."""

from __future__ import annotations

import argparse
import codecs
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Sequence, TextIO

try:
    import field_mapper_node
except ImportError as exc:
    raise SystemExit(
        f"Mapping dependency unavailable: {exc}. On Ubuntu install python3-numpy and python3-scipy "
        "and synchronize mapping/point_filters.py, mapping/pose_buffer.py, mapping/voxel_fusion.py "
        "mapping/ground_surface.py, mapping/cloud_io.py and mapping/runtime_gc.py, "
        "then source the ROS environment."
    ) from exc


PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUNTIME_FILES = (
    "mapping/field_mapper_node.py", "mapping/cloud_io.py", "mapping/point_filters.py",
    "mapping/pose_buffer.py", "mapping/voxel_fusion.py", "mapping/ground_surface.py",
    "mapping/runtime_gc.py", "mapping/deskew.py", "mapping/embedded_pose.py", "mapping/occupancy_map.py",
    "mapping/capture_mapping.py", "mapping/finalize_mapping.py", "mapping/finalize_pcd.sh",
    "mapping/run_mapping_rviz.py", "mapping/run_mapping_rviz.sh",
    "rviz/matrix.rviz",
)
PERFORMANCE_ENVIRONMENT_KEYS = ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")


def build_parser() -> argparse.ArgumentParser:
    parser = field_mapper_node.build_parser()
    parser.description = "Capture scans with official RViz; generate the final PCD after capture ends"
    parser.allow_abbrev = False
    parser.add_argument("--mode", choices=("capture", "online"), default="capture",
                        help="capture raw scans/poses for final PCD generation (default), or use the online mapper")
    parser.set_defaults(pcd_file=None, live_display="none", save_policy="final")
    parser.epilog = "Without --pcd-file, create a new maps/run_<timestamp>_<unique> directory."
    return parser


def preflight(project_root: Path, mode: str = "capture") -> str:
    if mode == "online" and field_mapper_node.ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS2 Python dependencies are unavailable: "
            f"{field_mapper_node.ROS_IMPORT_ERROR}. Source the ROS2 environment in this terminal first."
        )
    rviz = shutil.which("rviz2")
    if rviz is None:
        raise RuntimeError("rviz2 is not on PATH. Source the ROS2 environment and install RViz2 first.")
    if mode == "capture" and shutil.which("ros2") is None:
        raise RuntimeError("ros2 is not on PATH. Source the ROS2 environment and install rosbag2 first.")
    backend = "mapping/capture_mapping.py" if mode == "capture" else "mapping/field_mapper_node.py"
    for relative_path in (backend, "rviz/matrix.rviz"):
        if not (project_root / relative_path).is_file():
            raise RuntimeError(f"Required file is missing: {project_root / relative_path}")
    return rviz


def output_path(project_root: Path, requested: str | None) -> Path:
    if requested is not None:
        path = Path(requested).expanduser()
        return (path if path.is_absolute() else project_root / path).resolve()
    maps_dir = project_root / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    prefix = f"run_{datetime.now():%Y%m%d_%H%M%S}_"
    run_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=maps_dir))
    return run_dir / "field_map.pcd"


def child_commands(
    project_root: Path,
    rviz: str,
    mapper_argv: Sequence[str],
    pcd_path: Path,
    global_frame: str,
    rviz_config: Path | None = None,
    *,
    mode: str = "capture",
    bag_path: Path | None = None,
) -> tuple[list[str], list[str]]:
    backend_argv = _mapper_arguments(mapper_argv)
    if mode == "capture":
        args = build_parser().parse_args(backend_argv)
        mapper = [sys.executable, "-u", str(project_root / "mapping/capture_mapping.py"),
                  "--bag-dir", str(bag_path or (pcd_path.parent / "raw_bag")),
                  "--topic", args.topic, "--odom-topic", args.odom_topic]
    else:
        mapper = [sys.executable, "-u", str(project_root / "mapping/field_mapper_node.py")]
        # Keep mapping options intact; the last PCD option supplies the resolved path.
        mapper.extend(backend_argv)
        if not _has_option(backend_argv, "--live-display"):
            mapper.extend(["--live-display", "none"])
        if not _has_option(backend_argv, "--save-policy"):
            mapper.extend(["--save-policy", "final"])
        mapper.extend(["--pcd-file", str(pcd_path)])
    config = rviz_config or (project_root / "rviz/matrix.rviz")
    fixed_frame = "lidar" if config.name == "matrix.rviz" else global_frame
    view = [rviz, "-d", str(config), "-f", fixed_frame]
    return mapper, view


def _has_option(argv: Sequence[str], option: str) -> bool:
    return any(argument == option or argument.startswith(option + "=") for argument in argv)


def _mapper_arguments(argv: Sequence[str]) -> list[str]:
    """Remove launcher-only options while preserving both argparse value forms."""
    arguments = []
    skip_value = False
    for argument in argv:
        if skip_value:
            skip_value = False
        elif argument == "--mode":
            skip_value = True
        elif not argument.startswith("--mode="):
            arguments.append(argument)
    return arguments


def _runtime_manifest(project_root: Path) -> dict[str, Any]:
    """Return a small, deterministic manifest of files and runtime knobs."""
    files: dict[str, str | None] = {}
    for relative in RUNTIME_FILES:
        path = project_root / relative
        if not path.is_file():
            files[relative] = None
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        files[relative] = digest
    return {
        "files_sha256": files,
        "python": sys.version,
        "performance_environment": {
            key: os.environ.get(key) for key in PERFORMANCE_ENVIRONMENT_KEYS
        },
    }


def _snapshot_rviz(project_root: Path, directory: Path) -> Path:
    source = project_root / "rviz/matrix.rviz"
    if not source.is_file():
        raise FileNotFoundError(f"Required file is missing: {source}")
    target = directory / "matrix.rviz"
    shutil.copyfile(source, target)
    return target


def prepare_logs(pcd_path: Path, explicit_pcd: bool, mapper_command: Sequence[str],
                 rviz_command: Sequence[str], project_root: Path | None = None) -> Path:
    pcd_path.parent.mkdir(parents=True, exist_ok=True)
    directory = pcd_path.parent
    if explicit_pcd:
        prefix = f"logs_run_{datetime.now():%Y%m%d_%H%M%S}_"
        directory = Path(tempfile.mkdtemp(prefix=prefix, dir=pcd_path.parent))
    metadata = {
        "started_at": datetime.now(timezone.utc).isoformat(),
        "pcd_path": str(pcd_path),
        "mapper_command": list(mapper_command),
        "rviz_command": list(rviz_command),
        "environment": {key: os.environ.get(key) for key in
                        ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "SDK_CLIENT_IP")},
    }
    if project_root is not None:
        metadata.update(_runtime_manifest(project_root))
    with (directory / "run.json").open("x", encoding="utf-8") as output:
        json.dump(metadata, output, indent=2, ensure_ascii=True)
        output.write("\n")
    return directory


def _update_run_metadata(directory: Path, updates: dict[str, Any]) -> None:
    path = directory / "run.json"
    metadata = json.loads(path.read_text(encoding="utf-8"))
    metadata.update(updates)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(metadata, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    temporary.replace(path)


class OutputTee:
    """Drain one child pipe without letting inherited pipes block shutdown."""

    def __init__(self, source: Any, log: TextIO, console: TextIO, name: str) -> None:
        self.source = source
        self.log = log
        self.console = console
        self.name = name
        self.stopped = threading.Event()
        self.lock = threading.Lock()
        self.error: str | None = None
        self.thread = threading.Thread(target=self.read_output, name=f"mapping-log-{name}", daemon=True)
        self.thread.start()

    def emit(self, text: str) -> None:
        with self.lock:
            if self.stopped.is_set():
                return
            self.log.write(text)
            self.log.flush()
        # The terminal may block independently of disk capture. Do not hold
        # the log lock while writing it, so shutdown can still close the log.
        if not self.stopped.is_set():
            try:
                self.console.write(text)
                self.console.flush()
            except (OSError, ValueError):
                # A closed terminal must not stop persistent log capture.
                pass

    def read_output(self) -> None:
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        read = getattr(self.source, "read1", self.source.read)
        try:
            while not self.stopped.is_set():
                chunk = read(4096)
                if not chunk:
                    self.emit(decoder.decode(b"", final=True))
                    break
                self.emit(decoder.decode(chunk) if isinstance(chunk, bytes) else chunk)
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        finally:
            try:
                self.source.close()
            finally:
                with self.lock:
                    self.log.close()

    def finish(self, timeout: float) -> None:
        deadline = time.monotonic() + max(0.0, timeout)
        self.thread.join(timeout=max(0.0, timeout))
        # Do not close a pipe from another thread while read() holds its lock.
        # A grandchild may keep it open after our direct child has exited.
        self.stopped.set()
        if self.lock.acquire(timeout=max(0.0, deadline - time.monotonic())):
            try:
                self.log.close()
            finally:
                self.lock.release()
        else:
            print(f"Stopped waiting for {self.name} log write; its capture thread will close the file.",
                  file=sys.stderr, flush=True)
        if self.thread.is_alive():
            print(f"Stopped waiting for {self.name} log pipe; another process may still hold it open.",
                  file=sys.stderr, flush=True)
        if self.error is not None:
            print(f"Could not capture all {self.name} output: {self.error}", file=sys.stderr, flush=True)


def stop_child(child: subprocess.Popen, name: str, save_timeout: float = 15.0) -> int:
    status = child.poll()
    if status is not None:
        return status
    print(f"Stopping {name} with SIGINT...", flush=True)
    try:
        child.send_signal(signal.SIGINT)
    except ProcessLookupError:
        return child.wait()
    try:
        return child.wait(timeout=save_timeout)
    except subprocess.TimeoutExpired:
        print(f"{name} did not exit after {save_timeout:g}s; terminating it.", file=sys.stderr, flush=True)
    child.terminate()
    try:
        child.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        print(f"{name} did not terminate; killing this child process.", file=sys.stderr, flush=True)
        child.kill()
        child.wait()
    # Escalation can interrupt rosbag's SQLite/metadata flush even if the final
    # process exit happens to be zero; it must not be reported as a clean stop.
    return 1


def supervise(
    mapper_command: Sequence[str],
    rviz_command: Sequence[str],
    project_root: Path,
    log_directory: Path | None = None,
    save_timeout: float = 15.0,
) -> int:
    mapper = None
    rviz = None
    stop_signal = 0
    logs: dict[str, TextIO] = {}
    captures: list[OutputTee] = []
    result = 1
    mapper_exit_status = None

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal stop_signal
        stop_signal = signum

    previous_handlers = {
        sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        # Separate POSIX sessions prevent a second Ctrl+C while the worker
        # flushes its bag or PCD. Only this supervisor sends child signals.
        child_options = {"cwd": str(project_root), "start_new_session": os.name == "posix"}
        if log_directory is not None:
            for name in ("mapper", "rviz"):
                logs[name] = (log_directory / f"{name}.log").open("x", encoding="utf-8")
            child_options.update(stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        mapper = subprocess.Popen(mapper_command, **child_options)
        if log_directory is not None:
            captures.append(OutputTee(mapper.stdout, logs["mapper"], sys.stdout, "mapper"))
        initial_status = mapper.poll()
        if initial_status is not None and not stop_signal:
            print(f"Mapping worker exited during startup (code {initial_status}).", file=sys.stderr)
            result = initial_status if initial_status > 0 else 1
        elif not stop_signal:
            rviz = subprocess.Popen(rviz_command, **child_options)
            if log_directory is not None:
                captures.append(OutputTee(rviz.stdout, logs["rviz"], sys.stdout, "RViz"))
            while not stop_signal:
                mapper_status = mapper.poll()
                if mapper_status is not None:
                    print(f"Mapping worker exited (code {mapper_status}); stopping RViz.", file=sys.stderr)
                    result = mapper_status if mapper_status > 0 else 1
                    break
                rviz_status = rviz.poll()
                if rviz_status is not None:
                    print(f"RViz exited (code {rviz_status}); flushing and stopping the mapping worker.", flush=True)
                    result = rviz_status if rviz_status >= 0 else 128 - rviz_status
                    break
                time.sleep(0.1)
        if stop_signal:
            result = 128 + stop_signal
    except OSError as exc:
        print(f"Could not start mapping/RViz: {exc}", file=sys.stderr, flush=True)
        result = 1
    finally:
        try:
            if mapper is not None:
                mapper_exit_status = stop_child(mapper, "mapping worker (flush pending data)", save_timeout)
        finally:
            try:
                if rviz is not None:
                    stop_child(rviz, "RViz", save_timeout=3.0)
            finally:
                try:
                    deadline = time.monotonic() + 2.0
                    for capture in captures:
                        capture.finish(deadline - time.monotonic())
                finally:
                    captured_logs = {id(capture.log) for capture in captures}
                    for output in logs.values():
                        if id(output) not in captured_logs:
                            output.close()
                    for sig, handler in previous_handlers.items():
                        signal.signal(sig, handler)
    if isinstance(mapper_exit_status, int) and mapper_exit_status != 0 and result in (0, 130, 143):
        print(f"Mapping worker did not stop cleanly (code {mapper_exit_status}); check mapper.log.",
              file=sys.stderr, flush=True)
        return mapper_exit_status if mapper_exit_status > 0 else 1
    return result


def main(argv: Sequence[str] | None = None) -> int:
    mapper_argv = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(mapper_argv)
    try:
        rviz = preflight(PROJECT_ROOT, args.mode)
        pcd_path = output_path(PROJECT_ROOT, args.pcd_file)
    except (RuntimeError, OSError) as exc:
        print(f"Cannot start mapping/RViz: {exc}", file=sys.stderr, flush=True)
        return 1
    print(f"Final PCD output: {pcd_path}", flush=True)
    if pcd_path.exists():
        print("WARNING: the explicitly selected PCD path exists and will be overwritten.", file=sys.stderr)
    print("RViz fixed frame: lidar (official current scan). Close RViz or press Ctrl+C to stop cleanly.", flush=True)
    mapper, view = child_commands(PROJECT_ROOT, rviz, mapper_argv, pcd_path, args.global_frame, mode=args.mode)
    try:
        log_directory = prepare_logs(
            pcd_path, args.pcd_file is not None, mapper, view, project_root=PROJECT_ROOT
        )
        rviz_config = _snapshot_rviz(PROJECT_ROOT, log_directory)
        bag_path = log_directory / "raw_bag"
        mapper, view = child_commands(
            PROJECT_ROOT, rviz, mapper_argv, pcd_path, args.global_frame, rviz_config,
            mode=args.mode, bag_path=bag_path,
        )
        mapping_args = dict(vars(args), pcd_file=str(pcd_path))
        finalize_command = ["bash", "mapping/finalize_pcd.sh", str(log_directory)]
        _update_run_metadata(log_directory, {
            "mapper_command": list(mapper),
            "rviz_command": list(view),
            "rviz_config": str(rviz_config),
            "rviz_config_sha256": hashlib.sha256(rviz_config.read_bytes()).hexdigest(),
            "mode": args.mode,
            "bag_path": str(bag_path) if args.mode == "capture" else None,
            "mapping_args": mapping_args,
            "finalize_command": finalize_command if args.mode == "capture" else None,
            "capture_complete": False if args.mode == "capture" else None,
        })
    except OSError as exc:
        print(f"Cannot create run logs: {exc}", file=sys.stderr, flush=True)
        return 1
    print(f"Mapper log: {log_directory / 'mapper.log'}", flush=True)
    print(f"RViz log: {log_directory / 'rviz.log'}", flush=True)
    print(f"Run metadata: {log_directory / 'run.json'}", flush=True)
    if args.mode == "capture":
        import shlex
        print(f"Raw capture: {bag_path}", flush=True)
        print("After capture stops, generate PCD: " + shlex.join(finalize_command), flush=True)
    else:
        print("Online mode: save the PCD explicitly or on normal shutdown.", flush=True)
    result = supervise(mapper, view, PROJECT_ROOT, log_directory=log_directory,
                       save_timeout=60.0 if args.mode == "capture" else 15.0)
    if args.mode == "capture":
        complete = result in (0, 130, 143) and (bag_path / "metadata.yaml").is_file()
        if not complete:
            print("Capture did not finish cleanly or metadata.yaml is missing; check mapper.log before finalizing.",
                  file=sys.stderr, flush=True)
            if result in (0, 130, 143):
                result = 1
        try:
            _update_run_metadata(log_directory, {
                "capture_complete": complete,
                "capture_exit_code": result,
                "capture_finished_at": datetime.now(timezone.utc).isoformat(),
            })
        except OSError as exc:
            print(f"Cannot update capture completion metadata: {exc}", file=sys.stderr, flush=True)
            return 1
        if complete:
            print("Capture stopped cleanly. Generate PCD: " + shlex.join(finalize_command), flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
