"""ARM64/x86_64 hardware preparation; no SDK connection, motion or firing."""
from __future__ import annotations

import argparse
from datetime import datetime
import importlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from .sdk_platform import inspect_sdk

PACKAGE = Path(__file__).resolve().parent


def write_report(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")


def preflight(sdk_root, *, check_import=False):
    report = inspect_sdk(sdk_root)
    report.update(scope="Hardware dependency/observation preparation only; no robot client constructed",
                  autonomous_hardware_ready=False, motion_enabled=False, shooting_enabled=False,
                  simulation_runtime_required=False, dependencies={})
    for name in ("numpy", "cv2", "pupil_apriltags"):
        try:
            module = importlib.import_module(name)
            report["dependencies"][name] = getattr(module, "__version__", "import OK")
        except (ImportError, OSError) as exc:
            report["errors"].append(f"{name}: {exc}")
    if report["ready"]:
        library_dir = str(Path(report["files"]["library"]).parent)
        native_env = dict(os.environ)
        native_env["LD_LIBRARY_PATH"] = library_dir + (":" + native_env["LD_LIBRARY_PATH"]
                                                        if native_env.get("LD_LIBRARY_PATH") else "")
        for name, path in report["files"].items():
            try:
                result = subprocess.run(["ldd", path], capture_output=True, text=True,
                                        timeout=15, check=False, env=native_env)
                report.setdefault("libraries", {})[name] = result.stdout+result.stderr
                if result.returncode or "not found" in report["libraries"][name]:
                    report["errors"].append(f"Unresolved {name} dependencies; see libraries.{name}")
            except (OSError, subprocess.TimeoutExpired) as exc:
                report["errors"].append(f"Cannot check {name} dependencies: {exc}")
    if not report["errors"]:
        try:
            # Exercise the ARM-compiled AprilTag library and its managed
            # destructor, not merely the pure-Python wrapper import.
            import numpy as np
            from .perception import create_managed_detector
            detector = create_managed_detector({"family": "tag36h11", "detector_threads": 2})
            try:
                detector.detect(np.zeros((64, 64), np.uint8), estimate_tag_pose=False)
            finally:
                detector.close()
            report["apriltag_native_smoke"] = True
        except Exception as exc:
            report["errors"].append(f"AprilTag native smoke failed: {exc}")
    if check_import and not report["errors"]:
        try:
            from .official_sdk import load_official_sdk_module
            module = load_official_sdk_module(sdk_root)
            if not hasattr(module, "HighLevel"):
                raise ValueError("Official binding has no HighLevel interface")
            report["sdk_imported"] = True
        except Exception as exc:
            report["errors"].append(f"SDK import failed: {exc}")
    report["ready"] = not report["errors"]
    return report


def validate_observer_config(config):
    if (not isinstance(config, dict) or config.get("backend") != "hardware_observer"
            or config.get("robot_model") != "zsl-1"):
        raise ValueError("Use the hardware_observer ZSL-1 profile; simulation configs are not accepted")
    ids = config.get("target_ids")
    if (not isinstance(ids, list) or not ids
            or any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v < 587 for v in ids)
            or len(set(ids)) != len(ids)):
        raise ValueError("target_ids must be unique tag36h11 integers in 0..586")
    tag = config.get("tag", {})
    if not isinstance(tag, dict) or tag.get("family") != "tag36h11":
        raise ValueError("This observer profile requires tag36h11")
    threads = tag.get("detector_threads", 2)
    if isinstance(threads, bool) or not isinstance(threads, int) or not 1 <= threads <= 8:
        raise ValueError("detector_threads must be an integer in 1..8")
    for key, default in (("quad_decimate", 1.0), ("quad_sigma", 0.0), ("min_decision_margin", 40.0)):
        value = tag.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                or value < 0 or (key == "quad_decimate" and value < 1)):
            raise ValueError(f"Invalid {key}")
    hamming = tag.get("max_hamming", 0)
    if isinstance(hamming, bool) or not isinstance(hamming, int) or not 0 <= hamming <= 2:
        raise ValueError("max_hamming must be an integer in 0..2")
    video = config.get("video", {})
    if not isinstance(video, dict):
        raise ValueError("video must be an object")
    for key in ("open_timeout_ms", "read_timeout_ms"):
        value = video.get(key, 2000)
        if isinstance(value, bool) or not isinstance(value, int) or not 100 <= value <= 10000:
            raise ValueError(f"{key} must be an integer in 100..10000")


def identity_records(results, config):
    """Raw-image IDs/corners only; no invented calibration or acquisition time."""
    records = []
    for result in results:
        margin = float(result.decision_margin)
        if (not math.isfinite(margin) or margin < config["tag"].get("min_decision_margin", 40)
                or result.hamming > config["tag"].get("max_hamming", 0)):
            continue
        corners = [[float(v) for v in corner] for corner in result.corners]
        if len(corners) != 4 or any(len(c) != 2 or not all(math.isfinite(v) for v in c) for c in corners):
            continue
        records.append({"tag_id": int(result.tag_id), "declared_target": int(result.tag_id) in config["target_ids"],
                        "decision_margin": margin, "hamming": int(result.hamming), "corners_px": corners})
    return records


def observe_video(args):
    if not args.rtsp or not args.rtsp.lower().startswith(("rtsp://", "rtsps://")):
        raise ValueError("--video-only requires --rtsp with the actual camera RTSP URL")
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    validate_observer_config(config)
    if args.show and sys.platform == "linux" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise ValueError("--show needs a desktop display; omit it in an SSH terminal")
    output = (args.output or PACKAGE/"runs"/f"hardware_observe_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError(f"Refusing nonempty observation output: {output}")
    report = {"status": "ERROR", "scope": "real-video identity observation only",
              "sdk_initialized": False, "motion_enabled": False, "shooting_enabled": False,
              "camera_calibration_applied": False, "pose_estimated": False,
              "source_acquisition_timestamp_available": False, "usable_for_motion_or_fire": False,
              "frames": 0, "observed_ids": []}
    detector = cap = None
    started = time.monotonic()
    try:
        import cv2
        import numpy as np
        from .perception import create_managed_detector
        cv2.setNumThreads(2)
        # Explicit FFMPEG backend: pip's OpenCV wheels are not assumed to
        # include GStreamer or RK3588 hardware decoding.
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")
        video = config.get("video", {})
        cap = cv2.VideoCapture(args.rtsp, cv2.CAP_FFMPEG, [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, video.get("open_timeout_ms", 3000),
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, video.get("read_timeout_ms", 2000)])
        if not cap.isOpened():
            raise RuntimeError("Cannot open RTSP stream; verify the actual camera address and service")
        detector = create_managed_detector(config["tag"])
        seen = set()
        with (output/"detections.jsonl").open("x", encoding="utf-8") as events:
            while time.monotonic()-started < args.seconds:
                ok, image = cap.read()
                decoded_at = time.monotonic()
                if not ok or image is None:
                    raise RuntimeError("RTSP frame read failed or timed out; observation stopped")
                results = detector.detect(np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)), estimate_tag_pose=False)
                records = identity_records(results, config)
                ids = sorted({record["tag_id"] for record in records})
                seen.update(ids)
                report["frames"] += 1
                report["observed_ids"] = sorted(seen)
                events.write(json.dumps({"frame_index": report["frames"], "decoded_at_monotonic_s": decoded_at,
                    "source_acquisition_timestamp_s": None, "usable_for_motion_or_fire": False,
                    "frame_size": [int(image.shape[1]), int(image.shape[0])], "detections": records}, allow_nan=False)+"\n")
                events.flush()
                if ids:
                    print(f"TASK3_HARDWARE_TAGS frame={report['frames']} ids={ids}", flush=True)
                if report["frames"] == 1 and not cv2.imwrite(str(output/"first_frame.jpg"), image):
                    raise RuntimeError("Cannot save first camera image")
                if args.show:
                    for row in records:
                        cv2.polylines(image, [np.rint(row["corners_px"]).astype(np.int32)], True, (0, 255, 0), 2)
                        point = tuple(np.rint(row["corners_px"][0]).astype(int))
                        cv2.putText(image, str(row["tag_id"]), point, cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
                    cv2.imshow("Task3 real camera - observer only", image)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        report["status"] = "CANCELLED"
                        break
            else:
                report["status"] = "OBSERVATION_COMPLETED" if report["frames"] else "ERROR"
        if report["frames"] == 0:
            raise RuntimeError("No frames received before observation deadline")
    except KeyboardInterrupt:
        report["status"] = "CANCELLED"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        cleanup = []
        for resource, method in ((detector, "close"), (cap, "release")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as exc:
                    cleanup.append(str(exc))
        if args.show:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception as exc:
                cleanup.append(str(exc))
        report.update(cleanup_errors=cleanup, wall_time_s=time.monotonic()-started)
        if cleanup:
            report["status"] = "CLEANUP_ERROR"
        write_report(output/"summary.json", report)
    print(f"TASK3_HARDWARE_OBSERVATION status={report['status']} summary={output/'summary.json'}", flush=True)
    if report.get("error"):
        print(report["error"], file=sys.stderr)
    return 0 if report["status"] == "OBSERVATION_COMPLETED" else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--preflight", action="store_true", help="read-only dependencies/architecture check")
    modes.add_argument("--sdk-import-check", action="store_true", help="also import official binding; never construct HighLevel or call initRobot")
    modes.add_argument("--video-only", action="store_true", help="real RTSP Tag ID observation without SDK")
    parser.add_argument("--sdk-root", type=Path, default=PACKAGE.parent/"deps/zsibot_sdk")
    parser.add_argument("--config", type=Path, default=PACKAGE/"config/hardware_rk3588.json")
    parser.add_argument("--rtsp", help="actual camera URL; do not assume it is localhost")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--output", type=Path, help="new report JSON for checks, or new/empty directory for video")
    parser.add_argument("--show", action="store_true", help="requires a graphical desktop")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0,600]")
    try:
        if args.video_only:
            return observe_video(args)
        report = preflight(args.sdk_root, check_import=args.sdk_import_check)
        if args.output:
            write_report(args.output, report)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
        print(f"TASK3_HARDWARE_PREFLIGHT ready={str(report['ready']).lower()} sdk_initialized=false motion=false shooting=false")
        return 0 if report["ready"] else 2
    except (ValueError, OSError, ImportError) as exc:
        print(f"Task3 hardware preparation error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
