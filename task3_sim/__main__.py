"""Single-target image-to-actuation-to-referee milestone entry point."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import faulthandler
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import sys
import threading
import time
import traceback
import uuid


PACKAGE = Path(__file__).resolve().parent
PROJECT = PACKAGE.parent
_fault_fd = None


@contextmanager
def defer_cleanup_sigint():
    """Keep a repeated Ctrl+C from abandoning the bounded native shutdown.

    Only SIGINT is deferred, and only on the main thread during cleanup. A
    native abort/segfault, SystemExit, or cleanup exception is not swallowed.
    The caller still reports a cleanup failure and returns a nonzero status.
    """
    received = []
    if threading.current_thread() is not threading.main_thread():
        yield received
        return
    previous = signal.getsignal(signal.SIGINT)

    def remember(signum, frame):
        received.append(signum)

    signal.signal(signal.SIGINT, remember)
    try:
        yield received
    finally:
        signal.signal(signal.SIGINT, previous)


def configure_overview_camera(camera, config):
    """Frame both the initial robot position and target without altering RGB."""
    initial = config["simulation"]["initial_position_m"]
    target = config["simulation"]["target_center_world_m"]
    camera.lookat[:] = [(left+right)/2 for left,right in zip(initial,target)]
    camera.distance = max(3.5, 1.6*math.dist(initial,target))
    # MuJoCo azimuth 130 places the observer behind the textured background
    # wall. This front-quarter view keeps both the dog and board visible.
    camera.azimuth, camera.elevation = 45.0, -18.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PACKAGE/"config/single_target.json")
    parser.add_argument("--model-root", type=Path, default=PROJECT/"src/robot_mujoco/zsibot_robots/xgb")
    parser.add_argument("--output", type=Path, help="new/empty run directory; defaults to task3_sim/runs/run_<time>_<id>")
    parser.add_argument("--max-seconds", type=float, default=25.0, help="simulation time limit")
    parser.add_argument("--headless", action="store_true", help="offscreen RGB rendering, no UI (default)")
    parser.add_argument("--show", action="store_true", help="OpenCV observation window; Esc cancels")
    parser.add_argument("--viewer", action="store_true", help="MuJoCo passive observer; no task teleoperation")
    parser.add_argument("--realtime", action="store_true", help="pace simulation against wall time")
    parser.add_argument("--hold-seconds", type=float, default=None,
                        help="interactive result display: default waits for Esc/Q/window close; 0 exits immediately")
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_seconds) or args.max_seconds <= 0 or args.max_seconds > 600:
        parser.error("--max-seconds must be in (0,600]")
    if args.headless and (args.show or args.viewer):
        parser.error("--headless cannot be combined with --show/--viewer")
    if args.hold_seconds is not None and (not math.isfinite(args.hold_seconds) or args.hold_seconds < 0):
        parser.error("--hold-seconds must be finite and nonnegative")
    return args


def json_value(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"not JSON serializable: {type(value)}")


def write_json(path, value):
    path.write_text(json.dumps(value, default=json_value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def save_rgb(path, rgb):
    import cv2
    path.write_bytes(cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))[1].tobytes())


def enable_fault_log(output):
    """Keep one raw descriptor alive through Python/native finalization."""
    global _fault_fd
    descriptor = os.open(str(output/"fault.log"), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        faulthandler.enable(file=descriptor, all_threads=True)
    except Exception:
        os.close(descriptor)
        raise
    previous, _fault_fd = _fault_fd, descriptor
    if previous is not None:
        os.close(previous)


def annotate(frame, detections, decision, camera):
    import cv2
    import numpy as np
    image = frame.rgb.copy()
    for detection in detections:
        points = np.round(detection.corners_px).astype(np.int32)
        cv2.polylines(image, [points], True, (30,255,100), 2)
        cv2.putText(image, f"AprilTag {detection.tag_id}", tuple(points[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30,255,100), 1, cv2.LINE_AA)
    if decision.geometry is not None:
        from .geometry import camera_extrinsics
        rotation, position = camera_extrinsics(camera)
        p = rotation.T @ (decision.geometry.bullseye_base_m-position)
        if p[2] > 0:
            uv = (round(camera["fx"]*p[0]/p[2]+camera["cx"]), round(camera["fy"]*p[1]/p[2]+camera["cy"]))
            cv2.drawMarker(image, uv, (255,80,30), cv2.MARKER_CROSS, 20, 2)
    text = decision.state
    if decision.geometry is not None:
        text += f"  predicted miss {decision.geometry.error_norm_m*1000:.1f} mm"
    cv2.rectangle(image, (0,0), (image.shape[1],36), (15,15,15), -1)
    cv2.putText(image, text, (12,25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 1, cv2.LINE_AA)
    return image


def run(args) -> tuple[int, Path]:
    # Set the rendering backend before importing MuJoCo/OpenGL. Windows uses
    # GLFW's offscreen context; Ubuntu headless defaults to EGL.
    if sys.platform.startswith("linux") and not (args.show or args.viewer):
        os.environ.setdefault("MUJOCO_GL", "egl")
    output = (args.output or PACKAGE/"runs"/f"run_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    # The Bash launcher creates terminal.log before starting Python so native
    # stdout/stderr, including aborts during interpreter exit, are retained.
    if output.exists() and any(path.name != "terminal.log" for path in output.iterdir()):
        raise ValueError(f"refusing to overwrite nonempty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    enable_fault_log(output)
    from . import __version__
    summary = {"status": "ERROR", "milestone": "single_target_render_apriltag_autoaim_ray",
               "package_version": __version__,
               "output": str(output), "platform": platform.platform(), "python": platform.python_version(),
               "mujoco_gl": os.environ.get("MUJOCO_GL", "default"), "shutdown_completed": False,
               "motion_model": "kinematic_base_with_velocity_lag; no gait dynamics or hardware control",
               "official_score": False, "official_files_modified": False}
    robot, shooter, viewer, perception, ui = None, None, None, None, None
    wall_start = time.monotonic()
    last_frame = last_decision = last_detections = None
    result_saved = False
    try:
        import cv2
        import mujoco
        import numpy as np
        from .backend import MujocoRobot
        from .controller import AutoAimController
        from .perception import AprilTagPerception
        from .scene import build_scene
        from .scoring import RayScorer
        from .ui import InteractiveSession

        summary["mujoco_version"] = mujoco.__version__
        config = json.loads(args.config.read_text(encoding="utf-8-sig"))
        write_json(output/"config.json", config)
        summary["config_sha256"] = hashlib.sha256(args.config.read_bytes()).hexdigest()
        model, provenance = build_scene(config, args.model_root, output/"generated")
        summary["source_model"] = provenance["source_model"]
        robot = MujocoRobot(model, config)
        robot.Connect()
        robot.StandUp()
        # Deliberately keep target truth out of the vision/control objects.
        public_config = {key: config[key] for key in ("camera", "laser", "tag", "control")}
        perception = AprilTagPerception(public_config)
        controller = AutoAimController(public_config)
        shooter = RayScorer(model, robot.data, config)
        shooter.shooting_init()
        ui = InteractiveSession(show=args.show)
        if args.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, robot.data, key_callback=ui.on_viewer_key,
                                                 show_left_ui=False, show_right_ui=False)
            ui.viewer = viewer
            configure_overview_camera(viewer.cam, config)
            viewer.opt.geomgroup[2] = 0
            viewer.opt.geomgroup[3] = 1
        dt = 1.0/config["simulation"]["frame_rate_hz"]
        states, count, shot = [], 0, None
        # Scene compilation and GUI startup must not consume the pacing budget.
        mission_wall_start = time.monotonic()
        with (output/"events.jsonl").open("w", encoding="utf-8") as events:
            while robot.data.time <= args.max_seconds:
                frame = robot.capture()
                detections = perception.detect(frame)
                feedback = robot.ReadFeedback()
                decision = controller.update(frame, detections, feedback, float(robot.data.time))
                last_frame, last_decision, last_detections = frame, decision, detections
                if not states or states[-1] != decision.state:
                    states.append(decision.state)
                    print(f"[{robot.data.time:6.2f}s] {decision.state}: {decision.reason}", flush=True)
                robot.Execute(decision.command)
                event = {"time_s": frame.timestamp, "state": decision.state, "reason": decision.reason,
                         "command": asdict(decision.command), "feedback": asdict(feedback),
                         "detections": [asdict(d) for d in detections],
                         "geometry": asdict(decision.geometry) if decision.geometry and np.isfinite(decision.geometry.error_norm_m) else None}
                if count == 0:
                    save_rgb(output/"initial_rgb.png", frame.rgb)
                if decision.should_fire:
                    robot.Stop()
                    if shooter.ammo_remaining <= 0:
                        summary["status"] = "NO_AMMO"
                        event["fire_rejected"] = "no_ammo"
                    else:
                        shot = shooter.shooting_fire()
                        controller.mark_fired()
                        event["shot"] = asdict(shot)
                        summary["status"] = "SUCCESS" if shot.hit else "MISSED"
                events.write(json.dumps(event, default=json_value, ensure_ascii=False, allow_nan=False)+"\n")
                events.flush()
                count += 1
                preview = cv2.cvtColor(annotate(frame,detections,decision,config["camera"]), cv2.COLOR_RGB2BGR) if args.show else None
                if not ui.present(preview):
                    summary["status"] = "CANCELLED"
                    break
                if shot is not None or summary["status"] == "NO_AMMO":
                    break
                robot.advance(dt)
                if args.realtime:
                    time.sleep(max(0.0, float(robot.data.time)-(time.monotonic()-mission_wall_start)))
            else:
                summary["status"] = "TIMEOUT"
        summary.update(simulation_time_s=float(robot.data.time), frames=count, states=states,
                       detections_last_frame=len(last_detections), ammo_remaining=shooter.ammo_remaining,
                       shot=asdict(shot) if shot else None, last_scores=shooter.last_scores,
                       mission_wall_time_s=time.monotonic()-mission_wall_start,
                       perception_pose_backend="AprilTag3 detection + OpenCV IPPE/LM pose")
        if last_frame is not None:
            save_rgb(output/"final_rgb.png", last_frame.rgb)
            save_rgb(output/"final_annotated.png", annotate(last_frame,last_detections,last_decision,config["camera"]))
            # Ground truth comparison is strictly post-decision diagnostics.
            if last_decision.geometry is not None:
                base = shooter.base_id
                estimated_world = robot.data.xpos[base] + robot.data.xmat[base].reshape(3,3) @ last_decision.geometry.bullseye_base_m
                summary["diagnostics"] = {"estimated_bullseye_world_m": estimated_world,
                    "bullseye_position_error_m": float(np.linalg.norm(estimated_world-np.array(config["simulation"]["target_center_world_m"]))),
                    "predicted_miss_m": last_decision.geometry.error_norm_m if np.isfinite(last_decision.geometry.error_norm_m) else None,
                    "actual_miss_m": float(np.linalg.norm(shot.point_world_m-np.array(config["simulation"]["target_center_world_m"]))) if shot and shot.target_id is not None else None}
            overview = mujoco.MjvCamera()
            configure_overview_camera(overview, config)
            robot.renderer.update_scene(robot.data, camera=overview, scene_option=robot.render_option)
            # Post-shot overview only: this visual beam never enters perception.
            if shot is not None and shot.point_world_m is not None:
                scene = robot.renderer.scene
                beam = scene.geoms[scene.ngeom]
                origin = robot.data.xpos[shooter.base_id] + robot.data.xmat[shooter.base_id].reshape(3,3) @ np.array(config["laser"]["origin_base_m"])
                mujoco.mjv_initGeom(beam, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), origin,
                                   np.eye(3).reshape(9), np.array([1.0,0.1,0.05,1.0],dtype=np.float32))
                mujoco.mjv_connector(beam, mujoco.mjtGeom.mjGEOM_LINE, 3.0, origin, shot.point_world_m)
                scene.ngeom += 1
                if viewer is not None and viewer.is_running():
                    with viewer.lock():
                        viewer.user_scn.ngeom = 1
                        target_beam = viewer.user_scn.geoms[0]
                        mujoco.mjv_initGeom(target_beam, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), origin,
                                           np.eye(3).reshape(9), np.array([1.0,0.1,0.05,1.0],dtype=np.float32))
                        mujoco.mjv_connector(target_beam, mujoco.mjtGeom.mjGEOM_LINE, 3.0, origin, shot.point_world_m)
            save_rgb(output/"overview.png", robot.renderer.render())
        # Verify precisely the original model/mesh files read by this run.
        summary["model_sources_unchanged"] = all(
            hashlib.sha256((args.model_root/relative).read_bytes()).hexdigest() == digest
            for relative,digest in provenance["source_files_sha256"].items())
        robot.Stop()
        # Destroy AprilTag's native resources in an explicit safe order before
        # any result-window waiting or interpreter teardown.
        perception.close()
        summary["perception_closed"] = True
        summary["phase"] = "result_ready"
        write_json(output/"summary.json", summary)
        result_saved = True
        print(f"TASK3_RESULT status={summary['status']} summary={output/'summary.json'}", flush=True)
        if args.show or args.viewer:
            if args.hold_seconds != 0:
                duration = "until Esc/Q/window close" if args.hold_seconds is None else f"for {args.hold_seconds:g} wall seconds"
                print(f"TASK3_DISPLAY holding result {duration}; simulation stopped, no further shots", flush=True)
            result_image = None
            if args.show and last_frame is not None:
                result_image = cv2.cvtColor(annotate(last_frame,last_detections,last_decision,config["camera"]),cv2.COLOR_RGB2BGR)
                label = f"{summary['status']}" + (f" - {shot.ring} ring" if shot else "") + " - Esc/Q to close"
                cv2.putText(result_image, label, (12,58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2, cv2.LINE_AA)
            summary["display_exit_reason"] = ui.hold_result(result_image, args.hold_seconds)
    except KeyboardInterrupt:
        if result_saved:
            summary["display_exit_reason"] = "keyboard_interrupt_after_result"
        else:
            summary["status"] = "CANCELLED"
    except Exception as exc:
        if summary["status"] != "ERROR":
            summary["task_status_before_error"] = summary["status"]
        summary["status"] = "ERROR"
        summary["error"] = str(exc)
        (output/"error.log").write_text(traceback.format_exc(), encoding="utf-8")
        print(f"ERROR: {exc}", file=sys.stderr)
    finally:
        with defer_cleanup_sigint() as cleanup_interrupts:
            cleanup_errors = []
            # Viewer render-thread cleanup precedes renderer/GL context destruction.
            for name, resource, method in (("ui",ui,"close"), ("perception",perception,"close"),
                                            ("shooter",shooter,"shooting_shutdown"), ("robot",robot,"close")):
                if resource is not None:
                    try:
                        warnings = getattr(resource,method)()
                        if isinstance(warnings,list):
                            cleanup_errors.extend(f"{name}: {warning}" for warning in warnings)
                    except Exception as cleanup_exc:
                        cleanup_errors.append(f"{name}: {cleanup_exc}")
            summary["cleanup_errors"] = cleanup_errors
            summary["cleanup_sigint_count"] = len(cleanup_interrupts)
            summary["shutdown_completed"] = not cleanup_errors
            if cleanup_errors:
                summary["task_status_before_cleanup"] = summary["status"]
                summary["status"] = "CLEANUP_ERROR"
            summary["phase"] = "finished" if not cleanup_errors else "cleanup_error"
            summary["wall_time_s"] = time.monotonic()-wall_start
            write_json(output/"summary.json", summary)
    code = 0 if summary["status"] == "SUCCESS" and summary.get("model_sources_unchanged") and summary["shutdown_completed"] else 2
    print(f"TASK3_{summary['status']} summary={output/'summary.json'}", flush=True)
    if summary.get("shot"):
        print(f"ring={summary['shot']['ring']} ammo_remaining={summary['ammo_remaining']}", flush=True)
    return code, output


def main(argv=None):
    try:
        args = parse_args(argv)
        config = json.loads(args.config.read_text(encoding="utf-8-sig"))
        if "targets" in config:
            from .multi import run_multi
            code, _ = run_multi(args)
        else:
            code, _ = run(args)
        return code
    except Exception as exc:
        print(f"Task3 startup error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
