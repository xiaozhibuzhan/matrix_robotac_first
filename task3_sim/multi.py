"""Independent fixed-target mission runner; single-target entry stays intact."""

from __future__ import annotations

from dataclasses import asdict, replace
from contextlib import nullcontext
from datetime import datetime
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time
import traceback
import uuid

from .__main__ import PACKAGE, annotate, defer_cleanup_sigint, enable_fault_log, json_value, save_rgb, write_json as original_write_json
from .telemetry import sanitize_record


def write_json(path, value):
    # Diagnostics only: control objects retain their invalid/NaN/inf values.
    original_write_json(path, sanitize_record(value) if path.name == "summary.json" else value)


def target_records(mission_summary):
    targets = mission_summary.get("targets", [])
    return list(targets.values()) if isinstance(targets, dict) else targets


def shot_progress_lines(mission_summary, ammo):
    """Human-readable receipts use committed shots, never scores as counters."""
    shots = mission_summary.get("shots", [])
    lines = ["TASK 3 - SHOT RECEIPTS", f"Shots fired: {len(shots)}",
             f"Total score: {mission_summary.get('total_score', 0)}",
             f"Ammo remaining: {ammo}", f"Round: {mission_summary.get('round_index', 1)}", ""]
    for record in target_records(mission_summary):
        identity = record.get("id", record.get("target_id"))
        lines.append(f"ID {identity}: shots={record.get('attempts', 0)} sum={record.get('score_sum', 0)} latest={record.get('ring', 0)}")
    lines.extend(["", "SHOT HISTORY (last 5)"])
    cumulative = 0
    for index, shot in enumerate(shots):
        cumulative += shot.get("credited_ring", 0)
        if index >= len(shots)-5:
            lines.append(f"#{shot['shot_index']} R{shot.get('round_index', 1)} ID{shot['intended_target_id']} -> {shot.get('actual_target_id')}  +{shot.get('credited_ring', 0)}")
            lines.append(f"  ammo {shot['ammo_before']}->{shot['ammo_remaining']}  total={cumulative}")
    return lines


def update_viewer_progress(viewer, mission_summary, ammo):
    if viewer is not None and hasattr(viewer, "set_texts"):
        import mujoco
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                          "\n".join(shot_progress_lines(mission_summary, ammo)), ""))


def save_shot_reports(output, mission_summary, physical_ledger):
    """Export an inspectable row for every committed debit, including failures."""
    write_json(output/"shots.json", physical_ledger)
    write_json(output/"mission.json", mission_summary)
    columns = ("shot_index", "mission_accepted", "credit_status", "round_index", "target_attempt", "timestamp", "intended_target_id",
               "actual_target_id", "ring", "credited_ring", "cumulative_score_after", "ammo_before",
               "ammo_remaining", "reason")
    with (output/"shots.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        total = 0
        accepted = {shot["shot_index"]: shot for shot in mission_summary.get("shots", [])}
        for index, physical in enumerate(physical_ledger, 1):
            sequence = physical.get("sequence", index)
            shot = accepted.get(sequence)
            if shot is not None:
                total += shot.get("credited_ring", 0)
                row = {**shot, "mission_accepted": True, "credit_status": "confirmed"}
            else:
                # A debit followed by a rejected mission transaction is an
                # error, not a missing physical shot or an invented score.
                row = {"shot_index": sequence, "mission_accepted": False, "credit_status": "unconfirmed",
                       "timestamp": physical.get("timestamp_s"),
                       "intended_target_id": physical.get("intended_target_id"),
                       "actual_target_id": physical.get("actual_target_id"), "ring": physical.get("ring"),
                       "credited_ring": "", "ammo_before": physical.get("ammo_before"),
                       "ammo_remaining": physical.get("ammo_after"),
                       "reason": "mission_commit_missing: " + str(physical.get("reason", ""))}
            writer.writerow({**row, "cumulative_score_after": total})


def print_shot_receipt(mission_summary, ammo):
    shot = mission_summary["shots"][-1]
    print(f"TASK3_SHOT index={shot['shot_index']} round={shot.get('round_index', 1)} "
          f"intended_id={shot['intended_target_id']} actual_id={shot.get('actual_target_id')} "
          f"ring={shot['ring']} credited={shot['credited_ring']} "
          f"total_score={mission_summary['total_score']} ammo_before={shot['ammo_before']} ammo_remaining={ammo}", flush=True)


def annotate_multi(frame, detections, decision, camera, mission_summary, ammo, age=None):
    import cv2
    import numpy as np
    image = annotate(frame, [], decision, camera)
    records = target_records(mission_summary)
    states = {int(record.get("id", record.get("target_id"))): record["state"] for record in records}
    for detection in detections:
        state = states.get(detection.tag_id, "DISTRACTOR")
        color = (255, 80, 80) if state == "DISTRACTOR" else ((255, 220, 20) if detection.tag_id == decision.target_id else (30, 255, 100))
        points = np.rint(detection.corners_px).astype(np.int32)
        cv2.polylines(image, [points], True, color, 2)
        cv2.putText(image, f"ID {detection.tag_id} {state}", tuple(points[3]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
    cv2.rectangle(image, (0, 36), (image.shape[1], 91), (15, 15, 15), -1)
    active = "none" if decision.target_id is None else str(decision.target_id)
    text = f"active={active}  ammo={ammo}  mission={mission_summary.get('status', 'RUNNING')}"
    if mission_summary.get("shooting_policy") == "round_robin":
        text += f"  round={mission_summary.get('round_index', 1)}"
    if age is not None:
        text += f"  image age={age*1000:.0f}ms"
    cv2.putText(image, text, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    labels = f"SHOTS FIRED={len(mission_summary.get('shots', []))}  TOTAL SCORE={mission_summary.get('total_score', 0)}"
    cv2.putText(image, labels, (12, 81), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    lines = shot_progress_lines(mission_summary, ammo)
    height = max(image.shape[0], 26+len(lines)*22)
    panel = np.full((height, image.shape[1]+370, 3), 20, dtype=np.uint8)
    panel[:image.shape[0], :image.shape[1]] = image
    for index, line in enumerate(lines):
        color = (80, 255, 140) if line.startswith(("Shots fired:", "Total score:", "#")) else (240, 240, 240)
        cv2.putText(panel, line, (image.shape[1]+12, 24+index*22), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, color, 1, cv2.LINE_AA)
    return panel


def configure_multi_overview(camera, config):
    import numpy as np
    from .multi_config import expand_target
    initial = np.asarray(config["simulation"]["initial_position_m"], dtype=float)
    centers = np.array([expand_target(config, target)["simulation"]["target_center_world_m"]
                        for target in config["targets"]])
    camera.lookat[:] = (initial+centers.mean(axis=0))/2
    span = max(float(np.linalg.norm(center-initial)) for center in centers)
    camera.distance = max(4.5, 1.75*span)
    camera.azimuth, camera.elevation = 45, -20
    if config.get("appearance", {}).get("background", {}).get("layout") == "surround":
        # Look through the open roof so four real background walls do not
        # obscure the dog and clock-direction targets in the observer view.
        # Robot-camera rendering keeps all four walls visible as usual.
        height = float(config["appearance"]["background"]["height_m"])
        camera.lookat[:] = [0.0, 0.0, 0.3]
        camera.distance = max(8.5, 2.7*span, height+4.0)
        camera.azimuth, camera.elevation = 45, -65


def draw_beams(scene, beams):
    import mujoco
    import numpy as np
    for beam in beams:
        if scene.ngeom >= scene.maxgeom:
            break
        geom = scene.geoms[scene.ngeom]
        color = [0.1, 1.0, 0.2, 1.0] if beam["correct_hit"] else [1.0, 0.1, 0.05, 1.0]
        origin, point = np.array(beam["origin"]), np.array(beam["point"])
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), origin,
                           np.eye(3).reshape(9), np.array(color, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 2.0, origin, point)
        scene.ngeom += 1


def _fire_transaction(mission, shooter, target_id, now):
    """Commit the physical shot and mission debit before honoring Ctrl+C."""
    if not mission.authorize_shot(target_id, now, shooter.ammo_remaining):
        raise RuntimeError("physical shot has no current mission authorization")
    with defer_cleanup_sigint() as interrupts:
        before = shooter.ammo_remaining
        shot = shooter.shooting_fire(target_id)
        accepted = mission.record_shot(target_id, shot, now, shooter.ammo_remaining)
        if shooter.ammo_remaining != before-1 or not accepted:
            raise RuntimeError("shot ledger and armed mission transaction disagree")
    return shot, interrupts


def _attempt_fire(mission, robot, shooter, decision, frame, feedback, now):
    """Keep a verified policy hold through the atomic virtual shot/debit."""
    guard = robot.fire_guard() if hasattr(robot, "fire_guard") else nullcontext()
    with guard:
        gate = None
        if mission.shooting_policy == "round_robin":
            checked_at = robot.current_time() if hasattr(robot, "current_time") else now
            if (frame is None or not decision.should_fire or not mission.authorize_shot(
                    decision.target_id, checked_at, shooter.ammo_remaining,
                    frame_timestamp=frame.timestamp)):
                robot.Stop()
                return None, [], {"allowed": False, "reason": "mission target identity, visit or image authority expired"}, now
        if hasattr(robot, "prepare_fire"):
            gate = robot.prepare_fire(frame, feedback)
            if not gate["allowed"]:
                return None, [], gate, now
            if gate.get("target_id") != decision.target_id:
                return None, [], {**gate, "allowed": False, "reason": "held target identity differs from armed target"}, now
        else:
            robot.Stop()
        if frame is None or decision.target_id is None or shooter.ammo_remaining <= 0:
            raise RuntimeError("mission requested firing without fresh image, intended ID or ammunition")
        fire_time = robot.current_time() if hasattr(robot, "current_time") else now
        if not mission.authorize_shot(decision.target_id, fire_time, shooter.ammo_remaining,
                frame_timestamp=frame.timestamp if mission.shooting_policy == "round_robin" else None):
            robot.Stop()
            return None, [], {"allowed": False, "reason": "mission target identity or fire authority expired before debit"}, now
        shot, interrupts = _fire_transaction(mission, shooter, decision.target_id, fire_time)
        return shot, interrupts, gate, fire_time


def run_multi(args) -> tuple[int, Path]:
    if sys.platform.startswith("linux") and not (args.show or args.viewer):
        os.environ.setdefault("MUJOCO_GL", "egl")
    output = (args.output or PACKAGE/"runs"/f"multi_{datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists() and any(path.name != "terminal.log" for path in output.iterdir()):
        raise ValueError(f"refusing to overwrite nonempty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    enable_fault_log(output)
    from . import __version__
    summary = {"status": "ERROR", "milestone": "multi_fixed_target_identity_switch_finite_ammo",
        "package_version": __version__, "output": str(output), "platform": platform.platform(),
        "python": platform.python_version(), "mujoco_gl": os.environ.get("MUJOCO_GL", "default"),
        "shutdown_completed": False, "official_files_modified": False, "official_score": False,
        "motion_model": "kinematic_base_with_velocity_lag; no gait dynamics or moving targets"}
    robot = shooter = viewer = perception = ui = mission = transport = None
    last_frame = last_decision = None
    last_detections = []
    result_saved = False
    wall_start = time.monotonic()
    beams, diagnostics = [], []
    count = delivered_count = 0
    try:
        import cv2
        import mujoco
        import numpy as np
        from .backend import MujocoRobot
        from .mission import MultiTargetMission
        from .multi_config import expand_target, public_mission_config
        from .multi_perception import MultiTagPerception
        from .multi_scene import build_multi_scene
        from .multi_scoring import MultiRayScorer
        from .transport import FrameTransport
        from .ui import InteractiveSession

        config = json.loads(args.config.read_text(encoding="utf-8-sig"))
        write_json(output/"config.json", config)
        summary.update(config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                       mujoco_version=mujoco.__version__)
        model, provenance = build_multi_scene(config, args.model_root, output/"generated")
        summary["source_model"] = provenance["source_model"]
        if config.get("backend", "kinematic") == "official_dynamics":
            from .official_backend import OfficialDynamicsRobot
            from .official_runtime import OfficialRuntime, OfficialRuntimeError
            from .official_scene import prepare_scene
            model, native_scene = prepare_scene(output/"generated/scene.xml", config, args.model_root)
            settings = config.get("official_dynamics", {})
            runtime = OfficialRuntime(PACKAGE.parent, output/"official_runtime", native_scene,
                                      sdk_port=int(settings.get("sdk_port", 44888)))
            bridge = PACKAGE / settings.get("bridge_executable", "local/native/task3_official_state_bridge")
            summary["motion_model"] = OfficialDynamicsRobot.motion_model
            summary["official_dynamics_mission_completed"] = False
            # Check local native dependencies before creating GL resources or
            # UI windows. start() repeats the checks under its process lock.
            report = runtime.preflight(bridge)
            summary["official_startup_preflight"] = report
            write_json(output/"startup_preflight.json", report)
            if not report["ready"]:
                for action in report.get("repair_actions", []):
                    print(f"TASK3_REPAIR cwd={action['cwd']} command={action['command']}", flush=True)
                raise OfficialRuntimeError("Official runtime preflight failed before graphics startup: " + "; ".join(report["errors"]))
            robot = OfficialDynamicsRobot(model, config, runtime=runtime, bridge_executable=bridge, output=output)
        elif config.get("backend", "kinematic") == "kinematic":
            robot = MujocoRobot(model, config)
        else:
            raise ValueError("backend must be kinematic or official_dynamics")
        public = public_mission_config(config)
        perception = MultiTagPerception(public)
        if hasattr(robot, "warmup_renderer"):
            from .interfaces import Frame
            robot.warmup_renderer()
            # Detector cold-start work is discarded before any native command.
            perception.detect(Frame(np.zeros((config["camera"]["height"], config["camera"]["width"], 3), dtype=np.uint8), 0.0))
        mission = MultiTargetMission(public)
        transport = FrameTransport(config.get("observation", {}))
        shooter = MultiRayScorer(model, robot.data, config)
        shooter.shooting_init()
        ui = InteractiveSession(show=args.show)
        if args.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(model, robot.data, key_callback=ui.on_viewer_key,
                                                 show_left_ui=False, show_right_ui=False)
            ui.viewer = viewer
            configure_multi_overview(viewer.cam, config)
            viewer.opt.geomgroup[2] = 0
            viewer.opt.geomgroup[3] = 1
            update_viewer_progress(viewer, mission.summary(), shooter.ammo_remaining)
        robot.Connect()
        robot.StandUp()
        last_observation_epoch = getattr(robot, "observation_epoch", 0)
        dt = 1.0/config["simulation"]["frame_rate_hz"]
        states, last_state = [], None
        last_aim_report_time = -float("inf")
        initial_ammo = shooter.ammo_remaining
        mission_start = time.monotonic()
        mission_deadline = float(robot.data.time)+args.max_seconds if hasattr(robot, "current_time") else args.max_seconds
        with (output/"events.jsonl").open("w", encoding="utf-8") as events:
            while robot.data.time <= mission_deadline:
                frame_work_start = time.monotonic()
                now = float(robot.data.time)
                capture = robot.capture()
                if hasattr(robot, "current_time"):
                    now = robot.current_time()
                if count == 0:
                    save_rgb(output/"initial_rgb.png", capture.rgb)
                transport.push(capture, now)
                frame = transport.poll(now)
                epoch = getattr(robot, "observation_epoch", 0)
                if epoch != last_observation_epoch:
                    mission.invalidate_control_observation(now, "native feedback gap; new hold required")
                    last_observation_epoch = epoch
                if frame is not None and frame.timestamp <= getattr(robot, "minimum_observation_timestamp_s", -float("inf")):
                    frame = None
                detections = perception.detect(frame) if frame is not None else []
                feedback = robot.ReadFeedback()
                if hasattr(robot, "current_time"):
                    now = robot.current_time()
                decision = mission.update(frame, detections, feedback, now, shooter.ammo_remaining)
                last_decision = decision
                if frame is not None:
                    last_frame, last_detections = frame, detections
                    delivered_count += 1
                state = (decision.target_id, decision.state)
                if state != last_state:
                    print(f"[{now:6.2f}s] target={decision.target_id} {decision.state}: {decision.reason}", flush=True)
                    states.append({"time_s": now, "target_id": decision.target_id, "state": decision.state})
                    last_state = state
                if hasattr(robot, "ExecuteDecision"):
                    if robot.ExecuteDecision(decision, observation_timestamp_s=frame.timestamp if frame is not None else None,
                                             feedback=feedback):
                        mission.invalidate_control_observation(robot.current_time())
                        decision = replace(decision, should_fire=False, state="MODE_TRANSITION",
                                           reason="official mode transition completed; waiting for a new image",
                                           command=type(decision.command)())
                        last_decision = decision
                    elif robot.last_execution.get("hold_restarted"):
                        mission.invalidate_control_observation(robot.current_time(), "SDK input interruption; new hold required")
                        decision = replace(decision, should_fire=False, state="HOLD_RESTART",
                                           reason="SDK input interrupted; collecting a new stable hold")
                        last_decision = decision
                    elif robot.last_execution.get("aim_failure"):
                        failure = robot.last_execution["aim_failure"]
                        robot.Stop()
                        decision = mission.fail_active(robot.current_time(), failure)
                        last_decision = decision
                        print(f"[{robot.current_time():6.2f}s] target={decision.target_id} FAILED: {failure}", flush=True)
                else:
                    robot.Execute(decision.command)
                mission_summary = mission.summary()
                records = target_records(mission_summary)
                event = {"time_s": now, "capture_timestamp_s": capture.timestamp,
                    "frame_delivered": frame is not None,
                    "frame_timestamp_s": frame.timestamp if frame is not None else None,
                    "frame_age_s": now-frame.timestamp if frame is not None else None,
                    "state": decision.state, "mission_status": mission.status,
                    "active_target_id": decision.target_id, "reason": decision.reason,
                    "round_index": mission_summary["round_index"], "shooting_policy": mission.shooting_policy,
                    "command": asdict(decision.command), "feedback": asdict(feedback),
                    "detections": [asdict(detection) for detection in detections],
                    "target_states": {str(record.get("id", record.get("target_id"))): record["state"] for record in records},
                    "geometry": asdict(decision.geometry) if decision.geometry and np.isfinite(decision.geometry.error_norm_m) else None}
                if hasattr(robot, "last_execution"):
                    event["official_feedback_meta"] = robot.session.last_feedback_meta
                    event["official_observation_epoch"] = getattr(robot, "observation_epoch", 0)
                    event["official_execution"] = robot.last_execution
                    servo = robot.last_execution.get("aim_servo", {})
                    if decision.geometry is not None and now-last_aim_report_time >= 2.0:
                        held = servo.get("held_command", {})
                        print(f"TASK3_AIM id={decision.target_id} phase={servo.get('phase')} "
                              f"error_mm={decision.geometry.error_norm_m*1000:.2f} "
                              f"yaw_input={held.get('yaw_rate_rad_s', 0):.5f} "
                              f"pitch_input={held.get('pitch_rate_rad_s', 0):.5f}", flush=True)
                        last_aim_report_time = now
                shot_interrupts = []
                shot = None
                if decision.should_fire:
                    shot, shot_interrupts, gate, fire_time = _attempt_fire(
                        mission, robot, shooter, decision, frame, feedback, now)
                    if gate is not None:
                        event["official_fire_gate"] = gate
                    if gate is not None and not gate["allowed"]:
                        mission.invalidate_control_observation(robot.current_time())
                        decision = replace(decision, should_fire=False, state="WAITING_PHYSICAL_STABILITY",
                                           reason=event["official_fire_gate"]["reason"])
                        last_decision = decision
                        event.update(state=decision.state, reason=decision.reason)
                if shot is not None:
                    print_shot_receipt(mission.summary(), shooter.ammo_remaining)
                    # The task manager is the only source of intended IDs. The
                    # independent nearest-surface referee supplies actual IDs.
                    if hasattr(robot, "current_time"):
                        event["decision_time_s"] = now
                        event["time_s"] = fire_time
                        event["frame_age_s"] = fire_time-frame.timestamp
                        event["shot_commit_time_s"] = fire_time
                    event.update(shot=asdict(shot), shot_record=shooter.ledger[-1], shot_accepted=True)
                    event["mission_shot_record"] = mission.summary()["shots"][-1]
                    event.update(post_shot_mission_status=mission.status,
                                 post_shot_target_states={str(key): value for key, value in mission.target_states.items()})
                    target_config = expand_target(config, next(spec for spec in config["targets"] if spec["id"] == decision.target_id))
                    # Simulator positions enter post-shot diagnostics only.
                    truth = np.array(target_config["simulation"]["target_center_world_m"])
                    diag = {"sequence": len(shooter.ledger), "intended_target_id": decision.target_id,
                        "actual_target_id": shot.target_id, "predicted_miss_m": decision.geometry.error_norm_m,
                        "actual_miss_m": float(np.linalg.norm(shot.point_world_m-truth)) if shot.point_world_m is not None and shot.target_id == decision.target_id else None}
                    diagnostics.append(diag)
                    event["diagnostics"] = diag
                    # The hold is preserved through the actual shot/debit,
                    # then revoked even when the next visit has the same ID.
                    robot.Stop()
                    mission.invalidate_control_observation(robot.current_time() if hasattr(robot, "current_time") else fire_time,
                                                           "shot committed; next visit needs a new stable image window")
                    save_shot_reports(output, mission.summary(), shooter.ledger)
                    update_viewer_progress(viewer, mission.summary(), shooter.ammo_remaining)
                    prefix = output/f"target_{decision.target_id}_shot_{len(shooter.ledger):02d}"
                    save_rgb(prefix.with_name(prefix.name+"_rgb.png"), frame.rgb)
                    save_rgb(prefix.with_name(prefix.name+"_annotated.png"),
                             annotate_multi(frame, detections, decision, config["camera"], mission.summary(), shooter.ammo_remaining))
                    if shot.point_world_m is not None:
                        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
                        origin = robot.data.xpos[base_id]+robot.data.xmat[base_id].reshape(3, 3) @ np.array(config["laser"]["origin_base_m"])
                        beams.append({"origin": origin.tolist(), "point": shot.point_world_m.tolist(),
                                      "correct_hit": bool(shot.hit and shot.target_id == decision.target_id)})
                        if viewer is not None and viewer.is_running():
                            with viewer.lock():
                                viewer.user_scn.ngeom = 0
                                draw_beams(viewer.user_scn, beams)
                events.write(json.dumps(sanitize_record(event), default=json_value, ensure_ascii=False, allow_nan=False)+"\n")
                events.flush()
                count += 1
                display = None
                if args.show and last_frame is not None:
                    display = cv2.cvtColor(annotate_multi(last_frame, last_detections, decision,
                        config["camera"], mission.summary(), shooter.ammo_remaining, now-last_frame.timestamp), cv2.COLOR_RGB2BGR)
                windows_open = ui.present(display)
                if mission.done:
                    summary["status"] = mission.status
                    break
                if shot_interrupts or not windows_open:
                    mission.cancel(now)
                    summary["status"] = "CANCELLED"
                    break
                robot.advance(max(0.0, dt-(time.monotonic()-frame_work_start))
                              if hasattr(robot, "current_time") else dt)
                if args.realtime and not hasattr(robot, "current_time"):
                    time.sleep(max(0.0, float(robot.data.time)-(time.monotonic()-mission_start)))
            else:
                mission.finish_timeout(float(robot.data.time))
                summary["status"] = "TIMEOUT"
        robot.Stop()
        if hasattr(robot, "report"):
            summary["official_dynamics"] = robot.report()
            summary["official_dynamics_mission_completed"] = summary["status"] == "SUCCESS"
        summary.update(simulation_time_s=float(robot.data.time), frames=count, delivered_frames=delivered_count,
            states=states, ammo_initial=initial_ammo, ammo_remaining=shooter.ammo_remaining,
            shots=shooter.ledger, shot_count=len(shooter.ledger), last_scores=shooter.last_scores,
            mission=mission.summary(), targets=target_records(mission.summary()), transport=transport.stats,
            credited_scores={str(record["id"]): record["ring"] for record in target_records(mission.summary())},
            total_score=mission.summary()["total_score"],
            latest_target_score_total=mission.summary()["latest_target_score_total"],
            shot_score_sum=mission.summary()["shot_score_sum"],
            credited_score_sums={str(record["id"]): record["score_sum"] for record in target_records(mission.summary())},
            diagnostics=diagnostics, mission_wall_time_s=time.monotonic()-mission_start,
            perception_pose_backend="AprilTag3 per-ID calibration + OpenCV IPPE/LM pose")
        save_shot_reports(output, mission.summary(), shooter.ledger)
        if last_frame is not None:
            save_rgb(output/"final_rgb.png", last_frame.rgb)
            save_rgb(output/"final_annotated.png", annotate_multi(last_frame, last_detections,
                last_decision, config["camera"], mission.summary(), shooter.ammo_remaining))
        overview = mujoco.MjvCamera()
        configure_multi_overview(overview, config)
        robot.renderer.update_scene(robot.data, camera=overview, scene_option=robot.render_option)
        draw_beams(robot.renderer.scene, beams)
        save_rgb(output/"overview.png", robot.renderer.render())
        summary["model_sources_unchanged"] = all(hashlib.sha256((args.model_root/relative).read_bytes()).hexdigest() == digest
            for relative, digest in provenance["source_files_sha256"].items())
        perception.close()
        summary["perception_closed"] = True
        summary["phase"] = "result_ready"
        write_json(output/"summary.json", summary)
        result_saved = True
        print(f"TASK3_RESULT status={summary['status']} summary={output/'summary.json'}", flush=True)
        if args.show or args.viewer:
            print("TASK3_DISPLAY holding multi-target result; motion commands stopped, no further shots", flush=True)
            result_image = None
            if args.show and last_frame is not None:
                result_image = cv2.cvtColor(annotate_multi(last_frame, last_detections, last_decision,
                    config["camera"], mission.summary(), shooter.ammo_remaining), cv2.COLOR_RGB2BGR)
            summary["display_exit_reason"] = ui.hold_result(result_image, args.hold_seconds)
    except KeyboardInterrupt:
        if result_saved:
            summary["display_exit_reason"] = "keyboard_interrupt_after_result"
        elif mission is not None and mission.done:
            summary["status"] = mission.status
            summary["display_exit_reason"] = "keyboard_interrupt_after_terminal_mission"
        else:
            if mission is not None:
                mission.cancel(float(robot.data.time) if robot is not None else 0.0)
            summary["status"] = "CANCELLED"
    except Exception as exc:
        summary["task_status_before_error"] = summary["status"]
        summary["status"], summary["error"] = "ERROR", str(exc)
        (output/"error.log").write_text(traceback.format_exc(), encoding="utf-8")
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
    finally:
        with defer_cleanup_sigint() as interrupts:
            cleanup_errors = []
            for name, resource, method in (("ui", ui, "close"), ("perception", perception, "close"),
                                            ("shooter", shooter, "shooting_shutdown"), ("robot", robot, "close")):
                if resource is not None:
                    try:
                        warnings = getattr(resource, method)()
                        if isinstance(warnings, list):
                            cleanup_errors.extend(f"{name}: {warning}" for warning in warnings)
                    except Exception as cleanup_exc:
                        cleanup_errors.append(f"{name}: {cleanup_exc}")
            if mission is not None:
                summary.update(mission=mission.summary(), targets=target_records(mission.summary()))
                summary["credited_scores"] = {str(record["id"]): record["ring"] for record in target_records(mission.summary())}
                summary["credited_score_sums"] = {str(record["id"]): record["score_sum"] for record in target_records(mission.summary())}
                for field in ("total_score", "latest_target_score_total", "shot_score_sum"):
                    summary[field] = mission.summary()[field]
                write_json(output/"mission.json", mission.summary())
            if shooter is not None:
                summary.update(shots=shooter.ledger, shot_count=len(shooter.ledger),
                               ammo_remaining=shooter.ammo_remaining, last_scores=shooter.last_scores)
                write_json(output/"shots.json", shooter.ledger)
                if mission is not None:
                    save_shot_reports(output, mission.summary(), shooter.ledger)
            if transport is not None:
                summary["transport"] = transport.stats
            if robot is not None and hasattr(robot, "report"):
                summary["official_dynamics"] = robot.report()
            summary.update(cleanup_errors=cleanup_errors, cleanup_sigint_count=len(interrupts),
                           shutdown_completed=not cleanup_errors,
                           phase="finished" if not cleanup_errors else "cleanup_error", wall_time_s=time.monotonic()-wall_start)
            if cleanup_errors:
                summary["task_status_before_cleanup"], summary["status"] = summary["status"], "CLEANUP_ERROR"
            write_json(output/"summary.json", summary)
    code = 0 if summary["status"] == "SUCCESS" and summary.get("model_sources_unchanged") and summary["shutdown_completed"] else 2
    print(f"TASK3_{summary['status']} summary={output/'summary.json'}", flush=True)
    print(f"shots={summary.get('shot_count', 0)} credited_scores={summary.get('credited_scores', {})} "
          f"total_score={summary.get('mission', {}).get('total_score', 0)} "
          f"latest_target_score_total={summary.get('mission', {}).get('latest_target_score_total', 0)} "
          f"ammo_remaining={summary.get('ammo_remaining')}", flush=True)
    return code, output
