"""Offline audit of a completed official-physics run, never a runtime emulator.

Usage: python -m task3_sim.validate_official RUN_DIR
The auditor reads recorded native packets and real staged executable bytes.
It does not execute log content, launch an SDK, or turn injected unit-test
fixtures into native acceptance. Publication time is distinguished from the
unknown-unit raw RobotState time_stamp and from getter/render read time.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import platform
import re

import numpy as np

from .official_runtime import sha256
from .validate_multi import repeat_ledger_audit, round_robin_order


OFFICIAL_MOTION_MODEL = "official_robot_mujoco_ELF_contact_dynamics+official_mc_ctrl+official_HighLevel_SDK"
OFFICIAL_HOLD_PROFILE = "official_rl_balance_policy_pi_v1"
COMMAND_FIELDS = ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s")
HOLD_PROFILE_FILES = {
    "deps/zsibot_sdk/lib/zsl-1/x86_64/libmc_sdk_zsl_1_x86_64.so": None,
    "src/robot_mc/build/export/mc/bin/mc_ctrl": "mc/build/export/mc/bin/mc_ctrl",
    "src/robot_mc/build/export/mc/bin/libbiomimetics.so": "mc/build/export/mc/bin/libbiomimetics.so",
    "src/robot_mc/build/export/onnx_model_crypto/xg/policy_balancestand_withyaw_0423": "mc/build/export/onnx_model_crypto/xg/policy_balancestand_withyaw_0423",
    "src/robot_mc/build/export/onnx_model_crypto/xg/odom_balancestand_withyaw_0423": "mc/build/export/onnx_model_crypto/xg/odom_balancestand_withyaw_0423",
}
PROCESSES = {"official_state_bridge", "robot_mujoco", "mc_ctrl"}
DERIVED_CONFIGS = {"simulate/config.yaml", "mc/build/export/config/xg-user-parameters.yaml",
                   "mc/build/export/config/sdk_config.yaml"}


def _read(path, *, lines=False):
    def reject(value):
        raise ValueError(f"Nonstandard JSON constant {value}")
    with path.open(encoding="utf-8-sig") as source:
        if not lines:
            return json.load(source, parse_constant=reject)
        result = []
        for index, line in enumerate(source):
            if index >= 200000:
                raise ValueError(f"Too many records in {path}")
            if line.strip():
                result.append(json.loads(line, parse_constant=reject))
        return result


def _finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def _vector(value, count):
    try:
        result = np.asarray(value, dtype=float)
        return result.shape == (count,) and bool(np.all(np.isfinite(result)))
    except (ValueError, TypeError):
        return False


def _age(now, timestamp, limit):
    return _finite(now) and _finite(timestamp) and -1e-8 <= now - timestamp <= limit + 1e-8


def _owned_stop_execution(event, execution, sdk_events, sdk_commands):
    """Accept a stale-observation stop only with the actual owned-family zero."""
    vector = [execution.get("applied_command", {}).get(key) for key in COMMAND_FIELDS]
    start, finish = execution.get("started_at_s"), execution.get("finished_at_s")
    if (not _vector(vector, 4) or any(vector) or not _finite(start) or not _finite(finish) or start > finish
            or event.get("shot_record") is not None or event.get("shot") is not None):
        return False
    during = [row for row in sdk_commands if start-1e-8 <= row["time_s"] <= finish+1e-8]
    actions = [row for row in sdk_events if row.get("kind") in ("sdk_result", "sdk_status_unavailable")
               and _finite(row.get("read_at_s")) and start-1e-8 <= row["read_at_s"] <= finish+1e-8]
    if any(row.get("operation") == "standUp" for row in actions) or any(row["active"] for row in during):
        return False
    family = execution.get("session_mode")
    if family in ("move", "attitude"):
        return bool(during) and all(row["mode"] == family and not row["active"] for row in during)
    return family == "standing" and not during


def _path_text(value):
    return str(value).replace("\\", "/").rstrip("/")


def _scalar_config_matches(path, expected):
    """Read only the controlled flat YAML scalar subset, without execution."""
    text = path.read_text(encoding="utf-8-sig")
    for key, value in expected.items():
        matches = re.findall(rf"^[ \t]*{re.escape(key)}[ \t]*:[ \t]*([^\r\n#]+)", text, re.MULTILINE)
        if len(matches) != 1 or matches[0].strip().strip("\"'") != str(value):
            return False
    return True


def _resolve_recorded(value, *, run_dir, original_run, runtime_dir, original_runtime, repo_root):
    """Relocate returned Ubuntu evidence without executing or trusting paths."""
    text = _path_text(value)
    for old, current in ((original_runtime, runtime_dir), (original_run, run_dir)):
        prefix = _path_text(old)
        if prefix and text.startswith(prefix + "/"):
            relative = text[len(prefix) + 1:]
            candidate = (current / relative).resolve()
            if not candidate.is_relative_to(current.resolve()):
                raise ValueError("Recorded path escapes its evidence directory")
            return candidate
    # Source models/controller libraries stay in the local official repository.
    for marker in ("/src/", "/deps/"):
        if marker in text:
            relative = text.rsplit(marker, 1)[1]
            candidate = (repo_root / marker.strip("/") / relative).resolve()
            if not candidate.is_relative_to(repo_root.resolve()):
                raise ValueError("Recorded source path escapes its repository")
            return candidate
    direct = Path(value)
    if direct.is_file():
        return direct
    raise ValueError(f"Cannot resolve recorded file: {value}")


def _sdk_audit(events, config):
    checks, failures = [], []
    family = None
    standing_at = None
    standing_confirmed = False
    last_zero_at = None
    commands = []
    for event in events:
        kind = event.get("kind")
        timestamp = event.get("read_at_s")
        if kind in ("fault", "stop_failed"):
            failures.append(kind)
        if kind == "stationary_confirmed" and event.get("control_mode") == 1 and standing_at is not None:
            standing_confirmed = _finite(timestamp) and timestamp - standing_at >= 4.0 - 1e-8
        if kind not in ("sdk_result", "sdk_status_unavailable"):
            continue
        if kind == "sdk_result" and event.get("code") != 0:
            failures.append("nonzero SDK result")
        if kind == "sdk_status_unavailable" and not config.get("official_dynamics", {}).get("allow_void_status", False):
            failures.append("unapproved void status compatibility")
        operation = event.get("operation")
        if operation == "standUp":
            if family is not None and (last_zero_at is None or not _finite(timestamp) or last_zero_at > timestamp):
                failures.append("standUp without current-mode zero")
            standing_at, standing_confirmed, family = timestamp, False, None
            continue
        if operation not in ("move", "attitudeControl"):
            continue
        arguments = event.get("arguments")
        count = 3 if operation == "move" else 4
        if not _vector(arguments, count) or not _finite(timestamp):
            failures.append("missing finite SDK command arguments/time")
            continue
        values = np.asarray(arguments, float)
        active = bool(np.any(values != 0))
        next_family = "move" if operation == "move" else "attitude"
        if operation == "move":
            legal = all(value == 0 or minimum <= abs(value) <= maximum
                        for value, minimum, maximum in zip(values, (.05, .1, .02), (3, 1, 3)))
        else:
            legal = values[0] == 0 and values[3] == 0 and np.all(np.abs(values) <= .5)
        if not legal:
            failures.append("SDK command outside official range or task axes")
        if active and not standing_confirmed:
            failures.append("motion before four-second standUp and stationary confirmation")
        if family is not None and family != next_family:
            failures.append("move and attitude mixed without standUp transition")
        if active:
            family, last_zero_at = next_family, None
        else:
            last_zero_at = timestamp
        commands.append({"time_s": timestamp, "mode": next_family, "arguments": list(values), "active": active})
    checks = {"sdk_results_and_mode_sequence": bool(commands) and not failures,
              "official_move_commands_recorded": any(c["mode"] == "move" and c["active"] for c in commands),
              "official_attitude_commands_recorded": any(c["mode"] == "attitude" and c["active"] for c in commands)}
    return checks, failures, commands


def _sdk_refresh_audit(events, config, observations, physics):
    """Bind retransmissions to the original image lease and actual SDK call."""
    settings = config.get("official_dynamics", {})
    enabled = settings.get("command_refresh_interval_s") is not None or any(
        row.get("kind", "").startswith("command_lease_") or row.get("kind") == "command_refresh"
        or "refresh" in row for row in events)
    failures, active_calls, refresh_calls = [], 0, 0
    if not enabled:
        return True, failures, {"enabled": False, "active_calls": 0, "refresh_calls": 0}
    ttl = config["control"]["max_frame_age_s"]
    feedback_ttl = config["control"]["max_feedback_age_s"]
    timeout = settings.get("command_timeout_s", 1.0)
    lease, pending, last_call, family = None, None, None, None

    def key(row):
        issued, until = row.get("original_issued_at_s"), row.get("valid_until_s")
        mode = row.get("command_family")
        count = 3 if mode == "move" else 4
        arguments = row.get("arguments")
        if mode not in ("move", "attitude") or not _finite(issued) or not _finite(until) or not _vector(arguments, count):
            return None
        operation = "move" if mode == "move" else "attitudeControl"
        return (mode, issued, until, tuple(arguments)) if row.get("operation") == operation else None

    def require(condition, reason):
        if not condition:
            failures.append(reason)

    for row in events:
        kind, at = row.get("kind"), row.get("read_at_s")
        if kind in ("sdk_result", "sdk_status_unavailable"):
            operation = row.get("operation")
            if operation == "standUp":
                lease, family = None, None
                continue
            if operation not in ("move", "attitudeControl"):
                continue
            arguments = row.get("arguments")
            mode = "move" if operation == "move" else "attitude"
            if not _vector(arguments, 3 if mode == "move" else 4):
                require(False, "invalid actual SDK refresh payload")
                continue
            called = row.get("sdk_called_at_s")
            require(_finite(called) and _finite(at) and called <= at + 1e-8,
                    "missing actual SDK call-start clock")
            active = any(arguments)
            family = mode
            last_call = row
            if not active:
                if lease is not None:
                    require(mode == lease[0] and row.get("safety_zero") is True and row.get("command_family") == lease[0]
                            and row.get("original_issued_at_s") == lease[1] and row.get("valid_until_s") == lease[2],
                            "safety zero lost original lease identity")
                lease = None
                continue
            active_calls += 1
            current = key(row)
            require(current is not None and isinstance(row.get("refresh"), bool), "missing finite SDK lease metadata")
            require(pending is None, "actual SDK command lacks its lease event")
            if current is None or not _finite(called):
                continue
            require(_age(called, current[1], timeout) and called < current[2],
                    "SDK send after original command or image deadline")
            if row.get("refresh") is True:
                refresh_calls += 1
                require(lease == current, "refresh changed original clocks/payload or revived a revoked lease")
                pending = ("command_refresh", current, called)
            else:
                require(lease is None or called < lease[2],
                        "expired original image lease overwritten without an actual zero")
                # Only a newly acquired image decision can create a new lease.
                # The firing gate may reissue the same held input later in the
                # same event, so its referee clock also bounds this association.
                matched = False
                for event in observations:
                    execution = event.get("official_execution", {})
                    command = execution.get("applied_command", {})
                    values = [command.get(name) for name in COMMAND_FIELDS]
                    if not _vector(values, 4):
                        continue
                    expected = values[:3] if mode == "move" else [0, values[3], values[2], 0]
                    start, finish = execution.get("started_at_s"), execution.get("finished_at_s")
                    gate_at = event.get("official_fire_gate", {}).get("referee_time_s")
                    end = max(finish, gate_at) if _finite(finish) and _finite(gate_at) else finish
                    timestamp = event.get("frame_timestamp_s")
                    if (_finite(start) and _finite(end) and _finite(timestamp)
                            and start - 1e-8 <= current[1] <= called <= end + 1e-8
                            and execution.get("requested_mode") == mode
                            and execution.get("observation_invalidated") is not True
                            and event.get("frame_delivered") is True
                            and np.allclose(expected, arguments, atol=1e-10, rtol=0)
                            and abs(current[2] - (timestamp + ttl)) <= 1e-8
                            and _age(called, timestamp, ttl)):
                        matched = True
                        break
                require(matched, "new SDK lease is not bound to a fresh recorded image decision")
                lease = current
                pending = ("command_lease_started", current, called)
        elif kind in ("command_lease_started", "command_refresh"):
            current = key(row)
            require(pending is not None and pending[0] == kind and pending[1] == current
                    and _finite(at) and pending[2] <= at + 1e-8,
                    "lease event does not match the actual SDK call")
            if kind == "command_refresh":
                source = row.get("feedback_source_timestamp_s")
                require(pending is not None and _age(pending[2], source, feedback_ttl)
                        and any(_finite(sample.get("published_at_s")) and _finite(source)
                                and abs(sample["published_at_s"] - source) <= 1e-8 for sample in physics),
                        "refresh lacks fresh recorded native feedback")
            pending = None
        elif kind in ("command_lease_expired", "command_refresh_cancelled"):
            current = key(row)
            require(current is not None and _finite(at), "invalid revoked lease metadata")
            if kind == "command_lease_expired" and current is not None and _finite(at):
                require(at >= current[2], "image lease expired before its recorded deadline")
            # An already-standing session owns no action family and needs no
            # fabricated zero call. Otherwise its latest real call must stop
            # the family it actually owned, not the expired request's family.
            require(family is None or (last_call is not None and not any(last_call.get("arguments", []))
                    and _finite(last_call.get("read_at_s")) and _finite(at)
                    and 0 <= at - last_call["read_at_s"] <= feedback_ttl + 1e-8),
                    "revoked lease lacks an actual current-family zero")
            lease = None
    require(pending is None, "final actual SDK command lacks its lease event")
    return not failures, failures, {"enabled": True, "active_calls": active_calls, "refresh_calls": refresh_calls}


def _hold_profile_audit(native, events, copied, *, repo_root, run_dir, runtime_dir, summary):
    required = any(event.get("shot_record") is not None
                   and event.get("official_fire_gate", {}).get("control_policy") == OFFICIAL_HOLD_PROFILE for event in events)
    if not required:
        return True, {"required": False, "failures": []}
    failures = []
    try:
        definition = _read(repo_root / "task3_sim/config/official_aim_profile.json")
        pins = definition.get("files_sha256", {})
        reported = native.get("aim_profile", {})
        if (definition.get("profile") != OFFICIAL_HOLD_PROFILE or set(pins) != set(HOLD_PROFILE_FILES)
                or reported.get("profile") != OFFICIAL_HOLD_PROFILE or reported.get("verified") is not True
                or reported.get("files_sha256") != pins):
            failures.append("missing or mismatched five-file official hold profile")
        for relative, staged in HOLD_PROFILE_FILES.items():
            source, expected = repo_root / relative, pins.get(relative)
            if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected) or sha256(source) != expected:
                failures.append(f"official hold source hash: {relative}")
            if staged is None:
                continue  # SDK .so is loaded directly from the original deps.
            matches = []
            for row in copied:
                common = dict(run_dir=run_dir, original_run=summary.get("output", ""), runtime_dir=runtime_dir,
                              original_runtime=native.get("runtime", {}).get("workdir", ""), repo_root=repo_root)
                if _resolve_recorded(row["destination"], **common) == (runtime_dir / staged).resolve():
                    matches.append(row)
            if (len(matches) != 1 or sha256(runtime_dir / staged) != expected
                    or matches[0].get("source_sha256") != expected
                    or matches[0].get("staged_initial_sha256") != expected
                    or matches[0].get("staged_final_sha256") != expected):
                failures.append(f"official hold staged hash/manifest: {staged}")
    except (OSError, KeyError, TypeError, ValueError) as exc:
        failures.append(f"{type(exc).__name__}: {exc}")
    return not failures, {"required": True, "failures": failures}


def _held_attitude_fire(event, previous_events, physics, sdk_events, sdk_commands, controls):
    """Narrow exception for a frozen official balance-policy input, not velocity."""
    execution = event.get("official_execution", {})
    servo = execution.get("aim_servo", {})
    gate = event.get("official_fire_gate", {})
    vectors = [[source.get(key) for key in COMMAND_FIELDS] for source in (
        execution.get("applied_command", {}), servo.get("held_command", {}), gate.get("hold_command", {}))]
    failures = []
    def require(condition, name):
        if not condition:
            failures.append(name)
    require(gate.get("control_policy") == servo.get("profile") == OFFICIAL_HOLD_PROFILE, "official_profile")
    require(execution.get("requested_mode") == execution.get("session_mode") == "attitude"
            and gate.get("sdk_ctrlmode") == 21, "same_attitude_mode_21")
    require(servo.get("phase") == "holding" and servo.get("target_id") == event.get("active_target_id")
            == event.get("shot_record", {}).get("intended_target_id") == gate.get("target_id"), "holding_current_target")
    require(servo.get("failure_reason") in (None, "") and servo.get("input_in_bounds") is True, "valid_servo_hold")
    require(gate.get("hold_source") == "bounded official SDK attitude input frozen; actual native feedback stationary", "hold_source")
    require(all(_vector(vector, 4) for vector in vectors), "finite_held_commands")
    if failures:
        return False, failures
    applied = np.asarray(vectors[0], float)
    require(all(np.allclose(applied, vector, atol=1e-10, rtol=0) for vector in vectors[1:]), "held_command_matches_execution_and_gate")
    require(applied[0] == applied[1] == 0
            and abs(applied[2]) <= min(.5, controls["max_yaw_rate_rad_s"])
            and abs(applied[3]) <= min(.5, controls["max_pitch_rate_rad_s"]), "actual_input_within_limits")
    require(_finite(controls.get("aim_tolerance_m")) and 0 < controls["aim_tolerance_m"] <= .008,
            "reviewed_hold_alignment_tolerance")
    fire = gate.get("referee_time_s")
    began, changed, gate_began = servo.get("hold_started_at_s"), servo.get("last_input_change_at_s"), gate.get("hold_started_at_s")
    dwell = controls["stable_duration_s"]
    require(all(_finite(value) for value in (fire, began, changed, gate_began, dwell)) and dwell >= .4,
            "finite_hold_clocks_and_required_dwell")
    if failures:
        return False, failures
    require(abs(began - gate_began) <= 1e-8 and changed <= began <= fire
            and fire - began >= dwell - 1e-8 and fire - changed >= dwell - 1e-8, "unchanged_input_for_full_dwell")
    arguments = [0, applied[3], applied[2], 0]
    commands = [row for row in sdk_commands if began - 1e-8 <= row["time_s"] <= fire + 1e-8]
    preceding = [row for row in sdk_commands if row["time_s"] < began - 1e-8]
    if preceding:
        last = preceding[-1]
        if last["mode"] == "attitude" and np.allclose(last["arguments"], arguments, atol=1e-10, rtol=0):
            commands.insert(0, last)
    require(bool(commands) and all(row["mode"] == "attitude" and np.allclose(row["arguments"], arguments, atol=1e-10, rtol=0)
                                  for row in commands), "unchanged_actual_sdk_command_ledger")
    require(bool(commands) and commands[-1]["time_s"] - commands[0]["time_s"] >= dwell - 1e-8,
            "actual_sdk_hold_duration")
    require(not any(row.get("operation") == "standUp" and row.get("kind") in ("sdk_result", "sdk_status_unavailable")
                    and _finite(row.get("read_at_s")) and began <= row["read_at_s"] <= fire for row in sdk_events),
            "no_mode_reset_during_hold")
    start = fire - dwell
    native_window = [row for row in physics if start - 1e-8 <= row.get("published_at_s", -math.inf) <= fire]
    native_prior = [row for row in physics if _finite(row.get("published_at_s")) and row["published_at_s"] < start - 1e-8]
    if native_prior and (not native_window or native_window[0]["published_at_s"] > start + 1e-8):
        native_window.insert(0, native_prior[-1])
    require(len(native_window) >= 2 and _age(start, native_window[0]["published_at_s"], controls["max_feedback_age_s"])
            and _age(fire, native_window[-1]["published_at_s"], controls["max_feedback_age_s"])
            and all(b["published_at_s"] - a["published_at_s"] <= controls["max_feedback_age_s"] + 1e-8 for a, b in zip(native_window, native_window[1:])),
            "native_feedback_covers_stationary_dwell")
    require(bool(native_window) and all(_vector(row.get("native_packet", {}).get("gyro_body"), 3)
            and _vector(row.get("native_packet", {}).get("velocity_world"), 3)
            and np.linalg.norm(row["native_packet"]["gyro_body"]) <= controls["max_gyro_rad_s"]
            and np.linalg.norm(row["native_packet"]["velocity_world"]) <= controls["max_speed_m_s"] for row in native_window),
            "native_stationary_for_dwell")
    frames = [row for row in previous_events if _finite(row.get("time_s")) and start - 1e-8 <= row["time_s"] <= fire + 1e-8]
    # The dwell boundary will usually fall between 10 Hz image events. Include
    # the last earlier observation rather than requiring an image exactly at
    # the boundary, but prove it still covered the beginning of the dwell.
    earlier = [row for row in previous_events if _finite(row.get("time_s")) and row["time_s"] < start - 1e-8]
    if earlier and (not frames or not _finite(frames[0].get("frame_timestamp_s"))
                    or frames[0]["frame_timestamp_s"] > start + 1e-8):
        frames.insert(0, earlier[-1])
    require(len(frames) >= 2 and all(row.get("active_target_id") == servo["target_id"] and row.get("frame_delivered") is True
            and _age(row["time_s"], row.get("frame_timestamp_s"), controls["max_frame_age_s"])
            and row.get("feedback", {}).get("valid") is True
            and _age(row["time_s"], row["feedback"].get("timestamp"), controls["max_feedback_age_s"])
            and (row.get("geometry") or {}).get("reachable") is True
            and _finite((row.get("geometry") or {}).get("error_norm_m"))
            and row["geometry"]["error_norm_m"] <= controls["aim_tolerance_m"] for row in frames),
            "fresh_aligned_current_target_during_dwell")
    require(len(frames) >= 2 and _age(start, frames[0].get("frame_timestamp_s"), controls["max_frame_age_s"])
            and _age(fire, frames[-1].get("frame_timestamp_s"), controls["max_frame_age_s"])
            and all(0 < b["frame_timestamp_s"] - a["frame_timestamp_s"] <= controls["max_frame_age_s"] + 1e-8
                                  for a, b in zip(frames, frames[1:])), "unique_continuous_dwell_images")
    return not failures, failures


def audit_run(run_dir, *, repo_root=None, min_ring=9, max_actual_miss_m=.02):
    run_dir = Path(run_dir).resolve()
    repo_root = Path(repo_root).resolve() if repo_root is not None else Path(__file__).resolve().parents[1]
    report = {"schema": "task3.official_validation.v1", "run_dir": str(run_dir), "complete": False,
              "all_passed": False, "native_acceptance_passed": False,
              "audit_platform": platform.platform(), "audit_only_no_processes_launched": True,
              "criteria": {"min_ring": min_ring, "max_actual_miss_m": max_actual_miss_m,
                           "min_gait_q_span_rad": .02, "min_gait_qd_rad_s": .05, "min_active_joints": 6},
              "checks": {}, "metrics": {}, "errors": [], "warnings": [],
              "limitations": ["Offline consistency audit of returned evidence, not independent authentication of a host or hardware trial.",
                              "eCAL publication time is source freshness evidence; raw protobuf time_stamp units remain unknown.",
                              "Contact dynamics are executed by the official native process; RGB/ray scoring use its state in a read-only rendering mirror."]}
    try:
        if not isinstance(min_ring, int) or not 1 <= min_ring <= 10 or not _finite(max_actual_miss_m) or max_actual_miss_m <= 0:
            raise ValueError("Invalid precision acceptance criteria")
        summary = _read(run_dir / "summary.json")
        config = _read(run_dir / "config.json")
        runtime_dir = run_dir / "official_runtime"
        status = _read(runtime_dir / "runtime_status.json")
        manifest = _read(runtime_dir / "staging_manifest.json")
        native = summary.get("official_dynamics", {})
        runtime = native.get("runtime", {})
        controls = config["control"]
        checks = report["checks"]
        checks.update({
            "not_a_test_fixture": not any(obj.get(key) is True for obj in (summary, native, manifest)
                                         for key in ("fixture_only", "synthetic_fixture", "test_fixture")),
            "mission_success": summary.get("status") == "SUCCESS" and summary.get("mission", {}).get("status") == "SUCCESS",
            "official_backend_no_kinematic_fallback": config.get("backend") == "official_dynamics"
                and summary.get("motion_model") == OFFICIAL_MOTION_MODEL and native.get("motion_model") == OFFICIAL_MOTION_MODEL
                and summary.get("official_dynamics_mission_completed") is True,
            "reported_linux_cpython310_runtime": str(summary.get("platform", "")).startswith("Linux") and str(summary.get("python", "")).startswith("3.10."),
            "shutdown_completed": summary.get("shutdown_completed") is True and summary.get("phase") == "finished" and summary.get("cleanup_errors") == [],
            "perception_closed": summary.get("perception_closed") is True,
            "model_sources_unchanged": summary.get("model_sources_unchanged") is True and summary.get("official_files_modified") is False,
            "native_fault_log_empty": (run_dir / "fault.log").is_file() and (run_dir / "fault.log").stat().st_size == 0,
            "runtime_preflight_passed": runtime.get("preflight", {}).get("ready") is True and runtime.get("preflight", {}).get("errors") == [],
            "owned_runtime_closed": runtime.get("closed") is True and set(status.get("owned_processes", {})) == PROCESSES
                and status.get("cleanup_errors") == [] and status.get("source_files_unchanged") is True,
        })
        controller_log = runtime_dir / "mc_ctrl.log"
        controller_text = controller_log.read_text(encoding="utf-8", errors="replace") if controller_log.is_file() else ""
        controller_ips = re.findall(r"^[ \t]*client_ip[ \t]*:[ \t]*(\S+)", controller_text, re.MULTILINE | re.IGNORECASE)
        sdk_init_errors = [line for line in controller_text.splitlines()
                           if re.search(r"sdk\s+init\s+failed|bind\s*:\s*cannot\s+assign\s+requested\s+address", line, re.IGNORECASE)]
        preflight = runtime.get("preflight", {})
        checks["controller_actual_log_uses_loopback"] = bool(controller_ips) and set(controller_ips) == {"127.0.0.1"}
        checks["controller_log_has_no_sdk_initialization_failure"] = bool(controller_text) and not sdk_init_errors
        checks["controller_bind_environment_recorded"] = (preflight.get("sdk_controller_bind_ip") == "127.0.0.1"
            and preflight.get("sdk_controller_bind_environment") == "SDK_CLIENT_IP")
        checks["owned_process_startup_completed"] = (runtime.get("all_official_processes_launched") is True
            and "startup_error" in runtime and runtime["startup_error"] is None)
        report["metrics"].update(controller_actual_client_ips=controller_ips,
                                  controller_sdk_initialization_error_lines=sdk_init_errors,
                                  controller_log_sha256=sha256(controller_log) if controller_log.is_file() else None,
                                  controller_bind_environment=preflight.get("sdk_controller_bind_environment"),
                                  controller_bind_ip=preflight.get("sdk_controller_bind_ip"))
        owned = status.get("owned_processes", {})
        shutdown = {}
        for name, row in owned.items():
            code = row.get("exit_code")
            recorded = all(key in row for key in ("exit_code_before_cleanup", "termination_requested", "forced_kill"))
            alive_before_cleanup = recorded and row["exit_code_before_cleanup"] is None
            requested = row.get("termination_requested") is True
            no_forced_kill = row.get("forced_kill") is False
            valid_code = isinstance(code, int) and not isinstance(code, bool)
            controlled = alive_before_cleanup and requested and no_forced_kill
            # The unchanged official controller can call std::terminate while
            # tearing down joinable threads after SIGTERM. This exception is
            # accepted only with explicit before/after cleanup evidence and its
            # known log signature. Keep the -6 visible; never relabel it as 0.
            teardown_abort = (controlled and valid_code and name == "mc_ctrl" and code == -6
                              and "terminate called without an active exception" in controller_text)
            accepted = controlled and valid_code and (code in (0, -15) or teardown_abort)
            if not recorded:
                classification = "missing_cleanup_evidence"
            elif not alive_before_cleanup:
                classification = "exited_before_cleanup"
            elif not no_forced_kill or code == -9:
                classification = "forced_kill"
            elif not requested:
                classification = "no_termination_request"
            elif teardown_abort:
                classification = "controlled_controller_teardown_abort"
            elif accepted:
                classification = "controlled_normal_exit" if code == 0 else "controlled_signal_termination"
            else:
                classification = "unexpected_cleanup_exit"
            shutdown[name] = {"pid": row.get("pid"), "exit_code": code,
                              "exit_code_before_cleanup": row.get("exit_code_before_cleanup"),
                              "termination_requested": row.get("termination_requested"), "forced_kill": row.get("forced_kill"),
                              "classification": classification, "accepted": bool(accepted)}
            if teardown_abort:
                report["warnings"].append(f"Owned mc_ctrl pid={row.get('pid')} exited with SIGABRT (-6) after an explicit termination request. "
                                          "The recorded pre-cleanup process was alive; the known official teardown anomaly remains a nonzero exit.")
        checks["owned_processes_alive_before_cleanup"] = (set(owned) == PROCESSES and all(
            "exit_code_before_cleanup" in row and row["exit_code_before_cleanup"] is None for row in owned.values()))
        checks["owned_processes_have_controlled_termination"] = set(shutdown) == PROCESSES and all(row["accepted"] for row in shutdown.values())
        checks["owned_process_exit_codes_and_identity"] = (len({p.get("pid") for p in owned.values()}) == 3
            and set(runtime.get("processes", {})) == PROCESSES
            and all(isinstance(row.get("pid"), int) and not isinstance(row["pid"], bool) and row["pid"] > 0 and shutdown[name]["accepted"]
                    and runtime["processes"][name].get("pid") == row["pid"]
                    and runtime["processes"][name].get("exit_code") == row["exit_code"] for name, row in owned.items()))
        report["metrics"]["owned_process_shutdown"] = shutdown
        report["metrics"]["cleanup_nonzero_exit_codes"] = {name: row["exit_code"] for name, row in shutdown.items() if row["exit_code"] != 0}

        # Startup failures may precede the first observation and shot ledger.
        # Retain native binding/lifecycle diagnostics before requiring those
        # mission artifacts; absent artifacts still prohibit acceptance.
        events = _read(run_dir / "events.jsonl", lines=True)
        physics = _read(run_dir / "official_physics.jsonl", lines=True)
        shots_file = _read(run_dir / "shots.json")
        mission_file = _read(run_dir / "mission.json")

        source_failures, staged_failures, config_changes, executable_rows = [], [], [], {}
        copied = manifest.get("copies", [])
        for row in copied:
            try:
                common = dict(run_dir=run_dir, original_run=summary.get("output", ""), runtime_dir=runtime_dir,
                              original_runtime=runtime.get("workdir", ""), repo_root=repo_root)
                source = _resolve_recorded(row["source"], **common)
                destination = _resolve_recorded(row["destination"], **common)
                if not destination.is_relative_to(runtime_dir):
                    raise ValueError("Staged destination outside runtime evidence")
                relative = destination.relative_to(runtime_dir).as_posix()
                if sha256(source) != row["source_sha256"] or row["staged_initial_sha256"] != row["source_sha256"]:
                    source_failures.append(str(source))
                # The official controller may update its staged mileage state.
                if destination.name != "mile_data.txt" and sha256(destination) != row["staged_final_sha256"]:
                    staged_failures.append(str(destination))
                if row.get("derived_config_changed"):
                    config_changes.append(relative)
                if destination.name in ("robot_mujoco", "mc_ctrl"):
                    header = destination.read_bytes()[:20]
                    executable_rows[destination.name] = (header[:6] == b"\x7fELF\x02\x01"
                        and len(header) >= 20 and header[18:20] == b"\x3e\x00"
                        and row["source_sha256"] == sha256(destination)
                        and runtime.get("official_executables_sha256", {}).get(destination.name) == row["source_sha256"])
            except (KeyError, ValueError, OSError) as exc:
                source_failures.append(str(exc))
        checks["official_source_hashes_verified"] = bool(copied) and not source_failures and manifest.get("sources_unchanged") is True
        checks["staged_files_match_manifest"] = bool(copied) and not staged_failures
        checks["only_allowed_staged_configs_changed"] = set(config_changes).issubset(DERIVED_CONFIGS)
        checks["unchanged_official_elf_executables"] = set(executable_rows) == {"robot_mujoco", "mc_ctrl"} and all(executable_rows.values())
        checks["staged_official_configs_use_task3_and_loopback"] = all((
            _scalar_config_matches(runtime_dir / "simulate/config.yaml", {
                "robot": "xgb", "robot_scene": "task3.xml", "robot_path": "zsibot_robots",
                "print_scene_information": 0, "enable_elastic_band": 0}),
            _scalar_config_matches(runtime_dir / "mc/build/export/config/xg-user-parameters.yaml", {"motor_platform_type": 5}),
            _scalar_config_matches(runtime_dir / "mc/build/export/config/sdk_config.yaml", {
                "target_ip": "127.0.0.1", "target_port": runtime.get("preflight", {}).get("sdk_client_port", 44888)})))
        report["metrics"].update(source_files_checked=len(copied), source_hash_failures=source_failures, staged_hash_failures=staged_failures)
        profile_valid, profile_evidence = _hold_profile_audit(native, events, copied, repo_root=repo_root,
            run_dir=run_dir, runtime_dir=runtime_dir, summary=summary)
        checks["official_hold_profile_source_and_staged_hashes_verified"] = profile_valid
        report["metrics"]["official_hold_profile_evidence"] = profile_evidence

        ids = [row["id"] for row in config["targets"]]
        mission = summary.get("mission", {})
        targets = mission.get("targets", [])
        shots = summary.get("shots", [])
        credited = mission.get("shots", [])
        fire_events = [e for e in events if e.get("shot_record") is not None]
        initial_ammo = config["simulation"]["ammo_count"]
        repeat = config.get("mission", {}).get("shooting_policy", "single_pass") == "round_robin"
        required_shots = initial_ammo if repeat else len(ids)
        expected_order = round_robin_order(ids, initial_ammo) if repeat else ids
        counts = {identity: sum(row.get("intended_target_id") == identity for row in shots) for identity in ids}
        checks["declared_targets_complete_with_expected_attempts"] = (bool(ids) and len(ids) == len(set(ids)) and len(targets) == len(ids)
            and {t.get("id") for t in targets} == set(ids)
            and all(t.get("state") == "COMPLETED" and t.get("attempts") == counts[t["id"]]
                    and counts[t["id"]] >= 1 for t in targets))
        checks["shot_event_and_ledgers_match"] = (len(shots) == len(credited) == len(fire_events) == required_shots == summary.get("shot_count")
            and shots_file == shots and mission_file == mission and [e["shot_record"] for e in fire_events] == shots)
        checks["unique_correct_target_and_ammo_debit"] = ([s.get("intended_target_id") for s in shots] == expected_order
            and all(s.get("sequence") == index + 1 and s.get("actual_target_id") == s.get("intended_target_id") and s.get("hit") is True
                    and s.get("ammo_before") == initial_ammo - index and s.get("ammo_after") == initial_ammo - index - 1
                    for index, s in enumerate(shots))
            and summary.get("ammo_remaining") == initial_ammo - len(shots) >= 0)
        last_scores = {identity: 0 for identity in ids}
        for shot in credited:
            if shot.get("intended_target_id") in last_scores:
                last_scores[shot["intended_target_id"]] = shot.get("credited_ring", 0)
        checks["score_attribution_and_precision"] = (len(shots) == required_shots and len(credited) == required_shots
            and all(_finite(s.get("ring")) and min_ring <= s["ring"] <= 10
                    and s.get("physical_ring") == s["ring"] and c.get("credited_ring") == s["ring"]
                    and c.get("intended_target_id") == s["intended_target_id"]
                    and c.get("actual_target_id") == s["actual_target_id"] and c.get("correct_target_hit") is True
                    for s, c in zip(shots, credited))
            and mission.get("total_score") == sum(s.get("ring", 0) for s in shots)
            and len(summary.get("diagnostics", [])) == required_shots
            and all(_finite(d.get("actual_miss_m")) and 0 <= d["actual_miss_m"] <= max_actual_miss_m for d in summary.get("diagnostics", [])))
        repeat_checks, repeat_metrics = repeat_ledger_audit(summary, events, config, expected_order)
        checks.update(repeat_checks)
        if repeat_metrics:
            report["metrics"]["repeat_shooting"] = repeat_metrics

        physics_valid, sequence_map, timestamp_map, clock_origins = [], {}, {}, []
        last_sequence = last_clock = -1
        last_publication = -math.inf
        for record in physics:
            packet = record.get("native_packet", {})
            sequence, clock = packet.get("bridge_seq"), packet.get("source_ecal_clock")
            publication, task_time = record.get("published_at_s"), record.get("task_time_s")
            valid = (packet.get("schema") == "task3.official_state.v1"
                and isinstance(sequence, int) and not isinstance(sequence, bool) and sequence > last_sequence
                and isinstance(clock, int) and not isinstance(clock, bool) and clock > last_clock
                and _finite(publication) and publication > last_publication
                and _age(task_time, publication, controls["max_feedback_age_s"])
                and _finite(record.get("publication_age_s")) and abs(record["publication_age_s"] - (task_time - publication)) < 1e-5
                and all(_vector(packet.get(key), count) for key, count in (("q", 12), ("qd", 12), ("tau", 12), ("quat_wxyz", 4),
                    ("gyro_body", 3), ("acc_body", 3), ("imu_position_world", 3), ("velocity_world", 3))))
            if valid:
                valid = abs(float(np.linalg.norm(packet["quat_wxyz"])) - 1) <= .01
                transit_age = (packet.get("received_ecal_time_us", 0) - packet.get("source_ecal_time_us", 0)) * 1e-6
                valid = valid and 0 <= transit_age <= controls["max_feedback_age_s"] and packet.get("source_ecal_time_us", 0) > 0
                received_elapsed = publication + transit_age
                valid = valid and 0 <= task_time - received_elapsed <= controls["max_feedback_age_s"]
                raw_monotonic = packet.get("received_monotonic_ns")
                valid = valid and isinstance(raw_monotonic, int) and not isinstance(raw_monotonic, bool) and raw_monotonic > 0
                if valid:
                    clock_origins.append(raw_monotonic * 1e-9 - received_elapsed)
            physics_valid.append(bool(valid))
            if valid:
                sequence_map[sequence] = record
                timestamp_map[float(publication)] = record
                last_sequence, last_clock, last_publication = sequence, clock, publication
        checks["native_physics_samples_finite_fresh_increasing"] = len(physics) >= 10 and all(physics_valid)
        checks["physics_sample_count_matches_report"] = len(physics) == native.get("physics_samples")
        checks["source_publication_to_monotonic_clock_mapping_consistent"] = bool(clock_origins) and np.ptp(clock_origins) <= 1e-5
        def sample_at(timestamp):
            if not _finite(timestamp):
                return None
            found = timestamp_map.get(float(timestamp))
            if found is None:
                found = next((record for key, record in timestamp_map.items() if abs(key - timestamp) <= 1e-8), None)
            return found
        checks["frames_and_feedback_match_recorded_source_publications"] = bool(events) and all(
            sample_at(e.get("capture_timestamp_s")) is not None
            and (not e.get("frame_delivered") or sample_at(e.get("frame_timestamp_s")) is not None)
            and (not e.get("feedback", {}).get("valid") or sample_at(e["feedback"].get("timestamp")) is not None) for e in events)
        delivered = [e.get("frame_timestamp_s") for e in events if e.get("frame_delivered")]
        checks["delivered_source_frames_strictly_increase"] = bool(delivered) and all(
            _finite(a) and _finite(b) and b > a for a, b in zip(delivered, delivered[1:]))

        sdk_checks, sdk_failures, sdk_commands = _sdk_audit(native.get("sdk_events", []), config)
        checks.update(sdk_checks)
        refresh_valid, refresh_failures, refresh_metrics = _sdk_refresh_audit(native.get("sdk_events", []), config, events, physics)
        checks["sdk_refresh_preserves_original_image_and_command_deadlines"] = refresh_valid
        report["metrics"]["sdk_refresh_evidence"] = {**refresh_metrics, "failures": refresh_failures}
        zero_times = [c["time_s"] for c in sdk_commands if not c["active"]]
        executions, coarse_intervals, execution_valid, fire_valid, hold_audits = [], [], [], [], []
        for index, event in enumerate(events):
            execution = event.get("official_execution", {})
            command = execution.get("applied_command", {})
            vector = [command.get(key) for key in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s")]
            start, finish = execution.get("started_at_s"), execution.get("finished_at_s")
            active = _vector(vector, 4) and any(vector)
            valid = (_vector(vector, 4) and _finite(start) and _finite(finish) and start <= finish
                     and execution.get("requested_mode") in ("move", "attitude", "stop_current_mode"))
            if execution.get("requested_mode") == "stop_current_mode":
                valid = valid and _owned_stop_execution(event, execution, native.get("sdk_events", []), sdk_commands)
            if active:
                valid = valid and event.get("frame_delivered") is True and event.get("feedback", {}).get("valid") is True
                valid = valid and _age(finish, event.get("frame_timestamp_s"), controls["max_frame_age_s"])
                valid = valid and _age(finish, event.get("feedback", {}).get("timestamp"), controls["max_feedback_age_s"])
                valid = valid and execution.get("observation_invalidated") is not True
            if execution.get("observation_invalidated"):
                valid = valid and not active and event.get("shot_record") is None
            execution_valid.append(bool(valid))
            executions.append(execution)
            if active and execution.get("requested_mode") == "move":
                end = events[index + 1].get("official_execution", {}).get("started_at_s", finish + .1) if index + 1 < len(events) else finish + .1
                if _finite(end):
                    # A blocking transition can leave a long gap between
                    # image events. SDK zero ends the coarse-motion interval;
                    # standing/attitude leg adjustment is not gait evidence.
                    end = min([end] + [stop for stop in zero_times if stop >= start])
                    coarse_intervals.append((start, end))
            if event.get("shot_record") is None:
                continue
            gate = event.get("official_fire_gate", {})
            source = sequence_map.get(gate.get("referee_source_sequence"))
            feedback = event.get("feedback", {})
            geometry = event.get("geometry") or {}
            hold_valid, hold_failures = (False, [])
            if active:
                hold_valid, hold_failures = _held_attitude_fire(event, events[:index + 1], physics,
                    native.get("sdk_events", []), sdk_commands, controls)
            hold_audits.append({"target_id": event["shot_record"].get("intended_target_id"),
                               "nonzero_input": bool(active), "held_attitude_exception": bool(hold_valid),
                               "hold_failed_checks": hold_failures})
            valid = (gate.get("allowed", False) is True and source is not None
                and _age(gate.get("referee_time_s"), gate.get("referee_published_at_s"), controls["max_feedback_age_s"])
                and _age(gate.get("referee_time_s"), event.get("frame_timestamp_s"), controls["max_frame_age_s"])
                and feedback.get("valid") is True and _age(gate.get("referee_time_s"), feedback.get("timestamp"), controls["max_feedback_age_s"])
                and _vector(feedback.get("gyro_rad_s"), 3) and np.linalg.norm(feedback["gyro_rad_s"]) <= controls["max_gyro_rad_s"]
                and _vector(feedback.get("velocity_m_s"), 3) and np.linalg.norm(feedback["velocity_m_s"]) <= controls["max_speed_m_s"]
                and event.get("state") == "READY" and geometry.get("reachable") is True
                and _finite(geometry.get("error_norm_m")) and geometry["error_norm_m"] <= controls["aim_tolerance_m"]
                and (not active or hold_valid) and execution.get("session_mode") in ("standing", "attitude")
                and event.get("shot_accepted") is True
                and any(d.get("tag_id") == event["shot_record"]["intended_target_id"] and d.get("hamming") == 0
                        and _finite(d.get("timestamp")) and abs(d["timestamp"] - event["frame_timestamp_s"]) <= 1e-8
                        and d.get("decision_margin", -math.inf) >= config["tag"]["min_decision_margin"]
                        and _finite(d.get("reprojection_error_px")) and d["reprojection_error_px"] <= config["tag"]["max_reprojection_error_px"]
                        for d in event.get("detections", [])))
            if source is not None:
                packet = source["native_packet"]
                valid = valid and abs(source["published_at_s"] - gate.get("referee_published_at_s", math.inf)) <= 1e-8
                valid = valid and np.linalg.norm(packet["gyro_body"]) <= controls["max_gyro_rad_s"]
                valid = valid and np.linalg.norm(packet["velocity_world"]) <= controls["max_speed_m_s"]
            fire_valid.append(bool(valid))
        checks["official_execution_fresh_and_transitions_stop"] = bool(execution_valid) and all(execution_valid)
        checks["all_shots_have_fresh_stationary_native_gate"] = len(fire_valid) == required_shots and all(fire_valid)
        report["metrics"]["shot_hold_evidence"] = hold_audits
        moving = [record for record in physics if any(start <= record.get("task_time_s", -math.inf) <= end for start, end in coarse_intervals)]
        q_span = np.ptp(np.array([r["native_packet"]["q"] for r in moving]), axis=0) if moving else np.zeros(12)
        qd_peak = np.max(np.abs(np.array([r["native_packet"]["qd"] for r in moving])), axis=0) if moving else np.zeros(12)
        yaw = []
        for record in moving:
            w, x, y, z = record["native_packet"]["quat_wxyz"]
            yaw.append(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))
        yaw_travel = sum(abs(math.atan2(math.sin(b-a), math.cos(b-a))) for a, b in zip(yaw, yaw[1:]))
        checks["coarse_turns_have_official_leg_motion"] = (len(moving) >= 10 and int(np.sum(q_span > .02)) >= 6
            and int(np.sum(qd_peak > .05)) >= 6 and yaw_travel > .25)
        sdk_matching = []
        for execution in executions:
            command = execution.get("applied_command", {})
            values = [command.get(key) for key in ("vx_m_s", "vy_m_s", "yaw_rate_rad_s", "pitch_rate_rad_s")]
            if not _vector(values, 4) or not any(values):
                continue
            arguments = values[:3] if execution.get("requested_mode") == "move" else [0, values[3], values[2], 0]
            sdk_matching.append(any(c["active"] and c["mode"] == execution.get("requested_mode")
                and execution["started_at_s"] - 1e-8 <= c["time_s"] <= execution["finished_at_s"] + 1e-8
                and np.allclose(arguments, c["arguments"], atol=1e-10, rtol=1e-7) for c in sdk_commands))
        checks["applied_execution_matches_actual_sdk_call_ledger"] = bool(sdk_matching) and all(sdk_matching)
        report["metrics"].update(native_physics_samples=len(physics), coarse_motion_samples=len(moving),
                                  coarse_joint_q_span_rad=q_span.tolist(), coarse_joint_qd_peak_rad_s=qd_peak.tolist(),
                                  coarse_body_yaw_travel_rad=yaw_travel, sdk_sequence_errors=sdk_failures,
                                  shot_count=len(shots), scores={str(key): value for key, value in last_scores.items()},
                                  per_shot_scores=[{"sequence": s.get("sequence"), "target_id": s.get("intended_target_id"),
                                                    "ring": s.get("ring")} for s in shots],
                                  total_score=mission.get("total_score"),
                                  latest_target_score_total=mission.get("latest_target_score_total"),
                                  cumulative_scores_by_target={str(row.get("id")): row.get("score_sum") for row in targets},
                                  package_version=summary.get("package_version"))
        terminal = run_dir / "terminal.log"
        checks["actual_cli_exit_zero_recorded"] = terminal.is_file() and "TASK3_PROCESS_EXIT code=0" in terminal.read_text(encoding="utf-8", errors="replace")
        report["complete"] = True
        report["all_passed"] = all(checks.values())
        report["native_acceptance_passed"] = report["all_passed"]
    except (OSError, KeyError, TypeError, ValueError, OverflowError) as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    # NumPy comparisons can yield bool_ even inside Python ``and`` clauses.
    # Keep the audit JSON portable and strict on every return path.
    report["checks"] = {key: bool(value) for key, value in report["checks"].items()}
    report["failed_checks"] = [key for key, value in report["checks"].items() if not value]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--min-ring", type=int, default=9)
    parser.add_argument("--max-actual-miss-m", type=float, default=.02)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = audit_run(args.run_dir, repo_root=args.repo_root, min_ring=args.min_ring,
                       max_actual_miss_m=args.max_actual_miss_m)
    output = args.output or args.run_dir / "official_validation.json"
    try:
        output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    except OSError as exc:
        result["errors"].append(f"Cannot write audit report: {exc}")
        result["all_passed"] = result["native_acceptance_passed"] = False
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result["all_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
