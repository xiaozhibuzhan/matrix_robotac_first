"""Audit-schema tests with explicit fixtures, never native acceptance runs."""

from __future__ import annotations

import contextlib
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from task3_sim.official_runtime import sha256
from task3_sim.validate_official import HOLD_PROFILE_FILES, OFFICIAL_HOLD_PROFILE, OFFICIAL_MOTION_MODEL, _owned_stop_execution, _sdk_refresh_audit, audit_run, main


class OwnedStopAuditTests(unittest.TestCase):
    def fixture(self, family="attitude"):
        event = {"state": "WAIT"}
        execution = {"requested_mode": "stop_current_mode", "applied_command": {
            "vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": 0, "pitch_rate_rad_s": 0},
            "started_at_s": 1., "finished_at_s": 1.01, "session_mode": family}
        commands = [{"time_s": 1.005, "mode": family, "active": False}]
        return event, execution, [], commands

    def test_owned_family_zero_is_valid_even_without_motion_observation(self):
        for family in ("move", "attitude"):
            self.assertTrue(_owned_stop_execution(*self.fixture(family)))
        event, execution, sdk, _ = self.fixture("standing")
        self.assertTrue(_owned_stop_execution(event, execution, sdk, []))

    def test_stop_cannot_disguise_nonzero_command_shot_wrong_family_or_missing_real_zero(self):
        for change in ("nonzero", "shot", "wrong_family", "missing_zero", "stand_up"):
            with self.subTest(change=change):
                event, execution, sdk, commands = self.fixture()
                if change == "nonzero":
                    execution["applied_command"]["yaw_rate_rad_s"] = .1
                elif change == "shot":
                    event["shot_record"] = {}
                elif change == "wrong_family":
                    commands[0]["mode"] = "move"
                elif change == "missing_zero":
                    commands.clear()
                else:
                    sdk.append({"kind": "sdk_result", "operation": "standUp", "read_at_s": 1.006})
                self.assertFalse(_owned_stop_execution(event, execution, sdk, commands))

    def test_standing_stop_cannot_issue_new_family_zero(self):
        event, execution, sdk, commands = self.fixture("standing")
        commands[0]["mode"] = "move"
        self.assertFalse(_owned_stop_execution(event, execution, sdk, commands))


class SDKRefreshAuditTests(unittest.TestCase):
    def setUp(self):
        self.config = {"control": {"max_frame_age_s": .15, "max_feedback_age_s": .15},
                       "official_dynamics": {"command_timeout_s": 1., "command_refresh_interval_s": .04}}
        self.command = {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": .1, "pitch_rate_rad_s": 0}
        self.context = {"command_family": "move", "operation": "move", "arguments": [0, 0, .1],
                        "original_issued_at_s": 10., "valid_until_s": 10.149}
        self.events = [self.call(10.0005, refresh=False), self.marker(10.001, "command_lease_started"),
                       self.call(10.04, refresh=True), self.marker(10.041, "command_refresh", feedback_source_timestamp_s=10.039)]
        self.observations = [self.observation(10.)]
        self.physics = [{"published_at_s": 10.039}]

    def observation(self, timestamp):
        return {"frame_delivered": True, "frame_timestamp_s": timestamp - .001,
                "official_execution": {"applied_command": self.command, "requested_mode": "move",
                    "started_at_s": timestamp, "finished_at_s": timestamp + .002, "observation_invalidated": False}}

    def call(self, timestamp, **fields):
        return {**self.context, "kind": "sdk_result", "code": 0, "sdk_called_at_s": timestamp,
                "read_at_s": timestamp + .0001, **fields}

    def marker(self, timestamp, kind, **fields):
        return {**self.context, "kind": kind, "read_at_s": timestamp, **fields}

    def audit(self):
        return _sdk_refresh_audit(self.events, self.config, self.observations, self.physics)

    def test_same_payload_retransmission_preserves_original_image_authority(self):
        valid, failures, metrics = self.audit()
        self.assertTrue(valid, failures)
        self.assertEqual(metrics["refresh_calls"], 1)

    def test_refresh_cannot_extend_either_original_clock_or_change_payload(self):
        for field, value in (("original_issued_at_s", 10.04), ("valid_until_s", 10.3), ("arguments", [0, 0, .2])):
            with self.subTest(field=field):
                old = self.events[2][field]
                self.events[2][field] = value
                self.assertFalse(self.audit()[0])
                self.events[2][field] = old

    def test_sdk_call_start_must_precede_original_image_deadline(self):
        self.events[2].update(sdk_called_at_s=10.149, read_at_s=10.15)
        self.events[3]["read_at_s"] = 10.151
        self.assertFalse(self.audit()[0])

    def test_sdk_call_return_after_deadline_does_not_rewrite_call_start(self):
        self.events[2].update(sdk_called_at_s=10.14, read_at_s=10.16)
        self.events[3]["read_at_s"] = 10.161
        self.assertTrue(self.audit()[0], self.audit()[1])

    def test_image_deadline_does_not_extend_the_original_motion_watchdog(self):
        self.config["control"]["max_frame_age_s"] = 2.
        for row in self.events:
            row["valid_until_s"] = 11.999
        self.events[2].update(sdk_called_at_s=11.01, read_at_s=11.011)
        self.events[3].update(read_at_s=11.012, feedback_source_timestamp_s=11.009)
        self.physics.append({"published_at_s": 11.009})
        self.assertFalse(self.audit()[0])

    def test_new_lease_cannot_restamp_an_old_image(self):
        self.events[2].update(refresh=False, original_issued_at_s=10.04, valid_until_s=10.189)
        self.events[3].update(kind="command_lease_started", original_issued_at_s=10.04, valid_until_s=10.189)
        self.assertFalse(self.audit()[0])

    def test_refresh_needs_both_real_sdk_result_and_fresh_native_source(self):
        self.events[3]["feedback_source_timestamp_s"] = 9.8
        self.physics.append({"published_at_s": 9.8})
        self.assertFalse(self.audit()[0])
        self.events[3]["feedback_source_timestamp_s"] = 10.039
        self.events.pop(2)
        self.assertFalse(self.audit()[0])

    def test_expiry_requires_current_family_zero_and_old_lease_cannot_resume(self):
        zero = self.call(10.15, refresh=False, safety_zero=True, arguments=[0, 0, 0])
        expired = self.marker(10.151, "command_lease_expired")
        self.events += [zero, expired]
        self.assertTrue(self.audit()[0], self.audit()[1])
        self.events += [self.call(10.16, refresh=True), self.marker(10.161, "command_refresh", feedback_source_timestamp_s=10.039)]
        self.assertFalse(self.audit()[0])
        self.events = self.events[:-4] + [expired]
        self.assertFalse(self.audit()[0])

    def test_fresh_image_can_resume_after_non_latching_expiry(self):
        self.events += [self.call(10.15, refresh=False, safety_zero=True, arguments=[0, 0, 0]),
                        self.marker(10.151, "command_lease_expired")]
        self.observations.append(self.observation(10.2))
        self.events += [self.call(10.2005, refresh=False, original_issued_at_s=10.2, valid_until_s=10.349),
                        self.marker(10.201, "command_lease_started", original_issued_at_s=10.2, valid_until_s=10.349)]
        self.assertTrue(self.audit()[0], self.audit()[1])

    def test_new_image_cannot_hide_an_old_lease_that_expired_without_zero(self):
        self.observations.append(self.observation(10.2))
        self.events += [self.call(10.2005, refresh=False, original_issued_at_s=10.2, valid_until_s=10.349),
                        self.marker(10.201, "command_lease_started", original_issued_at_s=10.2, valid_until_s=10.349)]
        valid, failures, _ = self.audit()
        self.assertFalse(valid)
        self.assertIn("expired original image lease overwritten without an actual zero", failures)

    def test_feedback_cancellation_stops_current_family_and_cannot_resume_old_lease(self):
        self.events += [self.call(10.1, refresh=False, safety_zero=True, arguments=[0, 0, 0]),
                        self.marker(10.101, "command_refresh_cancelled", reason="fresh feedback unavailable")]
        self.assertTrue(self.audit()[0], self.audit()[1])
        self.events += [self.call(10.12, refresh=True), self.marker(10.121, "command_refresh", feedback_source_timestamp_s=10.039)]
        self.assertFalse(self.audit()[0])

    def test_standing_expired_request_needs_no_fabricated_zero(self):
        self.events = [self.marker(10.3, "command_lease_expired")]
        self.assertTrue(self.audit()[0], self.audit()[1])

    def test_enabled_refresh_cannot_omit_call_clocks_or_original_image_lease(self):
        for field in ("sdk_called_at_s", "original_issued_at_s", "valid_until_s", "refresh"):
            with self.subTest(field=field):
                old = self.events[0].pop(field)
                self.assertFalse(self.audit()[0])
                self.events[0][field] = old


class OfficialAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name).resolve()
        self.run = self.repo / "task3_sim/runs/explicit_contract_fixture"
        self.runtime = self.run / "official_runtime"
        self.runtime.mkdir(parents=True)
        self.config = {"backend": "official_dynamics", "simulation": {"ammo_count": 5},
            "targets": [{"id": identity} for identity in (2, 5, 9)],
            "control": {"max_feedback_age_s": .15, "max_frame_age_s": .15, "max_gyro_rad_s": .02,
                        "max_speed_m_s": .01, "aim_tolerance_m": .008, "stable_duration_s": .4,
                        "max_yaw_rate_rad_s": .35, "max_pitch_rate_rad_s": .18},
            "tag": {"min_decision_margin": 40, "max_reprojection_error_px": 2}}
        elf = b"\x7fELF\x02\x01" + bytes(12) + b"\x3e\x00" + b"EXPLICIT SYNTHETIC CONTRACT FIXTURE"
        paths = [
            ("src/robot_mujoco/simulate/build/robot_mujoco", "simulate/build/robot_mujoco", elf, elf),
            ("src/robot_mc/build/export/mc/bin/mc_ctrl", "mc/build/export/mc/bin/mc_ctrl", elf, elf),
            ("src/robot_mujoco/simulate/config.yaml", "simulate/config.yaml", b"scene: old",
                b"robot: xgb\nrobot_scene: task3.xml\nrobot_path: zsibot_robots\nprint_scene_information: 0\nenable_elastic_band: 0\n"),
            ("src/robot_mc/build/export/config/xg-user-parameters.yaml", "mc/build/export/config/xg-user-parameters.yaml", b"motor_platform_type: 8", b"motor_platform_type: 5"),
            ("src/robot_mc/build/export/config/sdk_config.yaml", "mc/build/export/config/sdk_config.yaml", b"target_port: 43988",
                b"target_ip: 127.0.0.1\ntarget_port: 44888\n"),
        ]
        copies = []
        for source_name, dest_name, original, staged in paths:
            source, destination = self.repo / source_name, self.runtime / dest_name
            source.parent.mkdir(parents=True, exist_ok=True)
            destination.parent.mkdir(parents=True, exist_ok=True)
            source.write_bytes(original)
            destination.write_bytes(staged)
            copies.append({"source": str(source), "destination": str(destination), "source_sha256": sha256(source),
                "staged_initial_sha256": sha256(source), "staged_final_sha256": sha256(destination),
                "derived_config_changed": original != staged})
        self.manifest = {"fixture_only": True, "copies": copies, "sources_unchanged": True}
        processes = {name: {"pid": index + 1000, "exit_code": 0 if name == "official_state_bridge" else -15,
                            "exit_code_before_cleanup": None, "termination_requested": True, "forced_kill": False}
                     for index, name in enumerate(("official_state_bridge", "robot_mujoco", "mc_ctrl"))}
        self.controller_log_text = "EXPLICIT CONTRACT FIXTURE\nclient_ip: 127.0.0.1\nInitialized FSM state: PASSIVE\n"
        self.runtime_status = {"owned_processes": processes, "cleanup_errors": [], "source_files_unchanged": True}
        sdk_events = [{"kind": "sdk_result", "operation": "standUp", "arguments": [], "code": 0, "read_at_s": .5},
                      {"kind": "stationary_confirmed", "control_mode": 1, "read_at_s": 4.5}]
        self.physics, self.events, shots, credits, diagnostics = [], [], [], [], []
        times = [5 + index * .05 for index in range(17)] + [10.1, 10.15, 10.2]
        for index, timestamp in enumerate(times):
            coarse = index < 17
            publication = timestamp - .001
            yaw = .05 * min(index, 16)
            packet = {"schema": "task3.official_state.v1", "bridge_seq": index + 1, "source_ecal_clock": index + 1,
                "source_ecal_time_us": int(round((1000 + publication) * 1e6)),
                "received_ecal_time_us": int(round((1000 + timestamp) * 1e6)),
                "received_monotonic_ns": int(round((2000 + timestamp) * 1e9)), "source_time_stamp_raw": 0,
                "q": [.1 * math.sin(index + joint) for joint in range(12)],
                "qd": [math.cos(index + joint) if coarse else 0 for joint in range(12)], "tau": [0] * 12,
                "quat_wxyz": [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
                "gyro_body": [0, 0, .2 if coarse else 0], "acc_body": [0, 0, 9.81],
                "imu_position_world": [0, 0, .32], "velocity_world": [0, 0, 0]}
            self.physics.append({"task_time_s": timestamp, "published_at_s": publication,
                                 "publication_age_s": .001, "native_packet": packet})
            command = {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": .2 if coarse else 0, "pitch_rate_rad_s": 0}
            event = {"time_s": timestamp, "capture_timestamp_s": publication, "frame_timestamp_s": publication,
                "frame_delivered": True, "state": "SEARCHING" if coarse else "READY",
                "feedback": {"valid": True, "timestamp": publication, "gyro_rad_s": packet["gyro_body"], "velocity_m_s": [0, 0, 0]},
                "official_execution": {"requested_mode": "move" if coarse else "attitude", "applied_command": command,
                    "started_at_s": timestamp, "finished_at_s": timestamp + .001,
                    "session_mode": "move" if coarse else "attitude", "observation_invalidated": False}}
            if coarse:
                sdk_events.append({"kind": "sdk_result", "operation": "move", "arguments": [0, 0, .2], "code": 0, "read_at_s": timestamp + .0005})
            else:
                shot_index = index - 16
                identity = (2, 5, 9)[shot_index - 1]
                shot = {"sequence": shot_index, "intended_target_id": identity, "actual_target_id": identity,
                    "hit": True, "ring": 10, "physical_ring": 10, "ammo_before": 6 - shot_index, "ammo_after": 5 - shot_index}
                shots.append(shot)
                credits.append({"intended_target_id": identity, "actual_target_id": identity, "ring": 10,
                                "credited_ring": 10, "correct_target_hit": True})
                diagnostics.append({"actual_miss_m": .006})
                event.update(shot_record=shot, shot_accepted=True,
                    geometry={"reachable": True, "error_norm_m": .004},
                    detections=[{"tag_id": identity, "timestamp": publication, "hamming": 0,
                                 "decision_margin": 100, "reprojection_error_px": .1}],
                    official_fire_gate={"allowed": True, "referee_source_sequence": index + 1,
                        "referee_published_at_s": publication, "referee_time_s": timestamp + .002})
            self.events.append(event)
        sdk_events += [
            {"kind": "sdk_result", "operation": "move", "arguments": [0, 0, 0], "code": 0, "read_at_s": 5.84},
            {"kind": "sdk_result", "operation": "standUp", "arguments": [], "code": 0, "read_at_s": 5.85},
            {"kind": "stationary_confirmed", "control_mode": 1, "read_at_s": 9.85},
            {"kind": "sdk_result", "operation": "attitudeControl", "arguments": [0, .01, .001, 0], "code": 0, "read_at_s": 9.9},
            {"kind": "sdk_result", "operation": "attitudeControl", "arguments": [0, 0, 0, 0], "code": 0, "read_at_s": 10.0}]
        sdk_events.sort(key=lambda row: row["read_at_s"])
        mission = {"status": "SUCCESS", "total_score": 30, "shots": credits,
                   "targets": [{"id": identity, "state": "COMPLETED", "attempts": 1} for identity in (2, 5, 9)]}
        self.summary = {"fixture_only": True, "status": "SUCCESS", "mission": mission, "shots": shots, "shot_count": 3,
            "ammo_remaining": 2, "output": str(self.run), "platform": "Linux-fixture-only", "python": "3.10.12",
            "motion_model": OFFICIAL_MOTION_MODEL, "official_dynamics_mission_completed": True,
            "shutdown_completed": True, "phase": "finished", "cleanup_errors": [], "perception_closed": True,
            "model_sources_unchanged": True, "official_files_modified": False, "diagnostics": diagnostics,
            "official_dynamics": {"fixture_only": True, "motion_model": OFFICIAL_MOTION_MODEL,
                "physics_samples": len(self.physics), "sdk_events": sdk_events,
                "runtime": {"workdir": str(self.runtime), "closed": True,
                    "all_official_processes_launched": True, "startup_error": None,
                    "preflight": {"ready": True, "errors": [], "sdk_controller_bind_ip": "127.0.0.1",
                                  "sdk_controller_bind_environment": "SDK_CLIENT_IP"}, "processes": processes,
                    "official_executables_sha256": {"robot_mujoco": sha256(self.runtime / "simulate/build/robot_mujoco"),
                                                    "mc_ctrl": sha256(self.runtime / "mc/build/export/mc/bin/mc_ctrl")}}}}
        self.save()

    def save(self):
        for path, value in ((self.run / "summary.json", self.summary), (self.run / "config.json", self.config),
                            (self.run / "shots.json", self.summary["shots"]), (self.run / "mission.json", self.summary["mission"]),
                            (self.runtime / "runtime_status.json", self.runtime_status),
                            (self.runtime / "staging_manifest.json", self.manifest)):
            path.write_text(json.dumps(value), encoding="utf-8")
        for name, rows in (("events.jsonl", self.events), ("official_physics.jsonl", self.physics)):
            (self.run / name).write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
        (self.run / "fault.log").write_bytes(b"")
        (self.run / "terminal.log").write_text("EXPLICIT CONTRACT FIXTURE\nTASK3_PROCESS_EXIT code=0\n", encoding="utf-8")
        (self.runtime / "mc_ctrl.log").write_text(self.controller_log_text, encoding="utf-8")

    def audit(self):
        return audit_run(self.run, repo_root=self.repo)

    def held_fixture(self, *, repeat_ammo=None):
        """Three explicit frozen-input contracts; still never native evidence."""
        # Synthetic bytes and synthetic pins exercise the profile contract.
        # fixture_only remains true at every evidence level; these files can
        # never establish native acceptance or official policy behavior.
        pins = {}
        for relative, staged in HOLD_PROFILE_FILES.items():
            source = self.repo / relative
            if not source.exists():
                source.parent.mkdir(parents=True, exist_ok=True)
                source.write_bytes(b"EXPLICIT SYNTHETIC PROFILE FIXTURE " + relative.encode())
                if staged is not None:
                    destination = self.runtime / staged
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(source.read_bytes())
                    digest = sha256(source)
                    self.manifest["copies"].append({"source": str(source), "destination": str(destination),
                        "source_sha256": digest, "staged_initial_sha256": digest,
                        "staged_final_sha256": digest, "derived_config_changed": False})
            pins[relative] = sha256(source)
        definition = {"profile": OFFICIAL_HOLD_PROFILE, "files_sha256": pins}
        profile_path = self.repo / "task3_sim/config/official_aim_profile.json"
        profile_path.parent.mkdir(parents=True, exist_ok=True)
        profile_path.write_text(json.dumps(definition), encoding="utf-8")
        self.summary["official_dynamics"]["aim_profile"] = {**definition, "verified": True}
        original = self.events[17:]
        if repeat_ammo is not None:
            from task3_sim.tests.test_repeat_acceptance import repeated_evidence
            repeat_summary, _, _, _ = repeated_evidence(repeat_ammo)
            self.config["mission"] = {"shooting_policy": "round_robin"}
            self.config["simulation"]["ammo_count"] = repeat_ammo
            self.summary["mission"] = repeat_summary["mission"]
            self.summary["shot_count"] = repeat_ammo
            self.summary["ammo_remaining"] = 0
            templates = {event["shot_record"]["intended_target_id"]: event for event in original}
            original, shots = [], []
            for index, row in enumerate(self.summary["mission"]["shots"]):
                timestamp = 11.0 + index
                row["timestamp"] = timestamp
                template = copy.deepcopy(templates[row["intended_target_id"]])
                template["round_index"] = row["round_index"]
                raw = template["shot_record"]
                raw.update(sequence=index+1, ring=row["ring"], physical_ring=row["ring"],
                           ammo_before=repeat_ammo-index, ammo_after=repeat_ammo-index-1)
                template["mission_shot_record"] = copy.deepcopy(row)
                shots.append(raw)
                original.append(template)
            self.summary["shots"] = shots
            self.summary["diagnostics"] = [{"actual_miss_m": .006} for _ in shots]
            for record in self.summary["mission"]["targets"]:
                for visit in record["visits"]:
                    timestamp = 10.0+visit["shot_index"]
                    visit.update(selected_at_s=timestamp-.6, finished_at_s=timestamp)
        self.events = self.events[:17]
        self.physics = self.physics[:17]
        sdk = self.summary["official_dynamics"]["sdk_events"]
        for target_index, shot_event in enumerate(original):
            fire_at = 11.0 + target_index
            began = fire_at - .6
            identity = shot_event["shot_record"]["intended_target_id"]
            command = {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": .03 + target_index * .01, "pitch_rate_rad_s": -.02}
            for offset in range(7):
                timestamp = began + offset * .1
                event = copy.deepcopy(shot_event)
                is_shot = offset == 6
                if not is_shot:
                    for key in ("shot_record", "mission_shot_record", "shot_accepted", "official_fire_gate"):
                        event.pop(key, None)
                publication = timestamp - .001
                event.update(time_s=timestamp, capture_timestamp_s=publication, frame_timestamp_s=publication,
                             active_target_id=identity, state="READY" if is_shot else "SETTLING")
                event["feedback"]["timestamp"] = publication
                event["detections"][0]["timestamp"] = publication
                execution = event["official_execution"]
                execution.update(applied_command=dict(command), started_at_s=timestamp, finished_at_s=timestamp + .001,
                                 aim_servo={"phase": "holding", "target_id": identity, "hold_started_at_s": began,
                                            "last_input_change_at_s": began, "profile": OFFICIAL_HOLD_PROFILE,
                                            "held_command": dict(command), "input_in_bounds": True, "failure_reason": None})
                packet = copy.deepcopy(self.physics[-1]["native_packet"])
                sequence = len(self.physics) + 1
                packet.update(bridge_seq=sequence, source_ecal_clock=sequence,
                    source_ecal_time_us=int(round((1000 + publication) * 1e6)),
                    received_ecal_time_us=int(round((1000 + timestamp) * 1e6)),
                    received_monotonic_ns=int(round((2000 + timestamp) * 1e9)),
                    gyro_body=[0, 0, 0], velocity_world=[0, 0, 0], qd=[0] * 12)
                self.physics.append({"task_time_s": timestamp, "published_at_s": publication,
                                     "publication_age_s": .001, "native_packet": packet})
                if is_shot:
                    event["official_fire_gate"].update(referee_source_sequence=sequence, referee_published_at_s=publication,
                        referee_time_s=timestamp + .002, control_policy=OFFICIAL_HOLD_PROFILE, target_id=identity, hold_command=dict(command),
                        hold_source="bounded official SDK attitude input frozen; actual native feedback stationary",
                        hold_started_at_s=began, sdk_ctrlmode=21)
                self.events.append(event)
                sdk.append({"kind": "sdk_result", "operation": "attitudeControl", "code": 0,
                            "arguments": [0, command["pitch_rate_rad_s"], command["yaw_rate_rad_s"], 0], "read_at_s": timestamp + .0005})
        sdk.sort(key=lambda row: row["read_at_s"])
        self.summary["official_dynamics"]["physics_samples"] = len(self.physics)
        self.save()

    def test_five_shot_round_robin_checks_every_native_gate_and_still_rejects_fixture(self):
        self.held_fixture(repeat_ammo=5)
        result = self.audit()
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["failed_checks"], ["not_a_test_fixture"])
        self.assertEqual(result["metrics"]["repeat_shooting"]["actual_shot_order"], [2, 5, 9, 2, 5])
        self.assertEqual(len(result["metrics"]["shot_hold_evidence"]), 5)
        self.assertEqual(result["metrics"]["total_score"], 49)
        self.assertEqual(result["metrics"]["latest_target_score_total"], 29)
        self.assertFalse(result["native_acceptance_passed"])

    def test_repeated_score_cannot_use_last_target_total_or_invent_a_fifth_physical_shot(self):
        self.held_fixture(repeat_ammo=5)
        self.summary["mission"]["total_score"] = 29
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["score_attribution_and_precision"])
        self.assertFalse(result["checks"]["round_robin_total_score_is_all_shots"])
        self.summary["mission"]["total_score"] = 49
        self.summary["shots"].pop()
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["shot_event_and_ledgers_match"])
        self.assertFalse(result["checks"]["round_robin_complete_expected_shot_sequence"])

    def test_second_round_native_gate_and_first_round_score_cannot_hide_behind_last_dictionary(self):
        self.held_fixture(repeat_ammo=5)
        self.events[-1]["official_fire_gate"]["referee_time_s"] += 1
        self.summary["mission"]["shots"][0]["credited_ring"] = 0
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["all_shots_have_fresh_stationary_native_gate"])
        self.assertFalse(result["checks"]["round_robin_per_shot_credit_matches_physical_outcome"])
        self.assertFalse(result["checks"]["round_robin_shot_score_sum"])

    def test_frozen_official_policy_input_can_pass_gates_but_fixture_never_native_acceptance(self):
        self.held_fixture()
        result = self.audit()
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["failed_checks"], ["not_a_test_fixture"])
        self.assertTrue(all(row["held_attitude_exception"] for row in result["metrics"]["shot_hold_evidence"]))
        self.assertFalse(result["native_acceptance_passed"])

    def test_official_hold_requires_reported_and_actual_five_file_policy_pins(self):
        self.held_fixture()
        profile = self.summary["official_dynamics"]["aim_profile"]
        profile["verified"] = False
        self.save()
        self.assertFalse(self.audit()["checks"]["official_hold_profile_source_and_staged_hashes_verified"])
        profile["verified"] = True
        source = self.repo / next(iter(HOLD_PROFILE_FILES))
        source.write_bytes(b"changed SDK binary")
        self.save()
        self.assertFalse(self.audit()["checks"]["official_hold_profile_source_and_staged_hashes_verified"])

    def test_hold_policy_staged_bytes_and_manifest_cannot_be_omitted(self):
        self.held_fixture()
        policy = "mc/build/export/onnx_model_crypto/xg/policy_balancestand_withyaw_0423"
        self.manifest["copies"] = [row for row in self.manifest["copies"] if not row["destination"].replace("\\", "/").endswith(policy)]
        self.save()
        self.assertFalse(self.audit()["checks"]["official_hold_profile_source_and_staged_hashes_verified"])

    def test_hold_policy_staged_bytes_must_equal_pinned_source(self):
        self.held_fixture()
        policy = self.runtime / "mc/build/export/onnx_model_crypto/xg/policy_balancestand_withyaw_0423"
        policy.write_bytes(b"changed staged policy")
        self.assertFalse(self.audit()["checks"]["official_hold_profile_source_and_staged_hashes_verified"])

    def test_velocity_command_cannot_be_disguised_as_frozen_attitude(self):
        self.held_fixture()
        event = self.events[-1]
        event["official_execution"].update(requested_mode="move", session_mode="move")
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_wrong_profile_missing_hold_or_wrong_identity_rejects_nonzero_shot(self):
        self.held_fixture()
        for key, value in (("profile", "generic_velocity"), ("phase", "tracking"), ("target_id", 2),
                           ("input_in_bounds", False), ("failure_reason", "no_progress")):
            with self.subTest(key=key):
                servo = self.events[-1]["official_execution"]["aim_servo"]
                old = servo[key]
                servo[key] = value
                self.save()
                self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])
                servo[key] = old

    def test_gate_profile_mode_or_hold_time_must_match_servo(self):
        self.held_fixture()
        gate = self.events[-1]["official_fire_gate"]
        for key, value in (("control_policy", "other"), ("sdk_ctrlmode", 18), ("hold_started_at_s", 12.0),
                           ("hold_source", "arbitrary nonzero velocity"), ("target_id", 2)):
            with self.subTest(key=key):
                old = gate[key]
                gate[key] = value
                self.save()
                self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])
                gate[key] = old

    def test_hold_cannot_claim_in_bounds_while_its_real_input_exceeds_limits(self):
        self.held_fixture()
        event = self.events[-1]
        event["official_execution"]["applied_command"]["pitch_rate_rad_s"] = .4
        event["official_execution"]["aim_servo"]["held_command"]["pitch_rate_rad_s"] = .4
        event["official_fire_gate"]["hold_command"]["pitch_rate_rad_s"] = .4
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_insufficient_hold_time_rejects_nonzero_shot(self):
        self.held_fixture()
        event = self.events[-1]
        servo = event["official_execution"]["aim_servo"]
        servo["hold_started_at_s"] = event["time_s"] - .2
        event["official_fire_gate"]["hold_started_at_s"] = servo["hold_started_at_s"]
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_hold_alignment_limit_cannot_be_relaxed_beyond_reviewed_eight_mm(self):
        self.held_fixture()
        self.config["control"]["aim_tolerance_m"] = .02
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_forged_held_command_or_last_change_time_is_rejected(self):
        self.held_fixture()
        servo = self.events[-1]["official_execution"]["aim_servo"]
        servo["held_command"]["pitch_rate_rad_s"] = -.03
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])
        servo["held_command"]["pitch_rate_rad_s"] = -.02
        servo["last_input_change_at_s"] = self.events[-1]["time_s"] - .1
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_sdk_zero_during_declared_hold_cannot_be_hidden_by_matching_final_command(self):
        self.held_fixture()
        sdk = self.summary["official_dynamics"]["sdk_events"]
        sdk[-3]["arguments"] = [0, 0, 0, 0]
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_native_motion_or_lost_alignment_during_dwell_rejects_hold(self):
        self.held_fixture()
        self.physics[-3]["native_packet"]["gyro_body"] = [0, 0, .1]
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])
        self.physics[-3]["native_packet"]["gyro_body"] = [0, 0, 0]
        self.events[-3]["geometry"]["error_norm_m"] = .02
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_ten_hz_images_cover_non_frame_aligned_dwell_boundaries(self):
        self.held_fixture()
        self.config["control"]["max_frame_age_s"] = .1
        self.save()
        result = self.audit()
        self.assertTrue(result["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_late_first_image_cannot_shorten_claimed_stationary_dwell(self):
        self.held_fixture()
        self.events = [row for row in self.events if row.get("active_target_id") != 9 or row["time_s"] >= 12.8]
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["all_shots_have_fresh_stationary_native_gate"])
        self.assertIn("unique_continuous_dwell_images", result["metrics"]["shot_hold_evidence"][-1]["hold_failed_checks"])

    def test_native_sample_before_dwell_boundary_must_also_be_stationary(self):
        self.held_fixture()
        shot = self.events[-1]
        boundary = shot["official_fire_gate"]["referee_time_s"] - self.config["control"]["stable_duration_s"]
        prior = [row for row in self.physics if row["published_at_s"] < boundary]
        prior[-1]["native_packet"]["gyro_body"] = [0, 0, .1]
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_complete_fixture_schema_is_checked_but_never_native_acceptance(self):
        result = self.audit()
        self.assertEqual(result["errors"], [])
        self.assertTrue(result["complete"])
        self.assertEqual(result["failed_checks"], ["not_a_test_fixture"])
        self.assertFalse(result["all_passed"])
        self.assertFalse(result["native_acceptance_passed"])

    def test_stale_fire_gate_rejected(self):
        self.events[-1]["official_fire_gate"]["referee_time_s"] += 1
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_duplicate_identity_or_wrong_physical_target_rejected(self):
        self.summary["shots"][-1]["actual_target_id"] = 2
        self.save()
        self.assertFalse(self.audit()["checks"]["unique_correct_target_and_ammo_debit"])

    def test_duplicated_ammo_debit_rejected(self):
        self.summary["shots"][-1]["ammo_before"] = 4
        self.save()
        self.assertFalse(self.audit()["checks"]["unique_correct_target_and_ammo_debit"])

    def test_constant_q_qd_cannot_prove_four_leg_gait(self):
        for row in self.physics:
            row["native_packet"]["q"] = [0] * 12
            row["native_packet"]["qd"] = [0] * 12
        self.save()
        self.assertFalse(self.audit()["checks"]["coarse_turns_have_official_leg_motion"])

    def test_only_attitude_leg_adjustment_cannot_count_as_coarse_gait(self):
        for row in self.physics[:17]:
            row["native_packet"]["q"] = [0] * 12
            row["native_packet"]["qd"] = [0] * 12
        self.save()
        self.assertFalse(self.audit()["checks"]["coarse_turns_have_official_leg_motion"])

    def test_unrecorded_source_timestamp_cannot_be_a_fresh_frame(self):
        self.events[-1]["frame_timestamp_s"] -= .02
        self.save()
        self.assertFalse(self.audit()["checks"]["frames_and_feedback_match_recorded_source_publications"])

    def test_moving_native_state_cannot_pass_fire_gate(self):
        self.physics[-1]["native_packet"]["gyro_body"] = [0, 0, 1]
        self.save()
        self.assertFalse(self.audit()["checks"]["all_shots_have_fresh_stationary_native_gate"])

    def test_source_or_executable_tampering_detected(self):
        (self.repo / "src/robot_mc/build/export/mc/bin/mc_ctrl").write_bytes(b"tampered")
        self.assertFalse(self.audit()["checks"]["official_source_hashes_verified"])

    def test_force_killed_child_is_not_a_clean_shutdown(self):
        self.runtime_status["owned_processes"]["mc_ctrl"]["exit_code"] = -9
        self.save()
        self.assertFalse(self.audit()["checks"]["owned_process_exit_codes_and_identity"])

    def test_actual_wrong_controller_ip_is_rejected_despite_correct_config_and_environment(self):
        self.controller_log_text = "client_ip: 192.168.234.1\nInitialized FSM state: PASSIVE\n"
        self.save()
        result = self.audit()
        self.assertTrue(result["checks"]["staged_official_configs_use_task3_and_loopback"])
        self.assertTrue(result["checks"]["controller_bind_environment_recorded"])
        self.assertFalse(result["checks"]["controller_actual_log_uses_loopback"])

    def test_sdk_initialization_errors_are_not_hidden_by_fsm_or_sdk_success(self):
        for error in ("sdk init failed!", "bind:Cannot assign requested address", "BIND: Cannot assign requested address"):
            with self.subTest(error=error):
                self.controller_log_text = "client_ip: 127.0.0.1\n" + error + "\nInitialized FSM state: PASSIVE\n"
                self.save()
                result = self.audit()
                self.assertTrue(result["checks"]["sdk_results_and_mode_sequence"])
                self.assertFalse(result["checks"]["controller_log_has_no_sdk_initialization_failure"])

    def test_missing_controller_log_cannot_be_replaced_with_preflight(self):
        (self.runtime / "mc_ctrl.log").unlink()
        result = self.audit()
        self.assertFalse(result["checks"]["controller_actual_log_uses_loopback"])
        self.assertFalse(result["checks"]["controller_log_has_no_sdk_initialization_failure"])

    def test_early_startup_failure_keeps_binding_diagnosis_without_observation_artifacts(self):
        self.controller_log_text = "client_ip: 192.168.234.1\nbind: Cannot assign requested address\nsdk init failed!\n"
        self.save()
        (self.run / "events.jsonl").unlink()
        result = self.audit()
        self.assertFalse(result["complete"])
        self.assertFalse(result["checks"]["controller_actual_log_uses_loopback"])
        self.assertFalse(result["checks"]["controller_log_has_no_sdk_initialization_failure"])
        self.assertEqual(result["metrics"]["controller_actual_client_ips"], ["192.168.234.1"])

    def test_missing_or_conflicting_binding_environment_is_rejected(self):
        preflight = self.summary["official_dynamics"]["runtime"]["preflight"]
        for value in (None, "192.168.234.1"):
            with self.subTest(value=value):
                preflight["sdk_controller_bind_ip"] = value
                self.save()
                self.assertFalse(self.audit()["checks"]["controller_bind_environment_recorded"])

    def test_alive_process_does_not_prove_successful_native_initialization(self):
        runtime = self.summary["official_dynamics"]["runtime"]
        runtime["all_official_processes_launched"] = False
        runtime["startup_error"] = "controller SDK initialization failed"
        self.save()
        self.assertFalse(self.audit()["checks"]["owned_process_startup_completed"])

    def test_native_exit_before_cleanup_is_rejected_even_with_valid_sdk_ledger(self):
        row = self.runtime_status["owned_processes"]["mc_ctrl"]
        row.update(exit_code=12, exit_code_before_cleanup=12, termination_requested=False)
        self.save()
        result = self.audit()
        self.assertTrue(result["checks"]["sdk_results_and_mode_sequence"])
        self.assertFalse(result["checks"]["owned_processes_alive_before_cleanup"])
        self.assertEqual(result["metrics"]["owned_process_shutdown"]["mc_ctrl"]["classification"], "exited_before_cleanup")

    def test_known_controller_abort_after_requested_cleanup_is_explicitly_reported(self):
        self.runtime_status["owned_processes"]["mc_ctrl"]["exit_code"] = -6
        self.controller_log_text += "Data saved to file: 1\nterminate called without an active exception\n"
        self.save()
        result = self.audit()
        self.assertEqual(result["failed_checks"], ["not_a_test_fixture"])
        self.assertEqual(result["metrics"]["cleanup_nonzero_exit_codes"]["mc_ctrl"], -6)
        self.assertEqual(result["metrics"]["owned_process_shutdown"]["mc_ctrl"]["classification"], "controlled_controller_teardown_abort")
        self.assertIn("SIGABRT (-6)", result["warnings"][0])
        self.assertFalse(result["native_acceptance_passed"])

    def test_controller_abort_before_cleanup_is_never_the_teardown_exception(self):
        self.runtime_status["owned_processes"]["mc_ctrl"].update(exit_code=-6, exit_code_before_cleanup=-6, termination_requested=False)
        self.controller_log_text += "terminate called without an active exception\n"
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["owned_process_exit_codes_and_identity"])
        self.assertEqual(result["warnings"], [])

    def test_abort_without_explicit_cleanup_evidence_is_rejected(self):
        row = self.runtime_status["owned_processes"]["mc_ctrl"]
        row["exit_code"] = -6
        del row["exit_code_before_cleanup"]
        self.controller_log_text += "terminate called without an active exception\n"
        self.save()
        result = self.audit()
        self.assertFalse(result["checks"]["owned_processes_have_controlled_termination"])
        self.assertEqual(result["metrics"]["owned_process_shutdown"]["mc_ctrl"]["classification"], "missing_cleanup_evidence")

    def test_other_native_aborts_and_force_kills_cannot_use_controller_exception(self):
        for name, code, force in (("robot_mujoco", -6, False), ("mc_ctrl", -11, False), ("mc_ctrl", -6, True)):
            with self.subTest(name=name, code=code, force=force):
                row = self.runtime_status["owned_processes"][name]
                old = dict(row)
                row.update(exit_code=code, forced_kill=force)
                self.controller_log_text += "terminate called without an active exception\n"
                self.save()
                self.assertFalse(self.audit()["checks"]["owned_processes_have_controlled_termination"])
                row.update(old)

    def test_staged_config_must_select_native_motor_and_loopback(self):
        (self.runtime / "mc/build/export/config/xg-user-parameters.yaml").write_text("motor_platform_type: 8\n", encoding="utf-8")
        self.assertFalse(self.audit()["checks"]["staged_official_configs_use_task3_and_loopback"])

    def test_sdk_error_status_is_not_ignored(self):
        self.summary["official_dynamics"]["sdk_events"][2]["code"] = 0x3013
        self.save()
        self.assertFalse(self.audit()["checks"]["sdk_results_and_mode_sequence"])

    def test_kinematic_fallback_cannot_be_native_acceptance(self):
        self.summary["motion_model"] = "kinematic_base_with_velocity_lag"
        self.save()
        self.assertFalse(self.audit()["checks"]["official_backend_no_kinematic_fallback"])

    def test_cli_writes_json_and_returns_two_for_a_fixture(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            code = main([str(self.run), "--repo-root", str(self.repo)])
        self.assertEqual(code, 2)
        self.assertFalse(json.loads(output.getvalue())["native_acceptance_passed"])
        self.assertTrue((self.run / "official_validation.json").is_file())

    def test_missing_native_evidence_is_incomplete_not_a_pass(self):
        (self.run / "official_physics.jsonl").unlink()
        result = self.audit()
        self.assertFalse(result["complete"])
        self.assertFalse(result["all_passed"])
        self.assertTrue(result["errors"])


if __name__ == "__main__":
    unittest.main()
