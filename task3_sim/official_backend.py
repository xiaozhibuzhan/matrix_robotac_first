"""Official native physics + official HighLevel control, with a render mirror.

Python never steps or integrates this model. qpos/qvel writes ONLY replay an
acquired official RobotState for rendering and the virtual laser referee.
"""
from __future__ import annotations

import json
import hashlib
from collections import deque
from pathlib import Path
import time
import threading

import mujoco
import numpy as np

from .interfaces import Frame, RobotMotionCommand
from .official_aim import OfficialAimServo
from .official_scene import JOINT_NAMES
from .official_sdk import OfficialSDKSession, SDKLimits
from .official_state import OfficialStateReceiver, sample_feedback
from .sensor import CameraEffects


class OfficialDynamicsRobot:
    motion_model = "official_robot_mujoco_ELF_contact_dynamics+official_mc_ctrl+official_HighLevel_SDK"

    def __init__(self, model, config, *, runtime, bridge_executable, output):
        self.model, self.config, self.runtime = model, config, runtime
        self.bridge_executable, self.output = Path(bridge_executable), Path(output)
        self.data = mujoco.MjData(model)
        self.receiver = OfficialStateReceiver(max_age_s=config["control"]["max_feedback_age_s"])
        settings = config.get("official_dynamics", {})
        self.aim_servo = OfficialAimServo(config["control"], settings.get("aim_servo"))
        self._aim_observations = deque(maxlen=200)
        self._stationary_samples = deque(maxlen=500)
        self.aim_profile = self._verify_aim_profile()
        self.session = OfficialSDKSession(
            sdk_root=runtime.repo_root/"deps/zsibot_sdk", local_port=runtime.sdk_port,
            clock=self.receiver.now, wait=self._wait, state_feedback_provider=self._physics_feedback,
            limits=SDKLimits(max_yaw_rate_rad_s=config["control"]["max_yaw_rate_rad_s"],
                             max_pitch_rate_rad_s=config["control"]["max_pitch_rate_rad_s"],
                             max_attitude_yaw_rate_rad_s=min(0.5, config["control"]["max_yaw_rate_rad_s"])),
            command_timeout_s=float(settings.get("command_timeout_s", 1.0)),
            max_feedback_age_s=config["control"]["max_feedback_age_s"],
            transition_timeout_s=float(settings.get("transition_timeout_s", 20.0)),
            refresh_interval_s=float(settings.get("command_refresh_interval_s", .04)),
            allow_void_status=bool(settings.get("allow_void_status", False)))
        self.renderer = None
        self.render_option = mujoco.MjvOption()
        self.render_option.geomgroup[2], self.render_option.geomgroup[3] = 0, 1
        self.camera_effects = CameraEffects(config.get("appearance", {}).get("camera_effects"))
        self._qadr = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES])
        self._vadr = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES])
        self.sample = None
        self.last_execution = {}
        self.physics_samples = 0
        self._trace = (self.output/"official_physics.jsonl").open("w", encoding="utf-8")
        self._last_trace_seq = None
        self._trace_lock = threading.RLock()
        self.startup_phase = "not_started"
        self.state_recovery_timeout_s = 2.0
        self.recovery_events = []
        self.observation_epoch = 0
        self.minimum_observation_timestamp_s = -float("inf")

    def warmup_renderer(self):
        """Allocate GL/shader resources before native startup; discard this image.

        The XML default pose is only a graphics warmup, never an observation,
        feedback sample, control input or referee state.
        """
        self._startup("graphics_warmup", "allocating render resources; warmup image discarded before native startup")
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=self.config["camera"]["height"], width=self.config["camera"]["width"])
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="task3_camera", scene_option=self.render_option)
        self.renderer.render()

    def _startup(self, phase, message):
        self.startup_phase = phase
        print(f"TASK3_OFFICIAL_STARTUP phase={phase} {message}", flush=True)

    def _record_sample(self, sample):
        if sample is None:
            return
        with self._trace_lock:
            if self._last_trace_seq is None or sample.sequence > self._last_trace_seq:
                recorded_at = self.receiver.now()
                record = {"task_time_s": recorded_at, "published_at_s": sample.published_at_s,
                          "publication_age_s": recorded_at-sample.published_at_s,
                          "native_packet": sample.packet}
                self._trace.write(json.dumps(record, allow_nan=False)+"\n")
                self._trace.flush()
                self._last_trace_seq = sample.sequence
                self.physics_samples += 1
                self._stationary_samples.append(sample)

    def _verify_aim_profile(self):
        """Restrict the empirical hold response to the reviewed shipped policy."""
        path = Path(__file__).parent / "config/official_aim_profile.json"
        profile = json.loads(path.read_text(encoding="utf-8"))
        if profile["profile"] != self.aim_servo.profile:
            raise ValueError("Unknown official aiming profile")
        for relative, expected in profile["files_sha256"].items():
            source = self.runtime.repo_root / relative
            if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError(f"Official balance profile changed; revalidate its response before aiming: {relative}")
        return {"profile": profile["profile"], "verified": True,
                "files_sha256": profile["files_sha256"]}

    def _physics_feedback(self):
        sample = self.receiver.latest()
        self._record_sample(sample)
        return sample_feedback(sample, self.receiver.now(), self.receiver.max_age_s)

    def _wait(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            self.runtime.check_alive()
            # Keep acquired native evidence even if SDK startup subsequently
            # fails. No default pose or synthetic sample is recorded here.
            self._record_sample(self.receiver.latest())
            time.sleep(min(0.01, max(0, deadline-time.monotonic())))

    def Connect(self):
        self._startup("native_processes", "starting state bridge, official physics and controller")
        self.runtime.start(state_port=self.receiver.port, bridge_executable=self.bridge_executable)
        self._startup("physics_feedback", "waiting for a fresh official RobotState")
        deadline = time.monotonic()+15
        while not self.receiver.feedback().valid:
            if time.monotonic() >= deadline:
                raise RuntimeError("No fresh official physics RobotState; see official runtime logs")
            self._wait(0.025)
        self._record_sample(self.receiver.latest())
        self._startup("sdk_connect", "connecting official HighLevel SDK on loopback")
        self.session.Connect()
        self._startup("sdk_connected", "official SDK connection confirmed")

    def StandUp(self):
        self._startup("standing_up", "waiting for official standUp and fresh stationary feedback")
        self.session.StandUp()
        self._sync()
        self._startup("ready", "physics feedback, SDK connection and standing checks completed")

    def _sync(self, *, wait_for_fresh=True):
        self.runtime.check_alive()
        sample = self.receiver.latest()
        if (sample is None or not sample_feedback(sample, self.receiver.now(), self.receiver.max_age_s).valid
                or sample.published_at_s < getattr(self, "minimum_observation_timestamp_s", -float("inf"))):
            self.Stop()
            self._stationary_samples.clear()
            self.observation_epoch = getattr(self, "observation_epoch", 0)+1
            cutoff = self.receiver.now()
            self.minimum_observation_timestamp_s = cutoff
            if not hasattr(self, "recovery_events"):
                self.recovery_events = []
            recovery = {"started_at_s": cutoff, "status": "waiting",
                        "previous_sequence": sample.sequence if sample is not None else None,
                        "previous_published_at_s": sample.published_at_s if sample is not None else None,
                        "timeout_s": getattr(self, "state_recovery_timeout_s", 2.0)}
            self.recovery_events.append(recovery)
            if not wait_for_fresh:
                recovery["status"] = "shot_rejected"
                return False
            print("TASK3_STATE_RECOVERY stopped; waiting up to 2 s for a new official publication", flush=True)
            deadline = time.monotonic()+recovery["timeout_s"]
            while True:
                self.runtime.check_alive()
                sample = self.receiver.latest()
                if (sample is not None and sample.published_at_s >= cutoff
                        and sample_feedback(sample, self.receiver.now(), self.receiver.max_age_s).valid):
                    recovery.update(status="recovered", recovered_at_s=self.receiver.now(),
                                    source_sequence=sample.sequence, published_at_s=sample.published_at_s)
                    print(f"TASK3_STATE_RECOVERY recovered source_sequence={sample.sequence}; new hold required", flush=True)
                    break
                if time.monotonic() >= deadline:
                    recovery.update(status="timeout", finished_at_s=self.receiver.now())
                    raise RuntimeError("Official physics state is missing or stale after bounded recovery; no synthetic state fallback")
                self._wait(min(.01, max(0., deadline-time.monotonic())))
        packet = sample.packet
        self.data.qpos[:3] = packet["imu_position_world"]
        self.data.qpos[3:7] = np.array(packet["quat_wxyz"])/np.linalg.norm(packet["quat_wxyz"])
        self.data.qpos[self._qadr] = packet["q"]
        self.data.qvel[:3], self.data.qvel[3:6] = packet["velocity_world"], packet["gyro_body"]
        self.data.qvel[self._vadr] = packet["qd"]
        self.data.time = self.receiver.now()  # host elapsed, never raw protobuf simulation time
        mujoco.mj_forward(self.model, self.data)  # render/ray transforms only, NOT mj_step
        self.sample = sample
        self._record_sample(sample)
        return True

    def ReadFeedback(self):
        return self.session.ReadFeedback()

    def current_time(self):
        self.data.time = self.receiver.now()
        return float(self.data.time)

    def ExecuteDecision(self, decision, *, observation_timestamp_s=None, feedback=None):
        with self.session._lock:
            return self._execute_decision(decision, observation_timestamp_s=observation_timestamp_s, feedback=feedback)

    def _execute_decision(self, decision, *, observation_timestamp_s=None, feedback=None):
        command = decision.command
        now = self.receiver.now()
        if decision.state not in ("SEARCHING", "AIMING", "SETTLING", "READY"):
            # A missing observation authorizes only an owned-family stop,
            # never a new standUp/mode transition using that same old image.
            self.Stop()
            self.last_execution = {"requested_mode": "stop_current_mode", "applied_command": vars(RobotMotionCommand()),
                                   "started_at_s": now, "finished_at_s": self.receiver.now(),
                                   "session_mode": self.session.mode, "observation_invalidated": False,
                                   "aim_servo": self.aim_servo.report(), "hold_restarted": False, "aim_failure": None}
            return False
        old_servo = self.aim_servo.report()
        interrupted_hold = (old_servo["phase"] == "holding"
                            and vars(self.session.last_command) != old_servo["held_command"])
        sdk_event_start = len(self.session.events)
        # A small move yaw is outside the official .02 minimum. Stationary
        # attitudeControl supports continuous fine yaw and pitch together.
        coarse = decision.state == "SEARCHING" or abs(command.yaw_rate_rad_s) > 0.12
        if coarse:
            self.aim_servo.reset("coarse_move")
            self._aim_observations.clear()
            requested = RobotMotionCommand(command.vx_m_s, command.vy_m_s, command.yaw_rate_rad_s, 0.0)
            mode, action = "move", self.session.ExecuteMove
        else:
            requested = self.aim_servo.update(decision, now)
            mode, action = "attitude", self.session.ExecuteAttitude
        transition = self.session.mode in ("move", "attitude") and self.session.mode != mode
        if transition:
            # This also applies to a zero READY command: zeroing move alone
            # does not prepare stationary body attitude. Never reuse its image.
            self.session.StandUp()
            self.aim_servo.reset("official_mode_transition")
            self._aim_observations.clear()
            applied = RobotMotionCommand()
        else:
            deadline = (float(observation_timestamp_s) + self.config["control"]["max_frame_age_s"]
                        if observation_timestamp_s is not None else now)
            applied = action(requested, issued_at_s=now, valid_until_s=deadline)
        interrupted_hold = interrupted_hold or any(
            row.get("kind") == "sdk_result" and row.get("operation") in ("move", "attitudeControl")
            and not any(row.get("arguments", [])) for row in self.session.events[sdk_event_start:])
        hold_restarted = (interrupted_hold and self.aim_servo.phase == "holding" and not transition
                          and any(self.aim_servo.report()["held_command"].values()))
        if hold_restarted:
            self._aim_observations.clear()
            if vars(applied) == self.aim_servo.report()["held_command"]:
                self.aim_servo.restart_hold(self.receiver.now(), "actual_sdk_input_interrupted")
            else:
                self.aim_servo.reset("actual_sdk_input_interrupted_before_reissue")
        servo = self.aim_servo.report()
        if (servo["phase"] == "holding" and not transition
                and observation_timestamp_s is not None and feedback is not None):
            self._aim_observations.append({"time_s": now, "frame_timestamp_s": observation_timestamp_s,
                "target_id": decision.target_id, "error_m": decision.geometry.error_norm_m,
                "feedback_timestamp_s": feedback.timestamp, "feedback_valid": feedback.valid,
                "gyro": float(np.linalg.norm(feedback.gyro_rad_s)),
                "speed": float(np.linalg.norm(feedback.velocity_m_s))})
        else:
            self._aim_observations.clear()
        self.last_execution = {"requested_mode": mode, "applied_command": vars(applied),
                               "started_at_s": now, "finished_at_s": self.receiver.now(),
                               "session_mode": self.session.mode,
                               "observation_invalidated": transition, "aim_servo": servo,
                               "hold_restarted": hold_restarted,
                               "aim_failure": self.aim_servo.failure_reason}
        return transition

    def Execute(self, command):
        # Explicit family dispatch is used by run_multi; reject ambiguous use.
        return self.session.Execute(command)

    def Stop(self):
        self.aim_servo.reset("safety_stop")
        self._aim_observations.clear()
        self.session.Stop()

    def fire_guard(self):
        # Prevent a concurrent watchdog/lease stop between final gate and the
        # atomic virtual-ray/ammunition transaction. Physics remains native.
        return self.session._lock

    def _hold_error(self, now):
        control = self.config["control"]
        servo = self.aim_servo.report()
        started = servo["hold_started_at_s"]
        dwell = float(control["stable_duration_s"])
        ctrlmode = self.session.sdk.getCurrentCtrlmode()
        neutral_stand = (self.session.mode == "standing" and ctrlmode == 1
                         and not any(servo["held_command"].values()))
        if (not 0 < control["aim_tolerance_m"] <= .008 or dwell < .4
                or not self.aim_profile.get("verified") or servo["phase"] != "holding"
                or servo["failure_reason"] or not servo["input_in_bounds"]
                or started is None or now-started < dwell
                or not (neutral_stand or (self.session.mode == "attitude" and ctrlmode == 21))
                or vars(self.session.last_command) != servo["held_command"]):
            return "official policy input is not a verified unchanged attitude hold"
        # Inspect actual calls, including background refresh and watchdog zero.
        expected = [0.0, servo["held_command"]["pitch_rate_rad_s"],
                    servo["held_command"]["yaw_rate_rad_s"], 0.0]
        calls = [row for row in self.session.events if row.get("kind") == "sdk_result"
                 and row.get("operation") in ("move", "attitudeControl", "standUp")]
        window = [row for row in calls if row["read_at_s"] >= started]
        prior = [row for row in calls if row["read_at_s"] < started]
        if prior:
            window.insert(0, prior[-1])
        if not neutral_stand and (not window or now-window[0]["read_at_s"] < dwell
                or any(row["operation"] != "attitudeControl" or row.get("code") != 0
                       or row.get("arguments") != expected for row in window)):
            return "actual SDK ledger does not show a complete unchanged hold"
        if neutral_stand and any(row["read_at_s"] >= started and
                (row["operation"] == "standUp" or any(row.get("arguments", []))) for row in window):
            return "standing control changed during the aligned observation window"
        deadline = now-dwell
        observations = [row for row in self._aim_observations if row["time_s"] >= deadline]
        prior_frames = [row for row in self._aim_observations if row["time_s"] < deadline]
        if prior_frames:
            observations.insert(0, prior_frames[-1])
        if (len(observations) < 2 or observations[0]["time_s"] > deadline
                or now-observations[-1]["frame_timestamp_s"] > control["max_frame_age_s"]
                or any(not all(np.isfinite(row[key]) for key in ("time_s", "frame_timestamp_s", "feedback_timestamp_s",
                                                                 "error_m", "gyro", "speed"))
                       or row["target_id"] != servo["target_id"] or not 0 <= row["error_m"] <= control["aim_tolerance_m"]
                       or not row["feedback_valid"] or row["gyro"] > control["max_gyro_rad_s"]
                       or row["speed"] > control["max_speed_m_s"]
                       or not 0 <= row["time_s"]-row["frame_timestamp_s"] <= control["max_frame_age_s"]
                       or not 0 <= row["time_s"]-row["feedback_timestamp_s"] <= control["max_feedback_age_s"]
                       for row in observations)
                or any(not 0 < b["frame_timestamp_s"]-a["frame_timestamp_s"] <= control["max_frame_age_s"]
                       for a, b in zip(observations, observations[1:]))):
            return "fresh aligned images do not yet cover the full stationary hold"
        samples = [item for item in self._stationary_samples if item.published_at_s >= deadline]
        prior_samples = [item for item in self._stationary_samples if item.published_at_s < deadline]
        if prior_samples:
            samples.insert(0, prior_samples[-1])
        if (len(samples) < 2 or samples[0].published_at_s > deadline
                or deadline-samples[0].published_at_s > control["max_feedback_age_s"]
                or now-samples[-1].published_at_s > control["max_feedback_age_s"]
                or any(not np.isfinite(item.published_at_s)
                       or any(np.asarray(item.packet[key]).shape != (3,) or not np.all(np.isfinite(item.packet[key]))
                              for key in ("gyro_body", "velocity_world"))
                       or np.linalg.norm(item.packet["gyro_body"]) > control["max_gyro_rad_s"]
                       or np.linalg.norm(item.packet["velocity_world"]) > control["max_speed_m_s"] for item in samples)
                or any(b.published_at_s-a.published_at_s > control["max_feedback_age_s"]
                       for a, b in zip(samples, samples[1:]))):
            return "native stationary feedback does not yet cover the full hold"
        return None

    def prepare_fire(self, frame, feedback):
        now = self.receiver.now()
        control = self.config["control"]
        if (frame is None or not 0 <= now-frame.timestamp <= control["max_frame_age_s"]
                or not feedback.valid or not 0 <= now-feedback.timestamp <= control["max_feedback_age_s"]
                or self.session.mode not in ("standing", "attitude")):
            self.Stop()
            return {"allowed": False, "reason": "observation expired or control mode incomplete"}
        fresh = self.session.ReadFeedback()
        if (not fresh.valid or np.linalg.norm(fresh.gyro_rad_s) > control["max_gyro_rad_s"]
                or np.linalg.norm(fresh.velocity_m_s) > control["max_speed_m_s"]):
            self.Stop()
            return {"allowed": False, "reason": "current physics feedback is not stationary"}
        if not self._sync(wait_for_fresh=False):
            return {"allowed": False, "reason": "referee physics publication unavailable; shot cancelled without waiting"}
        physical = sample_feedback(self.sample, self.receiver.now(), self.receiver.max_age_s)
        if (not physical.valid or np.linalg.norm(physical.gyro_rad_s) > control["max_gyro_rad_s"]
                or np.linalg.norm(physical.velocity_m_s) > control["max_speed_m_s"]):
            return {"allowed": False, "reason": "referee physics sample changed before the shot"}
        hold_error = self._hold_error(self.receiver.now())
        if hold_error:
            return {"allowed": False, "reason": hold_error}
        servo = self.aim_servo.report()
        # All getters and audit work above consume real time. The fire guard
        # blocks the background monitor, so enforce its deadlines here too.
        if not self._sync(wait_for_fresh=False):
            return {"allowed": False, "reason": "final referee publication unavailable; shot cancelled without waiting"}
        physical = sample_feedback(self.sample, self.receiver.now(), self.receiver.max_age_s)
        ctrlmode = self.session.sdk.getCurrentCtrlmode()
        lease_valid = self.session.check_current_lease()
        final_now = self.receiver.now()
        if (not 0 <= final_now-frame.timestamp <= control["max_frame_age_s"]
                or not 0 <= final_now-feedback.timestamp <= control["max_feedback_age_s"]
                or not physical.valid or not 0 <= final_now-physical.timestamp <= control["max_feedback_age_s"]
                or not np.all(np.isfinite(physical.gyro_rad_s)) or not np.all(np.isfinite(physical.velocity_m_s))
                or np.linalg.norm(physical.gyro_rad_s) > control["max_gyro_rad_s"]
                or np.linalg.norm(physical.velocity_m_s) > control["max_speed_m_s"]
                or vars(self.session.last_command) != servo["held_command"]
                or (any(servo["held_command"].values()) and not lease_valid)
                or ctrlmode != (21 if self.session.mode == "attitude" else 1)):
            self.Stop()
            return {"allowed": False, "reason": "final shot instant expired or physical hold changed during verification"}
        return {"allowed": True, "referee_source_sequence": self.sample.sequence,
                "referee_published_at_s": self.sample.published_at_s,
                "referee_time_s": final_now, "sdk_ctrlmode": ctrlmode,
                "target_id": servo["target_id"],
                "control_policy": self.aim_servo.profile,
                "hold_source": "bounded official SDK attitude input frozen; actual native feedback stationary",
                "hold_command": servo["held_command"], "hold_started_at_s": servo["hold_started_at_s"]}

    def advance(self, duration):
        self._wait(duration)
        self._sync()

    def capture(self):
        self._sync()
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=self.config["camera"]["height"], width=self.config["camera"]["width"])
        self.renderer.update_scene(self.data, camera="task3_camera", scene_option=self.render_option)
        return Frame(self.camera_effects.apply(self.renderer.render().copy()), self.sample.published_at_s)

    def report(self):
        return {"motion_model": self.motion_model, "sdk_events": self.session.events,
                "aim_profile": self.aim_profile, "aim_servo": self.aim_servo.report(),
                "startup_phase": self.startup_phase,
                "last_execution": self.last_execution, "physics_samples": self.physics_samples,
                "state_packets_accepted": self.receiver.accepted, "stale_packets_dropped": self.receiver.stale_dropped,
                "stale_drop_reasons": self.receiver.stale_drop_reasons,
                "last_stale_drop": self.receiver.last_stale_drop,
                "state_recovery_events": self.recovery_events, "observation_epoch": self.observation_epoch,
                "time_basis": "host monotonic elapsed; frame/feedback timestamp = official eCAL publication mapped using bridge receive clocks; raw RobotState time_stamp unit unknown",
                "feedback_source": "official RobotState simulated sensors; SDK getter estimates are separate untimestamped diagnostics",
                "rendering": "read-only full official physical state replay; Python mj_forward only, no dynamics integration",
                "runtime": self.runtime.report()}

    def close(self):
        warnings = []
        try:
            self.session.close()
        except Exception as exc:
            warnings.append(f"SDK stop: {exc}")
        try:
            warnings.extend(self.runtime.close() or [])
        except Exception as exc:
            warnings.append(f"official runtime cleanup: {exc}")
        self.receiver.close()
        self._trace.close()
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        return warnings
