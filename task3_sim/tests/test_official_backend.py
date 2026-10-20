"""Adapter contracts with fake SDK/state inputs; no native gait acceptance.

These tests exercise Python replay, mode sequencing and fire gates. The
proprietary controller, official ELF and real robot are never executed here.
"""
from __future__ import annotations

import io
import threading
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import mujoco
import numpy as np

from task3_sim.interfaces import Frame, RobotFeedback, RobotMotionCommand
from task3_sim.geometry import AimGeometry
from task3_sim.mission import MissionDecision
from task3_sim.official_aim import OfficialAimServo
from task3_sim.official_backend import OfficialDynamicsRobot
from task3_sim.official_scene import JOINT_NAMES
from task3_sim.official_state import PhysicsSample, sample_feedback


class FakeClock:
    value = 10.0

    def now(self):
        return self.value


def sample(sequence=1, published=9.99, *, gyro=None, speed=None):
    packet = {"q": np.linspace(-.12, .12, 12).tolist(),
              "qd": np.linspace(.01, .12, 12).tolist(), "tau": [0.0] * 12,
              "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
              "imu_position_world": [0.1, -0.2, .3],
              "gyro_body": [0.0, 0.0, 0.0] if gyro is None else gyro,
              "velocity_world": [0.0, 0.0, 0.0] if speed is None else speed}
    return PhysicsSample(sequence, sequence, published, published + .001, packet)


class FakeReceiver:
    max_age_s = .15

    def __init__(self, clock):
        self.now = clock.now
        self.sample = sample()

    def latest(self):
        return self.sample


class FakeSession:
    def __init__(self, clock, mode="standing"):
        self.clock, self.mode = clock, mode
        self.calls = []
        self.events = []
        self.last_command = RobotMotionCommand()
        self.lease_until_s = None
        self._lock = threading.RLock()
        self.sdk = SimpleNamespace(getCurrentCtrlmode=lambda: {"standing": 1, "move": 18, "attitude": 21}[self.mode])
        self.feedback = RobotFeedback(True, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), clock.now())

    def _event(self, operation, arguments):
        self.events.append({"kind": "sdk_result", "operation": operation, "arguments": list(arguments),
                            "code": 0, "read_at_s": self.clock.now()})

    def StandUp(self):
        self.calls.append(("standUp",))
        self.clock.value += 4.0
        self.mode = "standing"
        self.last_command = RobotMotionCommand()
        self._event("standUp", ())

    def ExecuteMove(self, command, **kwargs):
        if kwargs.get("valid_until_s", self.clock.now()) <= self.clock.now():
            command = RobotMotionCommand()
        self.calls.append(("move", command, kwargs))
        if any(vars(command).values()):
            self.mode = "move"
        self.last_command = command
        self.lease_until_s = kwargs.get("valid_until_s") if any(vars(command).values()) else None
        if self.mode != "standing":
            self._event("move", (command.vx_m_s, command.vy_m_s, command.yaw_rate_rad_s))
        return command

    def ExecuteAttitude(self, command, **kwargs):
        if kwargs.get("valid_until_s", self.clock.now()) <= self.clock.now():
            command = RobotMotionCommand()
        self.calls.append(("attitude", command, kwargs))
        if any(vars(command).values()):
            self.mode = "attitude"
        self.last_command = command
        self.lease_until_s = kwargs.get("valid_until_s") if any(vars(command).values()) else None
        if self.mode != "standing":
            self._event("attitudeControl", (0., command.pitch_rate_rad_s, command.yaw_rate_rad_s, 0.))
        return command

    def ReadFeedback(self):
        return self.feedback

    def check_current_lease(self):
        nonzero = any(vars(self.last_command).values())
        if nonzero and (self.lease_until_s is None or self.clock.now() >= self.lease_until_s):
            self.Stop()
            return False
        return nonzero

    def Stop(self):
        self.calls.append(("stop",))
        self.last_command = RobotMotionCommand()
        self.lease_until_s = None
        if self.mode in ("move", "attitude"):
            self._event("move" if self.mode == "move" else "attitudeControl", (0., 0., 0.) if self.mode == "move" else (0., 0., 0., 0.))


def robot_fixture(mode="standing"):
    """Avoid the native constructor and model actual source packets explicitly."""
    clock = FakeClock()
    robot = OfficialDynamicsRobot.__new__(OfficialDynamicsRobot)
    robot.receiver = FakeReceiver(clock)
    robot.session = FakeSession(clock, mode)
    robot.config = {"control": {"max_frame_age_s": .15, "max_feedback_age_s": .15,
                                "max_gyro_rad_s": .02, "max_speed_m_s": .01,
                                "max_yaw_rate_rad_s": .35, "max_pitch_rate_rad_s": .18,
                                "aim_tolerance_m": .008, "stable_duration_s": .4}}
    robot.aim_servo = OfficialAimServo(robot.config["control"])
    robot._aim_observations = deque(maxlen=200)
    robot._stationary_samples = deque(maxlen=500)
    robot.aim_profile = {"verified": True, "profile": robot.aim_servo.profile}
    robot.runtime = SimpleNamespace(check_alive=Mock(return_value=True))
    children = "".join(f'<body name="leg_{index}" pos="0 0 .02"><joint name="{name}" axis="0 1 0"/>'
                       '<geom type="sphere" size=".01" mass=".1"/></body>'
                       for index, name in enumerate(JOINT_NAMES))
    robot.model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="base_link">'
        '<freejoint/><geom type="sphere" size=".1" mass="10"/>' + children + '</body></worldbody></mujoco>')
    robot.data = mujoco.MjData(robot.model)
    robot._qadr = np.array([robot.model.jnt_qposadr[mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                           for name in JOINT_NAMES])
    robot._vadr = np.array([robot.model.jnt_dofadr[mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                           for name in JOINT_NAMES])
    robot._trace, robot._trace_lock = io.StringIO(), threading.RLock()
    robot._last_trace_seq, robot.physics_samples = None, 0
    robot.sample, robot.last_execution = None, {}
    return robot, clock


def aim_decision(state="AIMING", *, target=2, command=None):
    angle = .04 if state == "AIMING" else .001
    geometry = AimGeometry(np.array([2.7, angle*2.7, angle*2.7]), np.array([1., 0., 0.]),
        np.array([2.7, 0., 0.]), np.array([angle, angle])*2.7,
        float(np.sqrt(2)*angle*2.7), angle, -angle, 2.7, True)
    if command is None:
        command = (RobotMotionCommand(yaw_rate_rad_s=.06, pitch_rate_rad_s=-.06)
                   if state == "AIMING" else RobotMotionCommand())
    return MissionDecision(command, state, state == "READY", geometry, "injected test observation", target, "RUNNING")


def ready_hold_fixture(*, neutral=False):
    """Synthetic packets and SDK calls model gate evidence; no native gait ran."""
    robot, clock = robot_fixture("standing" if neutral else "attitude")
    for index in range(13):
        clock.value = round(9.4 + index*.05, 8)
        robot.receiver.sample = sample(sequence=index+1, published=clock.now())
        robot.session.feedback = sample_feedback(robot.receiver.sample, clock.now(), robot.receiver.max_age_s)
        robot._sync()
        state = "AIMING" if index == 0 and not neutral else "READY" if index == 12 else "SETTLING"
        robot.ExecuteDecision(aim_decision(state), observation_timestamp_s=clock.now(), feedback=robot.session.feedback)
    frame = Frame(np.zeros((2, 2, 3), np.uint8), clock.now())
    return robot, clock, frame, robot.session.feedback


class OfficialBackendContractTests(unittest.TestCase):
    def test_sdk_startup_failure_still_keeps_acquired_physics_evidence(self):
        robot, _ = robot_fixture()
        robot.receiver.port = 55000
        robot.receiver.feedback = lambda: sample_feedback(robot.receiver.sample, robot.receiver.now(), robot.receiver.max_age_s)
        robot.bridge_executable = "fixture_bridge"
        robot.runtime.start = Mock()
        robot.session.Connect = Mock(side_effect=RuntimeError("SDK connection fixture failure"))
        with self.assertRaisesRegex(RuntimeError, "SDK connection fixture failure"):
            robot.Connect()
        self.assertEqual(robot.startup_phase, "sdk_connect")
        self.assertEqual(robot.physics_samples, 1)
        self.assertIn('"q":', robot._trace.getvalue())
        # Logging a source sample must not initialize a synthetic render pose.
        self.assertIsNone(robot.sample)

    def test_zero_ready_after_move_requires_stand_transition_and_invalidates_observation(self):
        robot, clock = robot_fixture("move")
        decision = aim_decision("READY")
        self.assertTrue(robot.ExecuteDecision(decision))
        self.assertEqual(robot.session.calls, [("standUp",)])
        self.assertEqual(robot.session.mode, "standing")
        self.assertEqual(clock.now(), 14)
        self.assertTrue(robot.last_execution["observation_invalidated"])
        self.assertEqual(robot.last_execution["applied_command"], vars(RobotMotionCommand()))

    def test_nonzero_old_aim_command_is_discarded_through_mode_transition(self):
        robot, clock = robot_fixture("move")
        old = aim_decision(command=RobotMotionCommand(yaw_rate_rad_s=.05, pitch_rate_rad_s=.08))
        self.assertTrue(robot.ExecuteDecision(old))
        self.assertEqual(robot.session.calls, [("standUp",)])
        self.assertEqual(robot.last_execution["applied_command"], vars(RobotMotionCommand()))
        # Only an explicit subsequent call with a newly acquired decision may
        # reach the SDK attitude family after the four-second transition.
        new = aim_decision(command=RobotMotionCommand(yaw_rate_rad_s=.02, pitch_rate_rad_s=-.03))
        self.assertFalse(robot.ExecuteDecision(new, observation_timestamp_s=clock.now(), feedback=robot.session.feedback))
        self.assertEqual(robot.session.calls[1][0], "attitude")
        self.assertEqual(robot.session.calls[1][1], new.command)
        self.assertEqual(robot.session.calls[1][2]["issued_at_s"], clock.now())

    def test_source_sample_used_by_referee_must_be_stationary_even_when_previous_feedback_was(self):
        for changes in ({"gyro": [.03, 0, 0]}, {"speed": [.02, 0, 0]}):
            with self.subTest(changes=changes):
                robot, clock = robot_fixture()
                robot.receiver.sample = sample(sequence=2, **changes)
                frame = Frame(np.zeros((2, 2, 3), np.uint8), 9.99)
                previous = robot.session.ReadFeedback()
                self.assertEqual(float(np.linalg.norm(previous.gyro_rad_s)), 0)
                self.assertEqual(float(np.linalg.norm(previous.velocity_m_s)), 0)
                gate = robot.prepare_fire(frame, previous)
                self.assertFalse(gate["allowed"])
                self.assertIn("referee physics sample changed", gate["reason"])
                self.assertEqual(robot.sample.sequence, 2)

    def test_fresh_stationary_referee_sample_is_identified_by_its_source_sequence(self):
        robot, clock, frame, feedback = ready_hold_fixture()
        gate = robot.prepare_fire(frame, feedback)
        self.assertTrue(gate["allowed"], gate)
        self.assertEqual(gate["referee_source_sequence"], 13)
        self.assertEqual(gate["referee_published_at_s"], clock.now())
        self.assertEqual(gate["sdk_ctrlmode"], 21)
        self.assertEqual(gate["hold_command"], vars(robot.session.last_command))

    def test_stale_image_or_move_mode_cannot_authorize_a_virtual_shot(self):
        for mode, timestamp in (("move", 9.99), ("standing", 9.7)):
            with self.subTest(mode=mode, timestamp=timestamp):
                robot, _ = robot_fixture(mode)
                gate = robot.prepare_fire(Frame(np.zeros((2, 2, 3), np.uint8), timestamp), robot.session.ReadFeedback())
                self.assertFalse(gate["allowed"])
                self.assertIsNone(robot.sample)  # No referee update was authorized.

    def test_nonzero_without_acquisition_timestamp_has_no_renewable_motion_lease(self):
        robot, _ = robot_fixture("attitude")
        robot.ExecuteDecision(aim_decision())
        self.assertEqual(robot.last_execution["applied_command"], vars(RobotMotionCommand()))
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertFalse(robot._aim_observations)

    def test_ready_keeps_exact_full_input_and_prepare_fire_never_stops_or_zeroes_it(self):
        robot, clock, frame, feedback = ready_hold_fixture()
        held = robot.session.last_command
        self.assertTrue(held.yaw_rate_rad_s)
        self.assertTrue(held.pitch_rate_rad_s)
        self.assertEqual(robot.last_execution["aim_servo"]["phase"], "holding")
        before = list(robot.session.calls)
        gate = robot.prepare_fire(frame, feedback)
        self.assertTrue(gate["allowed"], gate)
        self.assertEqual(robot.session.calls, before)
        self.assertEqual(robot.session.last_command, held)
        self.assertEqual(robot.fire_guard(), robot.session._lock)
        self.assertGreaterEqual(clock.now()-gate["hold_started_at_s"], .4)

    def test_already_aligned_neutral_stand_can_fire_after_complete_evidence_dwell(self):
        robot, clock, frame, feedback = ready_hold_fixture(neutral=True)
        self.assertEqual(robot.session.mode, "standing")
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertEqual(robot.session.events, [])  # Zero did not enter attitude mode.
        gate = robot.prepare_fire(frame, feedback)
        self.assertTrue(gate["allowed"], gate)
        self.assertEqual(gate["sdk_ctrlmode"], 1)
        self.assertGreaterEqual(clock.now()-gate["hold_started_at_s"], .4)
        for operation, args in (("standUp", []), ("attitudeControl", [0., .01, 0., 0.])):
            with self.subTest(operation=operation):
                robot, _, frame, feedback = ready_hold_fixture(neutral=True)
                robot.session.events.append({"kind": "sdk_result", "operation": operation,
                    "arguments": args, "code": 0, "read_at_s": 9.8})
                self.assertFalse(robot.prepare_fire(frame, feedback)["allowed"])

    def test_mode_profile_failure_and_current_input_changes_cannot_authorize_fire(self):
        for kind in ("mode21", "unverified_profile", "stalled", "zero_input", "different_input", "short_dwell"):
            with self.subTest(kind=kind):
                robot, _, frame, feedback = ready_hold_fixture()
                if kind == "mode21":
                    robot.session.sdk.getCurrentCtrlmode = lambda: 18
                elif kind == "unverified_profile":
                    robot.aim_profile["verified"] = False
                elif kind == "stalled":
                    robot.aim_servo.failure_reason = "aim_response_stalled"
                elif kind == "zero_input":
                    robot.session.last_command = RobotMotionCommand()
                elif kind == "different_input":
                    robot.session.last_command = RobotMotionCommand(yaw_rate_rad_s=.061, pitch_rate_rad_s=-.06)
                else:
                    robot.aim_servo.hold_started_at_s = 9.8
                gate = robot.prepare_fire(frame, feedback)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("unchanged attitude hold", gate["reason"])

    def test_actual_sdk_ledger_rejects_intervening_zero_change_move_standup_or_error(self):
        for operation, args, code in (("attitudeControl", [0., 0., 0., 0.], 0),
                                      ("attitudeControl", [0., -.061, .06, 0.], 0),
                                      ("move", [0., 0., 0.], 0), ("standUp", [], 0),
                                      ("attitudeControl", [0., -.06, .06, 0.], 1)):
            with self.subTest(operation=operation, args=args, code=code):
                robot, _, frame, feedback = ready_hold_fixture()
                robot.session.events.append({"kind": "sdk_result", "operation": operation, "arguments": args,
                                             "code": code, "read_at_s": 9.8})
                gate = robot.prepare_fire(frame, feedback)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("actual SDK ledger", gate["reason"])

    def test_image_hold_requires_complete_unique_aligned_same_target_stationary_history(self):
        for kind in ("wrong_target", "duplicate_frame", "capture_gap", "feedback_stale", "invalid_feedback",
                     "gyro", "speed", "outside_tolerance", "short_history", "nonfinite_error"):
            with self.subTest(kind=kind):
                robot, _, frame, feedback = ready_hold_fixture()
                rows = robot._aim_observations
                row = rows[-3]
                if kind == "wrong_target":
                    row["target_id"] = 9
                elif kind == "duplicate_frame":
                    row["frame_timestamp_s"] = rows[-4]["frame_timestamp_s"]
                elif kind == "capture_gap":
                    robot._aim_observations = deque([r for r in rows if not 9.65 <= r["time_s"] <= 9.8])
                elif kind == "feedback_stale":
                    row["feedback_timestamp_s"] -= .2
                elif kind == "invalid_feedback":
                    row["feedback_valid"] = False
                elif kind in ("gyro", "speed"):
                    row[kind] = .03
                elif kind == "outside_tolerance":
                    row["error_m"] = .02
                elif kind == "short_history":
                    robot._aim_observations = deque([r for r in rows if r["time_s"] > 9.8])
                else:
                    row["error_m"] = float("nan")
                gate = robot.prepare_fire(frame, feedback)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("fresh aligned images", gate["reason"])

    def test_native_packets_must_cover_entire_hold_without_unobserved_gap_or_motion(self):
        for kind in ("late_start", "gap", "gyro", "velocity", "nonfinite_gyro", "nonfinite_velocity"):
            with self.subTest(kind=kind):
                robot, _, frame, feedback = ready_hold_fixture()
                rows = robot._stationary_samples
                if kind == "late_start":
                    robot._stationary_samples = deque([r for r in rows if r.published_at_s > 9.7])
                elif kind == "gap":
                    robot._stationary_samples = deque([r for r in rows if not 9.65 <= r.published_at_s <= 9.8])
                else:
                    field = "gyro_body" if "gyro" in kind else "velocity_world"
                    rows[-3].packet[field] = [float("nan") if kind.startswith("nonfinite") else .03, 0., 0.]
                gate = robot.prepare_fire(frame, feedback)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("native stationary feedback", gate["reason"])

    def test_wait_searching_or_new_target_revokes_previous_hold_history(self):
        for state, target in (("WAIT", 2), ("SEARCHING", 2), ("AIMING", 9)):
            with self.subTest(state=state, target=target):
                robot, clock, _, feedback = ready_hold_fixture()
                clock.value += .05
                robot.ExecuteDecision(aim_decision(state, target=target),
                                      observation_timestamp_s=clock.now(), feedback=feedback)
                self.assertFalse(robot._aim_observations)
                self.assertNotEqual(robot.aim_servo.report()["phase"], "holding")

    def test_background_zero_or_same_call_zero_restarts_dwell_then_fresh_hold_can_fire(self):
        for interruption in ("background_zero", "zero_inside_reissue"):
            with self.subTest(interruption=interruption):
                robot, clock, _, _ = ready_hold_fixture()
                held = robot.session.last_command
                original = robot.session.ExecuteAttitude
                if interruption == "background_zero":
                    clock.value = 10.01
                    robot.session.Stop()
                calls = 0
                def reissue(command, **kwargs):
                    nonlocal calls
                    self.assertTrue(robot.session._lock._is_owned())
                    if interruption == "zero_inside_reissue" and calls == 0:
                        robot.session.Stop()
                    calls += 1
                    # Native calls consume real time. The final hold epoch is
                    # later than both an injected zero and its successful reissue.
                    clock.value += .001
                    applied = original(command, **kwargs)
                    clock.value += .001
                    return applied
                robot.session.ExecuteAttitude = reissue
                for index in range(12):
                    clock.value = round(10.05 + .05*index, 8)
                    robot.receiver.sample = sample(sequence=14+index, published=clock.now())
                    robot.session.feedback = sample_feedback(robot.receiver.sample, clock.now(), .15)
                    robot._sync()
                    stamp = clock.now()
                    self.assertFalse(robot.ExecuteDecision(aim_decision("READY"),
                        observation_timestamp_s=stamp, feedback=robot.session.feedback))
                    if index == 0:
                        self.assertTrue(robot.last_execution["hold_restarted"])
                        self.assertFalse(robot.last_execution["observation_invalidated"])
                        self.assertEqual(robot.session.last_command, held)
                        self.assertEqual(len(robot._aim_observations), 1)
                        restart = robot.aim_servo.report()["hold_started_at_s"]
                        self.assertGreater(restart, stamp)
                        gate = robot.prepare_fire(Frame(np.zeros((2,2,3), np.uint8), stamp), robot.session.feedback)
                        self.assertFalse(gate["allowed"], gate)
                    else:
                        self.assertFalse(robot.last_execution["hold_restarted"])
                image = Frame(np.zeros((2,2,3), np.uint8), robot.receiver.sample.published_at_s)
                gate = robot.prepare_fire(image, robot.session.feedback)
                self.assertTrue(gate["allowed"], gate)
                self.assertGreaterEqual(clock.now()-restart, .4)
                self.assertEqual(robot.session.last_command, held)

    def test_interrupted_hold_that_cannot_reissue_with_new_lease_is_revoked(self):
        robot, clock, _, _ = ready_hold_fixture()
        clock.value = 10.01
        robot.session.Stop()
        clock.value = 10.05
        # No acquisition timestamp grants no lease; the applied input is zero.
        self.assertFalse(robot.ExecuteDecision(aim_decision("READY")))
        self.assertTrue(robot.last_execution["hold_restarted"])
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertEqual(robot.aim_servo.report()["phase"], "reset")
        self.assertIsNone(robot.aim_servo.report()["hold_started_at_s"])
        self.assertFalse(robot._aim_observations)

    def test_repeated_identical_zero_input_does_not_restart_hold(self):
        for mode in ("standing", "attitude"):
            with self.subTest(mode=mode):
                robot, clock = robot_fixture(mode)
                first = None
                for index in range(12):
                    clock.value = round(10. + .05*index, 8)
                    robot.receiver.sample = sample(sequence=index+1, published=clock.now())
                    robot.session.feedback = sample_feedback(robot.receiver.sample, clock.now(), .15)
                    robot._sync()
                    robot.ExecuteDecision(aim_decision("READY"), observation_timestamp_s=clock.now(),
                                          feedback=robot.session.feedback)
                    if first is None:
                        first = robot.aim_servo.report()["hold_started_at_s"]
                    self.assertFalse(robot.last_execution["hold_restarted"])
                    self.assertEqual(robot.aim_servo.report()["hold_started_at_s"], first)
                gate = robot.prepare_fire(Frame(np.zeros((2,2,3), np.uint8), clock.now()), robot.session.feedback)
                self.assertTrue(gate["allowed"], gate)

    def test_native_fire_gate_cannot_relax_eight_mm_or_point_four_second_limits(self):
        for changes in ({"aim_tolerance_m": .008001}, {"aim_tolerance_m": 0.},
                        {"stable_duration_s": .39999}):
            with self.subTest(changes=changes):
                robot, _, frame, feedback = ready_hold_fixture()
                robot.config["control"].update(changes)
                gate = robot.prepare_fire(frame, feedback)
                self.assertFalse(gate["allowed"], gate)

    def test_replay_preserves_all_twelve_joint_samples_and_never_steps_physics(self):
        robot, clock = robot_fixture()
        original = robot.receiver.sample
        with patch("task3_sim.official_backend.mujoco.mj_step", side_effect=AssertionError("Python must not integrate native physics")):
            robot._sync()
            robot._sync()
        np.testing.assert_allclose(robot.data.qpos[:3], original.packet["imu_position_world"])
        np.testing.assert_allclose(robot.data.qpos[3:7], original.packet["quat_wxyz"])
        np.testing.assert_allclose(robot.data.qpos[robot._qadr], original.packet["q"])
        np.testing.assert_allclose(robot.data.qvel[robot._vadr], original.packet["qd"])
        np.testing.assert_allclose(robot.data.qvel[:3], original.packet["velocity_world"])
        np.testing.assert_allclose(robot.data.qvel[3:6], original.packet["gyro_body"])
        self.assertEqual(robot.data.time, clock.now())
        self.assertEqual(robot.physics_samples, 1)
        self.assertEqual(len(robot._trace.getvalue().splitlines()), 1)

    def test_missing_or_stale_state_stops_without_inventing_joint_motion(self):
        for invalid in (None, sample(published=9.0)):
            with self.subTest(invalid=invalid):
                robot, _ = robot_fixture("move")
                robot.state_recovery_timeout_s = .001
                robot.receiver.sample = invalid
                before = robot.data.qpos.copy()
                with self.assertRaisesRegex(RuntimeError, "missing or stale"):
                    robot._sync()
                np.testing.assert_array_equal(robot.data.qpos, before)
                self.assertEqual(robot.session.calls, [("stop",)])


if __name__ == "__main__":
    unittest.main()
