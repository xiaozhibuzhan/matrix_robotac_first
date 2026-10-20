"""Injected publication gaps and recovery; no official native gait is run.

The source samples and clocks are explicit fixture inputs. Production
publication timestamps are neither patched nor manufactured by recovery.
"""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from task3_sim.interfaces import RobotMotionCommand
from task3_sim.multi import _attempt_fire
from task3_sim.tests.test_official_backend import aim_decision, ready_hold_fixture, sample
from task3_sim.tests.test_official_fire_transaction import armed_mission, CheckedRayDebit


def recovery_fixture():
    robot, clock, image, feedback = ready_hold_fixture()
    robot.state_recovery_timeout_s = .03
    robot.observation_epoch = 0
    robot.minimum_observation_timestamp_s = -float("inf")
    robot.recovery_events = []
    return robot, clock, image, feedback


class OfficialRecoveryTests(unittest.TestCase):
    def test_nonblocking_gap_stops_nonzero_hold_and_revokes_all_old_evidence(self):
        for missing in (True, False):
            with self.subTest(missing=missing):
                robot, clock, _, _ = recovery_fixture()
                self.assertNotEqual(robot.session.last_command, RobotMotionCommand())
                self.assertTrue(robot._aim_observations)
                self.assertTrue(robot._stationary_samples)
                robot.receiver.sample = None if missing else sample(14, published=9.7)
                before = robot.data.qpos.copy()
                robot._wait = Mock(side_effect=AssertionError("shot gate must not wait"))
                with patch("task3_sim.official_backend.mujoco.mj_forward") as forward, \
                     patch("task3_sim.official_backend.mujoco.mj_step") as step:
                    self.assertFalse(robot._sync(wait_for_fresh=False))
                forward.assert_not_called()
                step.assert_not_called()
                robot._wait.assert_not_called()
                np.testing.assert_array_equal(robot.data.qpos, before)
                self.assertEqual(robot.session.last_command, RobotMotionCommand())
                self.assertEqual(robot.session.calls[-1], ("stop",))
                self.assertEqual(robot.aim_servo.report()["phase"], "reset")
                self.assertFalse(robot._aim_observations)
                self.assertFalse(robot._stationary_samples)
                self.assertEqual(robot.observation_epoch, 1)
                self.assertEqual(robot.minimum_observation_timestamp_s, clock.now())
                self.assertEqual(robot.recovery_events[-1]["status"], "shot_rejected")

    def test_wait_recovery_requires_post_stop_source_publication_and_keeps_its_timestamp(self):
        robot, clock, _, _ = recovery_fixture()
        robot.receiver.sample = sample(14, published=9.7)
        before = robot.data.qpos.copy()
        cutoff = clock.now()
        waits = []
        recovered = sample(16, published=10.015)
        recovered.packet["q"] = np.linspace(-.25, .25, 12).tolist()

        def wait(duration):
            waits.append(duration)
            np.testing.assert_array_equal(robot.data.qpos, before)
            self.assertEqual(robot.session.last_command, RobotMotionCommand())
            self.assertFalse(robot._stationary_samples)
            self.assertFalse(robot._aim_observations)
            clock.value += .01
            # First arrival is fresh in age but predates the safety stop. It
            # cannot authorize a new render/hold; only the next packet can.
            robot.receiver.sample = sample(15, published=9.999) if len(waits) == 1 else recovered

        robot._wait = wait
        with patch("task3_sim.official_backend.time.monotonic", side_effect=clock.now), \
             patch("task3_sim.official_backend.mujoco.mj_step") as step:
            self.assertTrue(robot._sync())
        step.assert_not_called()
        self.assertEqual(len(waits), 2)
        self.assertIs(robot.sample, recovered)
        self.assertEqual(robot.sample.published_at_s, 10.015)
        self.assertEqual(robot.minimum_observation_timestamp_s, cutoff)
        self.assertEqual(robot.observation_epoch, 1)
        self.assertEqual(robot.aim_servo.report()["phase"], "reset")
        np.testing.assert_array_equal(robot.data.qpos[robot._qadr], recovered.packet["q"])
        self.assertEqual(list(robot._stationary_samples), [recovered])
        event = robot.recovery_events[-1]
        self.assertEqual(event["status"], "recovered")
        self.assertEqual(event["started_at_s"], cutoff)
        self.assertEqual(event["source_sequence"], 16)
        self.assertEqual(event["published_at_s"], recovered.published_at_s)

    def test_next_sync_cannot_replay_pre_revocation_packet_even_if_its_age_is_fresh(self):
        robot, clock, _, _ = recovery_fixture()
        robot.receiver.sample = None
        self.assertFalse(robot._sync(wait_for_fresh=False))
        cutoff = robot.minimum_observation_timestamp_s
        clock.value += .01
        robot.receiver.sample = sample(14, published=cutoff-.001)
        before = robot.data.qpos.copy()
        with patch("task3_sim.official_backend.mujoco.mj_forward") as forward:
            self.assertFalse(robot._sync(wait_for_fresh=False))
        forward.assert_not_called()
        np.testing.assert_array_equal(robot.data.qpos, before)
        self.assertFalse(robot._aim_observations)
        clock.value += .01
        recovered = sample(15, published=clock.now()-.001)
        robot.receiver.sample = recovered
        self.assertTrue(robot._sync(wait_for_fresh=False))
        self.assertIs(robot.sample, recovered)
        self.assertEqual(robot.aim_servo.report()["phase"], "reset")

    def test_persistent_gap_exits_with_bounded_timeout_without_replaying_old_pose(self):
        robot, clock, _, _ = recovery_fixture()
        robot.receiver.sample = None
        before = robot.data.qpos.copy()
        started = clock.now()
        waits = []

        def wait(duration):
            waits.append(duration)
            clock.value += max(.001, duration)

        robot._wait = wait
        with patch("task3_sim.official_backend.time.monotonic", side_effect=clock.now), \
             patch("task3_sim.official_backend.mujoco.mj_forward") as forward, \
             patch("task3_sim.official_backend.mujoco.mj_step") as step:
            with self.assertRaisesRegex(RuntimeError, "missing or stale"):
                robot._sync()
        forward.assert_not_called()
        step.assert_not_called()
        np.testing.assert_array_equal(robot.data.qpos, before)
        self.assertGreaterEqual(clock.now(), started+.03)
        self.assertLessEqual(clock.now()-started, .04)
        self.assertLessEqual(len(waits), 4)
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertEqual(robot.recovery_events[-1]["status"], "timeout")
        self.assertFalse(robot._aim_observations)
        self.assertFalse(robot._stationary_samples)

    def test_native_process_failure_during_recovery_is_immediate(self):
        robot, _, _, _ = recovery_fixture()
        robot.receiver.sample = None
        robot.runtime.check_alive = Mock(side_effect=[None, RuntimeError("native controller exited")])
        robot._wait = Mock(side_effect=AssertionError("dead native process must not be retried"))
        with patch("task3_sim.official_backend.mujoco.mj_forward") as forward:
            with self.assertRaisesRegex(RuntimeError, "native controller exited"):
                robot._sync()
        robot._wait.assert_not_called()
        forward.assert_not_called()
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertNotEqual(robot.recovery_events[-1]["status"], "recovered")

    def test_malformed_source_during_recovery_is_immediate_and_never_becomes_feedback(self):
        robot, _, _, _ = recovery_fixture()
        robot.receiver.latest = Mock(side_effect=[None, RuntimeError("receiver rejected invalid quaternion")])
        robot._wait = Mock(side_effect=AssertionError("malformed source must not be retried"))
        before = robot.data.qpos.copy()
        with patch("task3_sim.official_backend.mujoco.mj_forward") as forward:
            with self.assertRaisesRegex(RuntimeError, "invalid quaternion"):
                robot._sync()
        robot._wait.assert_not_called()
        forward.assert_not_called()
        np.testing.assert_array_equal(robot.data.qpos, before)
        self.assertEqual(robot.session.last_command, RobotMotionCommand())
        self.assertNotEqual(robot.recovery_events[-1]["status"], "recovered")

    def test_capture_timeout_never_renders_a_missing_or_old_pose(self):
        robot, clock, _, _ = recovery_fixture()
        robot.receiver.sample = None
        robot.renderer = Mock()
        robot._wait = lambda duration: setattr(clock, "value", clock.now()+max(duration, .001))
        with patch("task3_sim.official_backend.time.monotonic", side_effect=clock.now), \
             patch("task3_sim.official_backend.mujoco.mj_forward") as forward, \
             patch("task3_sim.official_backend.mujoco.mj_step") as step:
            with self.assertRaisesRegex(RuntimeError, "missing or stale"):
                robot.capture()
        robot.renderer.update_scene.assert_not_called()
        robot.renderer.render.assert_not_called()
        forward.assert_not_called()
        step.assert_not_called()

    def test_capture_after_recovery_renders_only_new_source_and_returns_original_publication(self):
        robot, clock, _, _ = recovery_fixture()
        robot.receiver.sample = None
        recovered = sample(14, published=10.005)
        robot.renderer = Mock()
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        robot.renderer.render.return_value = rgb
        robot.render_option = object()
        robot.camera_effects = SimpleNamespace(apply=lambda value: value)

        def wait(duration):
            robot.renderer.update_scene.assert_not_called()
            robot.renderer.render.assert_not_called()
            clock.value += .01
            robot.receiver.sample = recovered

        robot._wait = wait
        with patch("task3_sim.official_backend.time.monotonic", side_effect=clock.now), \
             patch("task3_sim.official_backend.mujoco.mj_step") as step:
            image = robot.capture()
        step.assert_not_called()
        robot.renderer.update_scene.assert_called_once_with(robot.data, camera="task3_camera", scene_option=robot.render_option)
        robot.renderer.render.assert_called_once()
        self.assertEqual(image.timestamp, recovered.published_at_s)
        self.assertIs(robot.sample, recovered)
        self.assertEqual(robot.aim_servo.report()["phase"], "reset")

    def test_neither_fire_gate_waits_for_recovery_or_debits_ammo_after_source_gap(self):
        for phase in ("first", "final"):
            with self.subTest(phase=phase):
                robot, clock, image, feedback = recovery_fixture()
                robot._wait = Mock(side_effect=AssertionError("firing cannot wait on an old image"))
                original = robot.receiver.sample
                reads = 0

                def latest():
                    nonlocal reads
                    reads += 1
                    if phase == "first" or reads == 2:
                        # Real time consumes the original source's validity;
                        # no replacement timestamp is supplied to that source.
                        clock.value = 10.2
                    return original

                robot.receiver.latest = latest
                mission, decision = armed_mission()
                before = mission.summary()
                shooter = CheckedRayDebit(lambda: self.fail("stale source reached ray/debit"), [])
                shot, _, gate, _ = _attempt_fire(mission, robot, shooter, decision, image, feedback, 9.97)
                self.assertIsNone(shot)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("without waiting", gate["reason"])
                robot._wait.assert_not_called()
                self.assertEqual(reads, 1 if phase == "first" else 2)
                self.assertEqual(shooter.calls, [])
                self.assertEqual(shooter.ammo_remaining, 5)
                self.assertEqual(mission.summary(), before)
                self.assertEqual(robot.session.last_command, RobotMotionCommand())
                self.assertFalse(robot._aim_observations)
                self.assertFalse(robot._stationary_samples)

    def test_fresh_sync_preserves_hold_and_does_not_create_recovery_event(self):
        robot, _, _, _ = recovery_fixture()
        held = robot.session.last_command
        robot._wait = Mock(side_effect=AssertionError("fresh state needs no recovery"))
        self.assertTrue(robot._sync())
        self.assertEqual(robot.session.last_command, held)
        self.assertEqual(robot.aim_servo.report()["phase"], "holding")
        self.assertEqual(robot.observation_epoch, 0)
        self.assertEqual(robot.recovery_events, [])
        robot._wait.assert_not_called()

    def test_safe_states_stop_current_move_family_without_standup_or_mode_transition(self):
        for state in ("WAIT", "PROBING", "LOST", "FAILED", "SWITCHING", "UNKNOWN"):
            with self.subTest(state=state):
                robot, clock, _, feedback = recovery_fixture()
                # Model a coarse turn with stale hold evidence still present.
                # Even a nonzero command attached to a safety state has no
                # authority to start standing/attitude control.
                robot.session.mode = "move"
                robot.session.last_command = RobotMotionCommand(yaw_rate_rad_s=.2)
                before = len(robot.session.calls)
                now = clock.now()
                decision = aim_decision(state, command=RobotMotionCommand(yaw_rate_rad_s=.2))
                self.assertFalse(robot.ExecuteDecision(decision,
                    observation_timestamp_s=now, feedback=feedback))
                self.assertEqual(robot.session.calls[before:], [("stop",)])
                self.assertEqual(robot.session.mode, "move")
                self.assertEqual(clock.now(), now)
                self.assertEqual(robot.session.last_command, RobotMotionCommand())
                self.assertEqual(robot.session.events[-1]["operation"], "move")
                self.assertEqual(robot.session.events[-1]["arguments"], [0., 0., 0.])
                self.assertEqual(robot.aim_servo.report()["phase"], "reset")
                self.assertIsNone(robot.aim_servo.report()["hold_started_at_s"])
                self.assertFalse(robot._aim_observations)
                self.assertEqual(robot.last_execution["requested_mode"], "stop_current_mode")
                self.assertEqual(robot.last_execution["applied_command"], vars(RobotMotionCommand()))
                self.assertFalse(robot.last_execution["observation_invalidated"])

    def test_invalid_image_feedback_or_mode_immediately_stops_and_cannot_debit_ammo(self):
        cases = ("missing_image", "expired_image", "future_image", "invalid_feedback",
                 "expired_feedback", "invalid_current_feedback", "current_gyro",
                 "current_speed", "move_mode")
        for reason in cases:
            with self.subTest(reason=reason):
                robot, clock, image, feedback = recovery_fixture()
                if reason == "missing_image":
                    image = None
                elif reason == "expired_image":
                    image = replace(image, timestamp=clock.now()-.2)
                elif reason == "future_image":
                    image = replace(image, timestamp=clock.now()+.001)
                elif reason == "invalid_feedback":
                    feedback = replace(feedback, valid=False)
                elif reason == "expired_feedback":
                    feedback = replace(feedback, timestamp=clock.now()-.2)
                elif reason == "invalid_current_feedback":
                    robot.session.feedback = replace(feedback, valid=False)
                elif reason == "current_gyro":
                    robot.session.feedback = replace(feedback, gyro_rad_s=np.array([.03, 0., 0.]))
                elif reason == "current_speed":
                    robot.session.feedback = replace(feedback, velocity_m_s=np.array([.02, 0., 0.]))
                else:
                    robot.session.mode = "move"
                before_calls = len(robot.session.calls)
                robot._wait = Mock(side_effect=AssertionError("invalid shot observation cannot wait"))
                robot._sync = Mock(side_effect=AssertionError("invalid shot observation cannot replay state"))
                mission, decision = armed_mission()
                before_mission = mission.summary()
                shooter = CheckedRayDebit(lambda: self.fail("invalid observation reached ray/debit"), [])
                shot, _, gate, _ = _attempt_fire(mission, robot, shooter, decision,
                                                image, feedback, 9.97)
                self.assertIsNone(shot)
                self.assertFalse(gate["allowed"], gate)
                self.assertEqual(robot.session.calls[before_calls:], [("stop",)])
                self.assertEqual(robot.session.last_command, RobotMotionCommand())
                self.assertEqual(robot.aim_servo.report()["phase"], "reset")
                self.assertIsNone(robot.aim_servo.report()["hold_started_at_s"])
                self.assertFalse(robot._aim_observations)
                robot._wait.assert_not_called()
                robot._sync.assert_not_called()
                self.assertEqual(shooter.calls, [])
                self.assertEqual(shooter.ammo_remaining, 5)
                self.assertEqual(mission.summary(), before_mission)


if __name__ == "__main__":
    unittest.main()
