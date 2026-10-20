"""Outer-loop contracts on an illustrative bounded lag plant, not native gait."""
from dataclasses import replace
import json
import math
import unittest

import numpy as np

from task3_sim.geometry import AimGeometry
from task3_sim.interfaces import RobotMotionCommand
from task3_sim.mission import MissionDecision
from task3_sim.official_aim import OfficialAimServo


CONTROL = {"max_yaw_rate_rad_s": .35, "max_pitch_rate_rad_s": .18,
           "max_frame_age_s": .15, "stable_duration_s": .4}


def decision(angles=(.04, -.03), *, state="AIMING", target=2, command=None, error_m=None):
    angles = np.asarray(angles, dtype=float)
    miss = float(np.linalg.norm(angles) * 2.7) if error_m is None else error_m
    geometry = AimGeometry(np.array([2.7, angles[0] * 2.7, -angles[1] * 2.7]),
        np.array([1., 0., 0.]), np.array([2.7, 0., 0.]), angles * 2.7,
        miss, float(angles[0]), float(angles[1]), 2.7, True)
    if command is None:
        values = np.clip(1.5 * angles, [-.35, -.18], [.35, .18]) if state == "AIMING" else (0., 0.)
        command = RobotMotionCommand(yaw_rate_rad_s=float(values[0]), pitch_rate_rad_s=float(values[1]))
    return MissionDecision(command, state, state == "READY", geometry, "test observation", target, "RUNNING")


class OfficialAimTests(unittest.TestCase):
    def plant(self, servo=None):
        # An illustrative bounded static attitude response with a first-order
        # lag and bias. It is deliberately not an angular-velocity integrator.
        # Neither its constants nor convergence certify the proprietary policy.
        pose = np.zeros(2)
        goal, gain, bias = np.array([.08, -.06]), np.array([.7, .85]), np.array([.01, -.015])
        history = []
        for index in range(600):
            now = index * .05
            residual = goal - pose
            miss = float(np.linalg.norm(residual) * 2.7)
            state = "AIMING" if miss > .008 else "SETTLING"
            observed = decision(residual, state=state)
            command = servo.update(observed, now) if servo else observed.command
            applied = np.array([command.yaw_rate_rad_s, command.pitch_rate_rad_s])
            pose += .05 / .3 * (np.clip(gain * applied + bias, -.3, .3) - pose)
            history.append((miss, state, applied.copy(), pose.copy()))
        return history

    def test_existing_p_has_residual_but_pi_converges_and_holds_on_lag_plant(self):
        p = self.plant()
        self.assertGreater(p[-1][0], .05)
        self.assertLess(p[-1][0], .16)
        servo = OfficialAimServo(CONTROL)
        pi = self.plant(servo)
        settled = [row for row in pi if row[1] == "SETTLING"]
        self.assertTrue(settled)
        self.assertLess(pi[-1][0], .008)
        self.assertTrue(np.any(np.abs(pi[-1][2]) > .01))
        np.testing.assert_array_equal(pi[-1][2], pi[-30][2])
        self.assertEqual(servo.report()["phase"], "holding")
        self.assertIsNone(servo.failure_reason)

    def test_uses_existing_proportional_command_without_multiplying_kp(self):
        servo = OfficialAimServo({**CONTROL, "yaw_kp": 99., "pitch_kp": 99.})
        command = RobotMotionCommand(yaw_rate_rad_s=.041, pitch_rate_rad_s=-.017)
        observed = decision(command=command)
        self.assertEqual(servo.update(observed, 1.), command)
        result = servo.update(observed, 1.05)
        self.assertAlmostEqual(result.yaw_rate_rad_s, command.yaw_rate_rad_s + .8 * .04 * .05)
        self.assertAlmostEqual(result.pitch_rate_rad_s, command.pitch_rate_rad_s - .8 * .03 * .05)

    def test_settling_and_ready_hold_entire_last_input_and_freeze_integral(self):
        servo = OfficialAimServo(CONTROL)
        servo.update(decision(), 1.)
        aimed = servo.update(decision(), 1.05)
        before = servo.report()
        for state, now in (("SETTLING", 1.1), ("SETTLING", 1.3), ("READY", 1.6)):
            held = servo.update(decision((.001, -.001), state=state), now)
            self.assertEqual(held, aimed)
            report = servo.report()
            self.assertEqual(report["profile"], "official_rl_balance_policy_pi_v1")
            self.assertEqual(report["phase"], "holding")
            self.assertEqual(report["target_id"], 2)
            self.assertEqual(report["held_command"], vars(held))
            self.assertEqual(report["integral_input"], before["integral_input"])
            self.assertEqual(report["hold_started_at_s"], 1.1)
            self.assertEqual(report["last_input_change_at_s"], 1.05)
            self.assertEqual(report["integrated_dt_s"], 0.)
            self.assertTrue(report["input_in_bounds"])
        self.assertGreaterEqual(1.6 - servo.report()["hold_started_at_s"], CONTROL["stable_duration_s"])
        json.dumps(servo.report(), allow_nan=False)

    def test_restart_hold_retains_full_input_and_integral_but_replaces_dwell_epoch(self):
        servo = OfficialAimServo(CONTROL)
        with self.assertRaises(ValueError):
            servo.restart_hold(0., "not_holding")
        servo.update(decision(), 1.)
        command = servo.update(decision(), 1.05)
        servo.update(decision(state="SETTLING"), 1.1)
        before = servo.report()
        servo.restart_hold(2., "actual_sdk_input_interrupted")
        report = servo.report()
        self.assertEqual(report["held_command"], vars(command))
        self.assertEqual(report["integral_input"], before["integral_input"])
        self.assertEqual(report["proportional_input"], before["proportional_input"])
        self.assertEqual(report["hold_started_at_s"], 2.)
        self.assertEqual(report["last_input_change_at_s"], 2.)
        self.assertEqual(report["last_reset_reason"], "actual_sdk_input_interrupted")
        self.assertEqual(servo.update(decision(state="READY"), 2.05), command)
        self.assertEqual(servo.report()["hold_started_at_s"], 2.)
        for now in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                servo.restart_hold(now, "bad_time")

    def test_saturation_rejects_windup_and_integral_is_bounded(self):
        servo = OfficialAimServo(CONTROL, {"stall_duration_s": 1000})
        saturated = decision((1., -1.), command=RobotMotionCommand(yaw_rate_rad_s=.35, pitch_rate_rad_s=-.18))
        for index in range(100):
            self.assertEqual(servo.update(saturated, index * .05), saturated.command)
        self.assertEqual(servo.report()["integral_input"], {"yaw": 0., "pitch": 0.})
        zero_p = decision((1., -1.), command=RobotMotionCommand())
        for index in range(100, 300):
            command = servo.update(zero_p, index * .05)
            self.assertLessEqual(abs(command.yaw_rate_rad_s), .35)
            self.assertLessEqual(abs(command.pitch_rate_rad_s), .18)
        before = servo.report()["integral_input"]
        self.assertEqual(before, {"yaw": .35, "pitch": -.18})
        servo.update(decision((-.1, .1), command=RobotMotionCommand()), 15.)
        self.assertLess(servo.report()["integral_input"]["yaw"], before["yaw"])
        self.assertGreater(servo.report()["integral_input"]["pitch"], before["pitch"])

    def test_wait_lost_searching_mode_transition_and_target_change_reset(self):
        for state in ("WAIT", "LOST", "SEARCHING", "SWITCHING", "DONE", "UNREACHABLE"):
            with self.subTest(state=state):
                servo = OfficialAimServo(CONTROL)
                servo.update(decision(), 0.)
                servo.update(decision(), .05)
                self.assertNotEqual(servo.report()["integral_input"]["yaw"], 0.)
                self.assertEqual(servo.update(decision(state=state), .1), RobotMotionCommand())
                self.assertEqual(servo.report()["integral_input"], {"yaw": 0., "pitch": 0.})
                self.assertIsNone(servo.report()["hold_started_at_s"])
        servo = OfficialAimServo(CONTROL)
        servo.update(decision(), 0.)
        servo.update(decision(), .05)
        changed = decision(target=9)
        self.assertEqual(servo.update(changed, .1), changed.command)
        self.assertEqual(servo.report()["integral_input"], {"yaw": 0., "pitch": 0.})
        self.assertEqual(servo.report()["target_id"], 9)
        servo.reset("mode_transition")
        self.assertEqual(servo.report()["held_command"], vars(RobotMotionCommand()))
        self.assertEqual(servo.report()["last_reset_reason"], "mode_transition")

    def test_missing_frame_wait_does_not_integrate_elapsed_gap(self):
        servo = OfficialAimServo(CONTROL)
        servo.update(decision(), 0.)
        servo.update(decision(), .05)
        servo.update(decision(state="WAIT"), .1)
        recovered = decision()
        self.assertEqual(servo.update(recovered, 10.), recovered.command)
        self.assertEqual(servo.report()["integrated_dt_s"], 0.)
        servo.update(recovered, 20.)
        self.assertEqual(servo.report()["integrated_dt_s"], .15)
        self.assertAlmostEqual(servo.report()["integral_input"]["yaw"], .8 * .04 * .15)

    def test_duplicate_reverse_time_and_invalid_geometry_revoke_previous_input(self):
        for invalid in (decision((math.nan, .1)), decision(error_m=math.inf),
                        replace(decision(), geometry=None), decision(command=RobotMotionCommand(vx_m_s=.01)),
                        decision(command=RobotMotionCommand(yaw_rate_rad_s=math.inf)), decision(target=None)):
            servo = OfficialAimServo(CONTROL)
            servo.update(decision(), 0.)
            servo.update(decision(), .05)
            self.assertEqual(servo.update(invalid, .1), RobotMotionCommand())
            self.assertEqual(servo.report()["integral_input"], {"yaw": 0., "pitch": 0.})
            json.dumps(servo.report(), allow_nan=False)
        for now in (.05, .01, math.inf, math.nan):
            servo = OfficialAimServo(CONTROL)
            servo.update(decision(), .05)
            self.assertEqual(servo.update(decision(), now), RobotMotionCommand())
            self.assertEqual(servo.report()["integrated_dt_s"], 0.)

    def test_stagnation_latches_after_continuous_valid_aiming_and_clears_on_reset(self):
        servo = OfficialAimServo(CONTROL)
        for index in range(242):
            command = servo.update(decision((.01, -.01), error_m=.04), index * .05)
        self.assertEqual(servo.failure_reason, "aim_response_stalled")
        self.assertEqual(servo.report()["phase"], "failed")
        self.assertEqual(command, RobotMotionCommand())
        self.assertEqual(servo.update(decision(), 12.2), RobotMotionCommand())
        servo.reset("target_failed")
        self.assertIsNone(servo.failure_reason)

    def test_sparse_observations_cannot_prove_continuous_response_stall(self):
        servo = OfficialAimServo(CONTROL)
        for index in range(20):
            servo.update(decision((.01, -.01)), index * 5.)
            self.assertIsNone(servo.failure_reason)
            self.assertLessEqual(servo.report()["integrated_dt_s"], CONTROL["max_frame_age_s"])

    def test_saturation_stagnation_and_real_improvement_supervision(self):
        saturated = OfficialAimServo(CONTROL)
        for index in range(242):
            # Angular response is present, but there is no 2 mm improvement
            # and the request remains saturated.
            angle = .5 + .03 * math.sin(index * .05)
            saturated.update(decision((angle, -.4), error_m=.08), index * .05)
        self.assertEqual(saturated.failure_reason, "aim_response_stalled")
        progress = OfficialAimServo(CONTROL)
        for index in range(600):
            progress.update(decision((.01, -.01), error_m=.05 - index * .05 * .0003), index * .05)
        self.assertIsNone(progress.failure_reason)
        # A settling phase breaks the requirement for continuous AIMING.
        paused = OfficialAimServo(CONTROL)
        for index in range(201):
            paused.update(decision(), index * .05)
        paused.update(decision(state="SETTLING"), 10.05)
        for index in range(202, 350):
            paused.update(decision(), index * .05)
        self.assertIsNone(paused.failure_reason)

    def test_invalid_limits_and_gains_rejected(self):
        for control, settings in (({**CONTROL, "max_yaw_rate_rad_s": .51}, {}),
                                  ({**CONTROL, "max_pitch_rate_rad_s": math.nan}, {}),
                                  (CONTROL, {"yaw_ki_per_s": -.1}),
                                  (CONTROL, {"stall_duration_s": 0})):
            with self.subTest(control=control, settings=settings), self.assertRaises(ValueError):
                OfficialAimServo(control, settings)


if __name__ == "__main__":
    unittest.main()
