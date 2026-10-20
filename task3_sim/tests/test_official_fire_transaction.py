"""Injected gate/shot transactions; these tests do not execute native gait."""
from dataclasses import replace
from types import SimpleNamespace
import unittest

from task3_sim.interfaces import RobotMotionCommand, ShotResult
from task3_sim.mission import MultiTargetMission
from task3_sim.multi import _attempt_fire
from task3_sim.tests.test_multi_mission import config, detection, feedback, frame
from task3_sim.tests.test_official_backend import ready_hold_fixture


def armed_mission():
    mission = MultiTargetMission(config(ids=(2,)))
    for now in (9.66, 9.76, 9.86, 9.97):
        decision = mission.update(frame(now), [detection(2, now)], feedback(now), now, 5)
    if not decision.should_fire:
        raise AssertionError(decision.reason)
    return mission, decision


class CheckedRayDebit:
    """Inject the physical referee boundary and inspect its transaction lock."""
    def __init__(self, check, order):
        self.ammo_remaining = 5
        self.check, self.order = check, order
        self.calls = []

    def shooting_fire(self, target):
        self.check()
        self.order.append("ray_and_debit")
        self.calls.append(target)
        self.ammo_remaining -= 1
        return ShotResult(target, 10, True)


class OfficialFireTransactionTests(unittest.TestCase):
    def test_native_hold_is_preserved_through_ray_ammo_and_mission_ledger_under_same_lock(self):
        robot, clock, image, fb = ready_hold_fixture()
        mission, decision = armed_mission()
        held = robot.session.last_command
        calls_before = list(robot.session.calls)
        order = []

        def check():
            self.assertTrue(robot.session._lock._is_owned())
            self.assertEqual(robot.session.last_command, held)
            self.assertEqual(robot.session.calls, calls_before)
            self.assertNotEqual(held, RobotMotionCommand())

        shooter = CheckedRayDebit(check, order)
        original = mission.record_shot
        def commit(*args):
            check()
            self.assertEqual(shooter.ammo_remaining, 4)
            order.append("mission_ledger")
            return original(*args)
        mission.record_shot = commit
        shot, interrupts, gate, at = _attempt_fire(mission, robot, shooter, decision, image, fb, 9.97)
        self.assertEqual(order, ["ray_and_debit", "mission_ledger"])
        self.assertEqual(at, clock.now())
        self.assertEqual(shot.target_id, 2)
        self.assertEqual(interrupts, [])
        self.assertTrue(gate["allowed"], gate)
        self.assertEqual(shooter.ammo_remaining, 4)
        self.assertEqual(mission.summary()["ammo_remaining"], 4)
        self.assertEqual(mission.summary()["shots"][0]["timestamp"], clock.now())
        self.assertEqual(mission.status, "SUCCESS")
        self.assertEqual(robot.session.calls, calls_before)
        self.assertFalse(robot.session._lock._is_owned())

    def test_denied_real_hold_gate_never_runs_ray_or_changes_ammo_or_mission(self):
        robot, _, image, fb = ready_hold_fixture()
        robot.session.sdk.getCurrentCtrlmode = lambda: 18
        mission, decision = armed_mission()
        before = mission.summary()
        shooter = CheckedRayDebit(lambda: self.fail("denied gate reached physical shot"), [])
        shot, interrupts, gate, at = _attempt_fire(mission, robot, shooter, decision, image, fb, 9.97)
        self.assertIsNone(shot)
        self.assertEqual(interrupts, [])
        self.assertFalse(gate["allowed"])
        self.assertEqual(at, 9.97)
        self.assertEqual(shooter.calls, [])
        self.assertEqual(shooter.ammo_remaining, 5)
        self.assertEqual(mission.summary(), before)

    def test_held_target_cannot_shoot_another_decision_identity_before_debit(self):
        robot, _, image, fb = ready_hold_fixture()
        mission, decision = armed_mission()
        shooter = CheckedRayDebit(lambda: self.fail("identity mismatch reached physical shot"), [])
        shot, _, gate, _ = _attempt_fire(mission, robot, shooter, replace(decision, target_id=9), image, fb, 9.97)
        self.assertIsNone(shot)
        self.assertFalse(gate["allowed"])
        self.assertIn("identity", gate["reason"])
        self.assertEqual(shooter.ammo_remaining, 5)
        self.assertEqual(mission.summary()["shots"], [])

    def test_slow_final_mode_getter_cannot_outlive_lease_or_source_age_and_fire(self):
        for neutral in (False, True):
            with self.subTest(neutral=neutral):
                robot, clock, image, fb = ready_hold_fixture(neutral=neutral)
                mission, decision = armed_mission()
                before = mission.summary()
                reads = 0
                def slow_final_mode():
                    nonlocal reads
                    reads += 1
                    self.assertTrue(robot.session._lock._is_owned())
                    if reads == 2:
                        clock.value += .2
                    return 1 if neutral else 21
                robot.session.sdk.getCurrentCtrlmode = slow_final_mode
                shooter = CheckedRayDebit(lambda: self.fail("expired final gate reached ray/debit"), [])
                shot, _, gate, _ = _attempt_fire(mission, robot, shooter, decision, image, fb, 9.97)
                self.assertIsNone(shot)
                self.assertFalse(gate["allowed"], gate)
                self.assertIn("final shot instant expired", gate["reason"])
                self.assertEqual(shooter.ammo_remaining, 5)
                self.assertEqual(shooter.calls, [])
                self.assertEqual(mission.summary(), before)
                self.assertIn(("stop",), robot.session.calls)
                self.assertEqual(robot.session.last_command, RobotMotionCommand())
                self.assertFalse(robot._aim_observations)
                self.assertEqual(robot.aim_servo.report()["phase"], "reset")

    def test_kinematic_path_retains_stop_before_ray_and_uses_original_time(self):
        mission, decision = armed_mission()
        order = []
        robot = SimpleNamespace(Stop=lambda: order.append("stop"))
        shooter = CheckedRayDebit(lambda: self.assertEqual(order, ["stop"]), order)
        shot, interrupts, gate, at = _attempt_fire(mission, robot, shooter, decision, frame(9.97), feedback(9.97), 9.97)
        self.assertEqual(order, ["stop", "ray_and_debit"])
        self.assertEqual(shot.target_id, 2)
        self.assertEqual(interrupts, [])
        self.assertIsNone(gate)
        self.assertEqual(at, 9.97)
        self.assertEqual(mission.status, "SUCCESS")
        self.assertEqual(shooter.ammo_remaining, 4)


if __name__ == "__main__":
    unittest.main()
