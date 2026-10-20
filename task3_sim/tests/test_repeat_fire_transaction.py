"""Repeat-shot authorization and real debit transactions using injected rays."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from task3_sim.interfaces import ShotResult
from task3_sim.multi import _attempt_fire
from task3_sim.tests.test_multi_mission import feedback, frame
from task3_sim.tests.test_round_robin_mission import RoundDriver, round_config


class FiniteReferee:
    def __init__(self, ammo):
        self.ammo_remaining = ammo
        self.calls = []

    def shooting_fire(self, target):
        if self.ammo_remaining <= 0:
            raise AssertionError("empty shared magazine reached physical referee")
        self.ammo_remaining -= 1
        self.calls.append(target)
        return ShotResult(target, 10, True)


class RepeatFireTransactionTests(unittest.TestCase):
    def test_five_physical_debits_have_one_matching_mission_commit_each(self):
        driver = RoundDriver(ammo=5)
        shooter = FiniteReferee(5)
        robot = SimpleNamespace(Stop=Mock())
        for index, identity in enumerate((2, 5, 9, 2, 5)):
            decision = driver.arm(identity)
            shot, interrupts, gate, at = _attempt_fire(driver.mission, robot, shooter, decision,
                frame(driver.now), feedback(driver.now), driver.now)
            driver.ammo = shooter.ammo_remaining
            self.assertEqual(shot.target_id, identity)
            self.assertEqual(interrupts, [])
            self.assertIsNone(gate)
            self.assertEqual(at, driver.now)
            report = driver.mission.summary()
            self.assertEqual(report["ammo_remaining"], 4-index)
            self.assertEqual(len(report["shots"]), index+1)
            self.assertEqual(report["shots"][-1]["ammo_before"], 5-index)
            self.assertEqual(report["shots"][-1]["ammo_remaining"], 4-index)
        self.assertEqual(shooter.calls, [2, 5, 9, 2, 5])
        self.assertEqual(shooter.ammo_remaining, 0)
        self.assertEqual(driver.mission.status, "SUCCESS")
        self.assertEqual(robot.Stop.call_count, 5)

    def test_polling_new_frame_revokes_old_decision_before_physical_referee(self):
        driver = RoundDriver(ammo=5)
        old = driver.arm(2)
        old_time = driver.now
        self.assertTrue(driver.step().should_fire)
        shooter = FiniteReferee(5)
        robot = SimpleNamespace(Stop=Mock())
        before = driver.mission.summary()
        shot, _, gate, _ = _attempt_fire(driver.mission, robot, shooter, old,
            frame(old_time), feedback(old_time), driver.now)
        self.assertIsNone(shot)
        self.assertFalse(gate["allowed"], gate)
        self.assertEqual(shooter.calls, [])
        self.assertEqual(shooter.ammo_remaining, 5)
        self.assertEqual(driver.mission.summary(), before)
        robot.Stop.assert_called_once()

    def test_old_visit_or_consumed_token_cannot_debit_same_identity_again(self):
        driver = RoundDriver(round_config(ids=(2,)), ammo=2)
        shooter = FiniteReferee(2)
        robot = SimpleNamespace(Stop=Mock())
        old = driver.arm(2)
        old_time = driver.now
        shot, _, _, _ = _attempt_fire(driver.mission, robot, shooter, old,
            frame(old_time), feedback(old_time), old_time)
        self.assertEqual(shot.target_id, 2)
        driver.ammo = shooter.ammo_remaining
        self.assertEqual(driver.ammo, 1)
        for new_visit in (False, True):
            with self.subTest(new_visit=new_visit):
                if new_visit:
                    driver.arm(2)
                before = driver.mission.summary()
                shot, _, gate, _ = _attempt_fire(driver.mission, robot, shooter, old,
                    frame(old_time), feedback(old_time), driver.now)
                self.assertIsNone(shot)
                self.assertFalse(gate["allowed"], gate)
                self.assertEqual(shooter.calls, [2])
                self.assertEqual(shooter.ammo_remaining, 1)
                self.assertEqual(driver.mission.summary(), before)

    def test_expired_image_or_external_ammo_change_rejects_before_ray_and_preserves_magazine(self):
        for reason in ("expired", "ammo_changed"):
            with self.subTest(reason=reason):
                driver = RoundDriver(ammo=5)
                decision = driver.arm(2)
                capture = driver.now
                shooter = FiniteReferee(5)
                if reason == "expired":
                    driver.now += .2
                else:
                    shooter.ammo_remaining = 4
                before_ammo = shooter.ammo_remaining
                robot = SimpleNamespace(Stop=Mock())
                before = driver.mission.summary()
                shot, _, gate, _ = _attempt_fire(driver.mission, robot, shooter, decision,
                    frame(capture), feedback(capture), driver.now)
                self.assertIsNone(shot)
                self.assertFalse(gate["allowed"], gate)
                self.assertEqual(shooter.calls, [])
                self.assertEqual(shooter.ammo_remaining, before_ammo)
                self.assertEqual(driver.mission.summary(), before)
                robot.Stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
