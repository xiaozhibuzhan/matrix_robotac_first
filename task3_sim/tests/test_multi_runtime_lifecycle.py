"""Real SIGINT at the physical-fire/mission-ledger boundary stays atomic."""

import signal
import unittest

from task3_sim.interfaces import ShotResult
from task3_sim.mission import MultiTargetMission
from task3_sim.tests.test_multi_mission import config, detection, feedback, frame


class _InterruptingShooter:
    def __init__(self, ammo=5, *, debit=1, interrupts=0, result=None):
        self.ammo_remaining = ammo
        self.debit = debit
        self.interrupts = interrupts
        self.result = result or ShotResult(2, 10, True, reason="ring_hit")
        self.ledger = []

    def shooting_fire(self, intended_id):
        before = self.ammo_remaining
        self.ammo_remaining -= self.debit
        # This is the original failure window: ammunition has been consumed,
        # but neither the physical ledger nor the mission callback is done.
        for _ in range(self.interrupts):
            signal.raise_signal(signal.SIGINT)
        self.ledger.append({"intended_target_id": intended_id,
                            "actual_target_id": self.result.target_id,
                            "ammo_before": before, "ammo_after": self.ammo_remaining})
        return self.result


class MultiFireTransactionTests(unittest.TestCase):
    def armed_mission(self, *, ids=(2, 5)):
        mission = MultiTargetMission(config(ids=ids))
        for now in (0, 0.1, 0.2, 0.31):
            decision = mission.update(frame(now), [detection(2, now)], feedback(now), now, 5)
        self.assertTrue(decision.should_fire)
        return mission

    def transact(self, mission, shooter):
        from task3_sim.multi import _fire_transaction
        try:
            return _fire_transaction(mission, shooter, 2, 0.31)
        except KeyboardInterrupt:
            self.fail("SIGINT interrupted ammunition debit before mission record was committed")

    def test_sigint_inside_firing_completes_both_ledgers_then_returns_interrupt(self):
        mission = self.armed_mission()
        shooter = _InterruptingShooter(interrupts=1)
        previous = signal.getsignal(signal.SIGINT)
        shot, received = self.transact(mission, shooter)
        self.assertEqual(received, [signal.SIGINT])
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.assertEqual(shot.target_id, 2)
        self.assertEqual(shooter.ammo_remaining, 4)
        self.assertEqual(len(shooter.ledger), 1)
        self.assertEqual(mission.target_states, {2: "COMPLETED", 5: "PENDING"})
        self.assertEqual(mission.summary()["targets"][0]["attempts"], 1)
        self.assertEqual(len(mission.summary()["shots"]), 1)
        # The caller can honor cancellation after the transaction, retaining
        # the shot already committed and failing only unfinished targets.
        mission.cancel(0.31)
        self.assertEqual(mission.status, "CANCELLED")
        self.assertEqual(mission.target_states, {2: "COMPLETED", 5: "FAILED"})
        self.assertEqual(mission.summary()["total_score"], 10)

    def test_repeated_sigint_during_last_shot_preserves_success(self):
        mission = self.armed_mission(ids=(2,))
        shooter = _InterruptingShooter(interrupts=2)
        previous = signal.getsignal(signal.SIGINT)
        _, received = self.transact(mission, shooter)
        self.assertEqual(received, [signal.SIGINT, signal.SIGINT])
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.assertEqual(mission.status, "SUCCESS")
        self.assertTrue(mission.done)
        mission.cancel(0.31)
        self.assertEqual(mission.status, "SUCCESS")
        self.assertEqual(mission.target_states[2], "COMPLETED")
        self.assertEqual(mission.summary()["total_score"], 10)
        self.assertEqual(len(shooter.ledger), 1)

    def test_wrong_ammo_decrement_rejects_completion_and_restores_handler(self):
        for debit in (0, 2):
            with self.subTest(debit=debit):
                mission = self.armed_mission()
                shooter = _InterruptingShooter(debit=debit)
                previous = signal.getsignal(signal.SIGINT)
                with self.assertRaises(RuntimeError):
                    self.transact(mission, shooter)
                self.assertEqual(signal.getsignal(signal.SIGINT), previous)
                self.assertEqual(mission.target_states[2], "ACTIVE")
                self.assertEqual(mission.summary()["targets"][0]["attempts"], 0)
                self.assertEqual(mission.summary()["shots"], [])

    def test_missing_ready_authority_rejects_fabricated_completed_record(self):
        mission = MultiTargetMission(config())
        shooter = _InterruptingShooter()
        previous = signal.getsignal(signal.SIGINT)
        with self.assertRaises(RuntimeError):
            self.transact(mission, shooter)
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.assertEqual(mission.target_states, {2: "PENDING", 5: "PENDING"})
        self.assertEqual(mission.summary()["shots"], [])

    def test_native_failure_is_not_swallowed_as_success(self):
        class FailingShooter(_InterruptingShooter):
            def shooting_fire(self, intended_id):
                signal.raise_signal(signal.SIGINT)
                raise ValueError("native shooting adapter failed")

        mission = self.armed_mission()
        previous = signal.getsignal(signal.SIGINT)
        with self.assertRaisesRegex(ValueError, "native shooting adapter failed"):
            self.transact(mission, FailingShooter())
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)
        self.assertEqual(mission.summary()["shots"], [])
        self.assertEqual(mission.target_states[2], "ACTIVE")


if __name__ == "__main__":
    unittest.main()
