"""Finite-ammunition revisit contracts with injected images and referee results.

No MuJoCo dynamics or native SDK is run. Each fixture image has a unique
acquisition timestamp and a decoded, correctly calibrated declared identity.
"""
import json
import unittest

from task3_sim.interfaces import RobotMotionCommand, ShotResult
from task3_sim.mission import MultiTargetMission
from task3_sim.tests.test_multi_mission import config, detection, feedback, frame


def round_config(ids=(9, 2, 5), **settings):
    return config(ids=ids, shooting_policy="round_robin", **settings)


class RoundDriver:
    """Supply explicit source observations and actual one-round debits."""
    def __init__(self, source=None, ammo=5):
        self.source = round_config() if source is None else source
        self.mission = MultiTargetMission(self.source)
        self.ammo = ammo
        self.now = -.1
        self.offsets = {row["id"]: row["tag"]["bullseye_offset_tag_m"][1]
                        for row in self.source["targets"]}

    def step(self, *, detections=None):
        self.now = round(self.now+.1, 8)
        observed = ([detection(identity, self.now, offset) for identity, offset in self.offsets.items()]
                    if detections is None else detections)
        return self.mission.update(frame(self.now), observed, feedback(self.now), self.now, self.ammo)

    def select(self, identity):
        for _ in range(12):
            decision = self.step()
            if self.mission.active_target_id == identity:
                return decision
            if self.mission.done:
                break
        raise AssertionError(f"target {identity} was not selected: {self.mission.summary()}")

    def arm(self, identity):
        for _ in range(20):
            decision = self.step()
            if decision.should_fire:
                if decision.target_id != identity:
                    raise AssertionError(f"expected target {identity}, armed {decision.target_id}")
                return decision
            if self.mission.done:
                break
        raise AssertionError(f"target {identity} did not get a new stable image window: {self.mission.summary()}")

    def shoot(self, identity, ring=10, *, result=None):
        decision = self.arm(identity)
        result = ShotResult(identity, ring, True) if result is None else result
        self.ammo -= 1
        if not self.mission.record_shot(identity, result, self.now, self.ammo):
            raise AssertionError("a fresh image-authorized real debit was rejected")
        return decision


class RoundRobinMissionTests(unittest.TestCase):
    def test_five_and_six_rounds_follow_numeric_id_order_until_real_ammo_is_empty(self):
        for initial, order in ((5, [2, 5, 9, 2, 5]), (6, [2, 5, 9, 2, 5, 9])):
            with self.subTest(ammo=initial):
                driver = RoundDriver(ammo=initial)
                for index, identity in enumerate(order):
                    driver.shoot(identity)
                    if index < len(order)-1:
                        self.assertFalse(driver.mission.done)
                        self.assertEqual(driver.mission.status, "RUNNING")
                    if index == 2:
                        # Opening the next round may already mark records
                        # PENDING; cumulative completion must still be three.
                        after_first_round = driver.mission.summary()
                        self.assertEqual(after_first_round["completed_count"], 3)
                        self.assertTrue(all(row["successful_hits"] == 1 for row in after_first_round["targets"]))
                result = driver.mission.summary()
                self.assertEqual(driver.mission.status, "SUCCESS")
                self.assertEqual(result["ammo_remaining"], 0)
                self.assertEqual([shot["intended_target_id"] for shot in result["shots"]], order)
                self.assertEqual([shot["round_index"] for shot in result["shots"]],
                                 [index//3+1 for index in range(initial)])
                self.assertEqual([shot["target_attempt"] for shot in result["shots"]],
                                 [index//3+1 for index in range(initial)])
                self.assertEqual([shot["ammo_before"] for shot in result["shots"]], list(range(initial, 0, -1)))
                self.assertEqual([shot["ammo_remaining"] for shot in result["shots"]], list(range(initial-1, -1, -1)))
                self.assertEqual([shot["shot_index"] for shot in result["shots"]], list(range(1, initial+1)))
                rows = {row["id"]: row for row in result["targets"]}
                for identity in (2, 5, 9):
                    self.assertEqual(rows[identity]["attempts"], order.count(identity))
                    self.assertEqual(rows[identity]["successful_hits"], order.count(identity))
                    self.assertFalse(rows[identity]["retired"])
                    expected_shots = [index+1 for index, fired_id in enumerate(order) if fired_id == identity]
                    visits = rows[identity]["visits"]
                    self.assertEqual([visit["shot_index"] for visit in visits], expected_shots)
                    self.assertEqual([visit["round_index"] for visit in visits], list(range(1, len(visits)+1)))
                    self.assertTrue(all(visit["selected_at_s"] <= visit["finished_at_s"] for visit in visits))
                json.dumps(result, allow_nan=False)
                stopped = driver.step()
                self.assertFalse(stopped.should_fire)
                self.assertEqual(stopped.command, RobotMotionCommand())
                self.assertEqual(len(driver.mission.summary()["shots"]), initial)

    def test_zero_one_or_two_rounds_cannot_report_complete_three_target_coverage(self):
        for initial in (0, 1, 2):
            with self.subTest(ammo=initial):
                driver = RoundDriver(ammo=initial)
                if initial == 0:
                    self.assertFalse(driver.step().should_fire)
                for identity in (2, 5)[:initial]:
                    driver.shoot(identity)
                result = driver.mission.summary()
                self.assertEqual(driver.mission.status, "NO_AMMO")
                self.assertEqual(result["ammo_remaining"], 0)
                self.assertEqual(len(result["shots"]), initial)
                self.assertEqual(sum(row["successful_hits"] > 0 for row in result["targets"]), initial)

    def test_cumulative_total_score_is_separate_from_latest_per_target_snapshot(self):
        driver = RoundDriver(ammo=5)
        for identity, ring in zip((2, 5, 9, 2, 5), (10, 8, 7, 6, 9)):
            driver.shoot(identity, ring)
        result = driver.mission.summary()
        rows = {row["id"]: row for row in result["targets"]}
        self.assertEqual({identity: row["ring"] for identity, row in rows.items()}, {2: 6, 5: 9, 9: 7})
        self.assertEqual(result["total_score"], 40)
        self.assertEqual(result["shot_score_sum"], 40)
        self.assertEqual(result["latest_target_score_total"], 22)
        self.assertEqual({identity: row["score_sum"] for identity, row in rows.items()}, {2: 16, 5: 17, 9: 7})
        self.assertEqual([shot["credited_ring"] for shot in result["shots"]], [10, 8, 7, 6, 9])

    def test_four_and_five_ten_ring_shots_total_forty_and_fifty(self):
        for initial in (4, 5):
            with self.subTest(ammo=initial):
                driver = RoundDriver(ammo=initial)
                order = (2, 5, 9, 2, 5)[:initial]
                for index, identity in enumerate(order, 1):
                    driver.shoot(identity, 10)
                    current = driver.mission.summary()
                    self.assertEqual(current["total_score"], index*10)
                    self.assertEqual(current["shot_score_sum"], current["total_score"])
                    self.assertEqual(sum(row["score_sum"] for row in current["targets"]), current["total_score"])
                result = driver.mission.summary()
                self.assertEqual(result["total_score"], 40 if initial == 4 else 50)
                self.assertEqual(result["latest_target_score_total"], 30)
                self.assertEqual(result["ammo_remaining"], 0)
                self.assertEqual(result["status"], "SUCCESS")
                for row in result["targets"]:
                    self.assertEqual(row["ring"], 10)
                    self.assertEqual(row["score_sum"], order.count(row["id"])*10)

    def test_miss_and_wrong_identity_spend_one_round_and_remain_eligible_next_cycle(self):
        driver = RoundDriver(ammo=6)
        driver.shoot(2, result=ShotResult(None, 0, False, reason="outside_rings"))
        driver.shoot(5, result=ShotResult(9, 10, True, reason="wrong_target"))
        driver.shoot(9, 8)
        self.assertEqual(driver.mission.status, "RUNNING")
        first = {row["id"]: row for row in driver.mission.summary()["targets"]}
        self.assertFalse(first[2]["retired"])
        self.assertFalse(first[5]["retired"])
        driver.shoot(2, 7)
        driver.shoot(5, 6)
        driver.shoot(9, 5)
        report = driver.mission.summary()
        self.assertEqual(driver.mission.status, "SUCCESS")
        self.assertEqual([shot["credited_ring"] for shot in report["shots"]], [0, 0, 8, 7, 6, 5])
        self.assertEqual(report["shots"][1]["actual_target_id"], 9)
        self.assertEqual(report["shots"][1]["reason"], "wrong_target_hit")
        rows = {row["id"]: row for row in report["targets"]}
        self.assertEqual({identity: row["successful_hits"] for identity, row in rows.items()}, {2: 1, 5: 1, 9: 2})
        self.assertEqual(report["total_score"], 26)
        self.assertEqual(report["shot_score_sum"], 26)
        self.assertEqual(report["latest_target_score_total"], 18)
        self.assertEqual({identity: row["score_sum"] for identity, row in rows.items()}, {2: 7, 5: 6, 9: 13})

    def test_latest_miss_or_wrong_target_records_zero_without_erasing_hit_history(self):
        driver = RoundDriver(ammo=6)
        driver.shoot(2, 10)
        driver.shoot(5, 9)
        driver.shoot(9, 8)
        driver.shoot(2, result=ShotResult(None, 0, False, reason="miss"))
        driver.shoot(5, result=ShotResult(9, 10, True))
        driver.shoot(9, 7)
        result = driver.mission.summary()
        self.assertEqual(driver.mission.status, "SUCCESS")
        self.assertEqual({row["id"]: row["ring"] for row in result["targets"]}, {2: 0, 5: 0, 9: 7})
        self.assertEqual({row["id"]: row["successful_hits"] for row in result["targets"]}, {2: 1, 5: 1, 9: 2})
        self.assertEqual(result["total_score"], 34)
        self.assertEqual(result["shot_score_sum"], 34)
        self.assertEqual(result["latest_target_score_total"], 7)
        self.assertEqual({row["id"]: row["score_sum"] for row in result["targets"]}, {2: 10, 5: 9, 9: 15})

    def test_retirement_timeout_and_cancel_preserve_scores_from_repeated_real_shots(self):
        for ending in ("fail_active", "finish_timeout", "cancel"):
            with self.subTest(ending=ending):
                driver = RoundDriver(ammo=6)
                for identity in (2, 5, 9, 2):
                    driver.shoot(identity, 10)
                driver.select(5)
                before = driver.mission.summary()
                if ending == "fail_active":
                    driver.mission.fail_active(driver.now+.01, "aim_response_stalled")
                else:
                    getattr(driver.mission, ending)(driver.now+.01)
                after = driver.mission.summary()
                self.assertEqual(after["total_score"], 40)
                self.assertEqual(after["shot_score_sum"], 40)
                self.assertEqual(after["latest_target_score_total"], 30)
                self.assertEqual(after["ammo_remaining"], 2)
                self.assertEqual(after["shots"], before["shots"])
                self.assertEqual({row["id"]: row["score_sum"] for row in after["targets"]}, {2: 20, 5: 10, 9: 10})

    def test_prefire_timeout_retires_target_and_skips_it_without_spending_round(self):
        driver = RoundDriver(round_config(target_timeout_s=.8), ammo=5)
        driver.step(detections=[])
        driver.now = .81
        timed_out = driver.mission.update(frame(driver.now), [], feedback(driver.now), driver.now, driver.ammo)
        self.assertFalse(timed_out.should_fire)
        self.assertEqual(timed_out.state, "SWITCHING")
        row = next(row for row in driver.mission.summary()["targets"] if row["id"] == 2)
        self.assertTrue(row["retired"])
        self.assertEqual(row["attempts"], 0)
        self.assertEqual(row["reason"], "target_timeout")
        self.assertEqual(row["retired_round"], 1)
        self.assertEqual(row["retired_at_s"], .81)
        self.assertIsNone(row["visits"][-1]["shot_index"])
        for identity in (5, 9, 5, 9, 5):
            driver.shoot(identity)
        result = driver.mission.summary()
        self.assertEqual([shot["intended_target_id"] for shot in result["shots"]], [5, 9, 5, 9, 5])
        self.assertEqual(result["ammo_remaining"], 0)
        self.assertEqual(driver.mission.status, "NO_AMMO")

    def test_aim_failure_retires_one_identity_without_discharging_or_resetting_ammo(self):
        driver = RoundDriver(ammo=4)
        driver.select(2)
        failed = driver.mission.fail_active(driver.now, "aim_response_stalled")
        self.assertFalse(failed.should_fire)
        report = driver.mission.summary()
        self.assertEqual(report["ammo_remaining"], 4)
        self.assertEqual(report["shots"], [])
        row = next(row for row in report["targets"] if row["id"] == 2)
        self.assertTrue(row["retired"])
        self.assertEqual(row["attempts"], 0)
        for identity in (5, 9, 5, 9):
            driver.shoot(identity)
        self.assertEqual(driver.mission.status, "NO_AMMO")
        self.assertEqual(driver.mission.summary()["ammo_remaining"], 0)

    def test_all_targets_retired_stop_with_remaining_ammo_and_bounded_status(self):
        for prior_hit in (False, True):
            with self.subTest(prior_hit=prior_hit):
                driver = RoundDriver(ammo=5)
                order = (2, 5, 9)
                if prior_hit:
                    driver.shoot(2, 8)
                    order = (5, 9, 2)
                for identity in order:
                    driver.select(identity)
                    driver.mission.fail_active(driver.now, "aim_response_stalled")
                self.assertTrue(driver.mission.done)
                self.assertEqual(driver.mission.status, "PARTIAL" if prior_hit else "FAILED")
                report = driver.mission.summary()
                self.assertEqual(report["ammo_remaining"], 4 if prior_hit else 5)
                self.assertTrue(all(row["retired"] for row in report["targets"]))
                for _ in range(8):
                    self.assertFalse(driver.step().should_fire)
                self.assertEqual(len(driver.mission.summary()["shots"]), 1 if prior_hit else 0)

    def test_every_revisit_gets_new_controller_and_complete_dwell_not_old_fire_token(self):
        driver = RoundDriver(round_config(ids=(2,)), ammo=2)
        driver.arm(2)
        first_timestamp = driver.now
        controller = driver.mission._controller
        self.assertTrue(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 1))
        driver.ammo = 1
        self.assertFalse(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 0))
        selected = driver.select(2)
        self.assertIsNot(driver.mission._controller, controller)
        self.assertFalse(selected.should_fire)
        self.assertFalse(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 0))
        stale = driver.mission.update(frame(first_timestamp), [],
                                     feedback(driver.now), driver.now, driver.ammo)
        self.assertFalse(stale.should_fire)
        self.assertFalse(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 0))
        for _ in range(2):
            self.assertFalse(driver.step().should_fire)
        driver.shoot(2, 7)
        self.assertEqual(driver.mission.status, "SUCCESS")
        self.assertEqual(driver.mission.summary()["targets"][0]["attempts"], 2)

    def test_rejected_ledger_callbacks_cannot_create_extra_attempts_or_rounds(self):
        driver = RoundDriver(round_config(ids=(2,)), ammo=2)
        driver.arm(2)
        before = driver.mission.summary()
        for remaining in (2, 3, -1, 0):
            with self.subTest(remaining=remaining):
                self.assertFalse(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, remaining))
                self.assertEqual(driver.mission.summary(), before)
        self.assertTrue(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 1))
        self.assertFalse(driver.mission.record_shot(2, ShotResult(2, 10, True), driver.now, 0))
        self.assertEqual(len(driver.mission.summary()["shots"]), 1)

    def test_repeated_misses_end_on_fifth_real_debit_without_retirement_or_infinite_rounds(self):
        driver = RoundDriver(round_config(ids=(2,)), ammo=5)
        for _ in range(5):
            driver.shoot(2, result=ShotResult(None, 0, False, reason="miss"))
        report = driver.mission.summary()
        self.assertEqual(driver.mission.status, "NO_AMMO")
        self.assertEqual(report["ammo_remaining"], 0)
        self.assertEqual(len(report["shots"]), 5)
        self.assertEqual([shot["round_index"] for shot in report["shots"]], [1, 2, 3, 4, 5])
        self.assertEqual(report["targets"][0]["attempts"], 5)
        self.assertEqual(report["targets"][0]["successful_hits"], 0)
        self.assertFalse(report["targets"][0]["retired"])

    def test_ammo_cannot_rise_or_fall_outside_the_committed_shot_transaction(self):
        driver = RoundDriver(ammo=5)
        driver.arm(2)
        for changed in (6, 4, True, -1, 5.0):
            with self.subTest(ammo=changed):
                driver.now = round(driver.now+.01, 8)
                result = driver.mission.update(frame(driver.now),
                    [detection(2, driver.now, driver.offsets[2])], feedback(driver.now), driver.now, changed)
                self.assertEqual(result.state, "WAIT")
                self.assertFalse(result.should_fire)
                self.assertEqual(result.command, RobotMotionCommand())
                self.assertEqual(driver.mission.summary()["ammo_remaining"], 5)
                self.assertEqual(driver.mission.summary()["shots"], [])

    def test_preflight_authorization_is_read_only_and_requires_exact_live_frame_and_ammo(self):
        driver = RoundDriver(ammo=5)
        driver.arm(2)
        before = driver.mission.summary()
        self.assertTrue(driver.mission.authorize_shot(2, driver.now, 5, frame_timestamp=driver.now))
        cases = ((5, driver.now, 5, driver.now), (2, driver.now-.01, 5, driver.now),
                 (2, driver.now+.2, 5, driver.now), (2, driver.now, 4, driver.now),
                 (2, driver.now, 5, driver.now-.01), (2, driver.now, True, driver.now))
        for identity, now, ammo, captured in cases:
            with self.subTest(identity=identity, now=now, ammo=ammo, captured=captured):
                self.assertFalse(driver.mission.authorize_shot(identity, now, ammo, frame_timestamp=captured))
                self.assertEqual(driver.mission.summary(), before)

    def test_policy_is_explicit_and_old_single_pass_keeps_declared_order(self):
        for explicit in (False, True):
            with self.subTest(explicit=explicit):
                source = config(ids=(9, 2, 5), **({"shooting_policy": "single_pass"} if explicit else {}))
                driver = RoundDriver(source, ammo=5)
                for identity in (9, 2, 5):
                    driver.shoot(identity)
                self.assertEqual(driver.mission.status, "SUCCESS")
                self.assertEqual(driver.mission.summary()["ammo_remaining"], 2)
                self.assertEqual([row["intended_target_id"] for row in driver.mission.summary()["shots"]], [9, 2, 5])
        for invalid in ("repeat_forever", "", None, True):
            with self.subTest(policy=invalid), self.assertRaises(ValueError):
                source = config(shooting_policy=invalid)
                MultiTargetMission(source)

    def test_global_timeout_closes_active_visit_without_spending_or_replacing_earlier_score(self):
        driver = RoundDriver(ammo=5)
        driver.shoot(2, 8)
        driver.select(5)
        before = driver.mission.summary()
        ended_at = driver.now+.01
        driver.mission.finish_timeout(ended_at)
        after = driver.mission.summary()
        self.assertEqual(after["status"], "PARTIAL")
        self.assertEqual(after["termination_reason"], "global_timeout")
        self.assertEqual(after["ammo_remaining"], before["ammo_remaining"])
        self.assertEqual(after["shots"], before["shots"])
        self.assertEqual(after["total_score"], 8)
        self.assertEqual(after["shot_score_sum"], 8)
        rows = {row["id"]: row for row in after["targets"]}
        previous = {row["id"]: row for row in before["targets"]}
        self.assertEqual(rows[2]["visits"], previous[2]["visits"])
        self.assertEqual(rows[9]["visits"], [])
        visit = rows[5]["visits"][-1]
        self.assertEqual(visit["state"], "FAILED")
        self.assertEqual(visit["reason"], "global_timeout")
        self.assertEqual(visit["finished_at_s"], ended_at)
        self.assertIsNone(visit["shot_index"])
        self.assertEqual(rows[5]["attempts"], 0)
        self.assertFalse(rows[5]["retired"])
        self.assertIsNone(driver.mission.active_target_id)
        self.assertIsNone(driver.mission._armed)
        driver.mission.finish_timeout(ended_at+.1)
        self.assertEqual(driver.mission.summary(), after)

    def test_cancel_closes_only_open_visit_and_keeps_prior_committed_ledger(self):
        driver = RoundDriver(ammo=5)
        driver.shoot(2, 8)
        driver.select(5)
        before = driver.mission.summary()
        ended_at = driver.now+.01
        driver.mission.cancel(ended_at, "operator_requested_stop")
        after = driver.mission.summary()
        self.assertEqual(after["status"], "CANCELLED")
        self.assertEqual(after["termination_reason"], "operator_requested_stop")
        self.assertEqual(after["ammo_remaining"], before["ammo_remaining"])
        self.assertEqual(after["shots"], before["shots"])
        self.assertEqual(after["total_score"], 8)
        self.assertEqual(after["shot_score_sum"], 8)
        rows = {row["id"]: row for row in after["targets"]}
        previous = {row["id"]: row for row in before["targets"]}
        self.assertEqual(rows[2]["visits"], previous[2]["visits"])
        self.assertEqual(rows[9]["visits"], [])
        visit = rows[5]["visits"][-1]
        self.assertEqual(visit["state"], "FAILED")
        self.assertEqual(visit["reason"], "operator_requested_stop")
        self.assertEqual(visit["finished_at_s"], ended_at)
        self.assertIsNone(visit["shot_index"])
        self.assertEqual(rows[5]["attempts"], 0)
        self.assertFalse(rows[5]["retired"])
        self.assertIsNone(driver.mission.active_target_id)
        self.assertIsNone(driver.mission._armed)
        driver.mission.cancel(ended_at+.1, "second_cancel")
        self.assertEqual(driver.mission.summary(), after)


if __name__ == "__main__":
    unittest.main()
