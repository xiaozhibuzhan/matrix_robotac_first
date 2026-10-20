"""Repeated-shot audit fixtures verify accounting, never native acceptance."""

import copy
import json
import unittest

from task3_sim.__main__ import PACKAGE
from task3_sim.tests.test_validate_multi import evidence
from task3_sim.validate_multi import assess, repeat_ledger_audit, round_robin_order, scenarios


def repeated_evidence(ammo=5, *, perfect=False):
    summary, _, config, expected = evidence()
    config["mission"] = {"shooting_policy": "round_robin"}
    config["simulation"]["ammo_count"] = ammo
    config["control"]["stable_duration_s"] = .4
    ids = sorted(row["id"] for row in config["targets"])
    rows, physical, events = [], [], []
    targets = {identity: {"id": identity, "state": "FAILED", "reason": "no_ammo", "ring": 0,
        "attempts": 0, "successful_hits": 0, "score_sum": 0, "round_index": 0,
        "retired": False, "retired_round": None, "retired_at_s": None, "visits": []} for identity in ids}
    for index, identity in enumerate(round_robin_order(ids, ammo)):
        timestamp = 2.0 + index*2
        ring = 10 if perfect else 9 if index == 3 else 10
        point = [2.7, ids.index(identity)+.005, .5]
        round_index = index//len(ids)+1
        targets[identity]["attempts"] += 1
        targets[identity]["successful_hits"] += 1
        targets[identity]["score_sum"] += ring
        targets[identity].update(state="COMPLETED", reason="matching_target_hit", ring=ring, round_index=round_index)
        targets[identity]["visits"].append({"round_index": round_index, "selected_at_s": timestamp-.5,
            "finished_at_s": timestamp, "state": "COMPLETED", "reason": "matching_target_hit", "shot_index": index+1})
        row = {"shot_index": index+1, "timestamp": timestamp, "intended_target_id": identity,
            "actual_target_id": identity, "correct_target_hit": True, "hit": True, "ring": ring,
            "credited_ring": ring, "point_world_m": point, "ammo_before": ammo-index,
            "ammo_remaining": ammo-index-1, "round_index": round_index,
            "target_attempt": targets[identity]["attempts"]}
        raw = {"sequence": index+1, "timestamp_s": timestamp, "intended_target_id": identity,
            "actual_target_id": identity, "hit": True, "ring": ring, "physical_ring": ring,
            "point_world_m": point, "ammo_before": ammo-index, "ammo_after": ammo-index-1}
        rows.append(row)
        physical.append(raw)
        for step in range(6):
            now = timestamp-.5+step*.1
            event = {"time_s": now, "capture_timestamp_s": now, "frame_timestamp_s": now,
                "frame_age_s": 0, "frame_delivered": True, "active_target_id": identity,
                "state": "READY" if step == 5 else "SETTLING", "round_index": round_index,
                "command": {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": 0, "pitch_rate_rad_s": 0},
                "geometry": {"reachable": True, "error_norm_m": .005},
                "feedback": {"valid": True, "timestamp": now, "gyro_rad_s": [0, 0, 0], "velocity_m_s": [0, 0, 0]},
                "detections": [{"tag_id": identity, "hamming": 0, "decision_margin": 100,
                    "reprojection_error_px": .1, "timestamp": now}]}
            if step == 5:
                event.update(shot={"target_id": identity}, shot_record=raw, shot_accepted=True,
                             mission_shot_record=copy.deepcopy(row))
            events.append(event)
    status = "SUCCESS" if ammo >= len(ids) else "NO_AMMO"
    summary.update(status=status, targets=list(targets.values()), shots=physical, shot_count=ammo,
        ammo_initial=ammo, ammo_remaining=0, transport={"pushed": len(events), "delivered": len(events)})
    summary["mission"] = {"status": status, "targets": list(targets.values()), "shots": rows,
        "total_score": sum(row["credited_ring"] for row in rows),
        "latest_target_score_total": sum(row["ring"] for row in targets.values()),
        "shot_score_sum": sum(row["credited_ring"] for row in rows), "coverage_complete": ammo >= len(ids),
        "shooting_policy": "round_robin", "firing_order": ids, "round_index": max(1, (ammo+len(ids)-1)//len(ids)),
        "completed_rounds": ammo//len(ids), "termination_reason": "ammo_exhausted"}
    expected.update(status=status, shot_count=ammo, completed_ids=ids[:min(ammo, len(ids))],
                    shot_order=round_robin_order(ids, ammo))
    return summary, events, config, expected


class RepeatAcceptanceTests(unittest.TestCase):
    def test_full_order_and_accounting_for_one_three_five_and_six_shots(self):
        for ammo in (1, 3, 5, 6):
            with self.subTest(ammo=ammo):
                summary, events, config, expected = repeated_evidence(ammo)
                checks, _ = assess(summary, events, 0 if ammo >= 3 else 2, config, expected)
                self.assertTrue(all(checks.values()), [key for key, value in checks.items() if not value])

    def test_declaration_order_does_not_change_numeric_firing_order(self):
        summary, events, config, expected = repeated_evidence()
        config["targets"].reverse()
        checks, metrics = assess(summary, events, 0, config, expected)
        self.assertTrue(all(checks.values()), checks)
        self.assertEqual(metrics["repeat_shooting"]["actual_shot_order"], [2, 5, 9, 2, 5])

    def test_each_ledger_entry_matters_even_when_last_scores_still_look_correct(self):
        summary, events, config, _ = repeated_evidence()
        summary["mission"]["shots"][0]["credited_ring"] = 0
        checks, _ = repeat_ledger_audit(summary, events, config)
        self.assertTrue(checks["round_robin_last_score_per_target"])
        self.assertFalse(checks["round_robin_per_shot_credit_matches_physical_outcome"])
        self.assertFalse(checks["round_robin_shot_score_sum"])
        self.assertFalse(checks["round_robin_event_mission_rows_match"])

    def test_repeated_identity_in_same_round_or_wrong_global_order_is_rejected(self):
        for field, value in (("round_index", 1), ("target_attempt", 1), ("intended_target_id", 9)):
            with self.subTest(field=field):
                summary, events, config, _ = repeated_evidence()
                summary["mission"]["shots"][3][field] = value
                checks, _ = repeat_ledger_audit(summary, events, config)
                self.assertFalse(checks["round_robin_one_shot_per_visit"]
                                 and checks["round_robin_complete_expected_shot_sequence"])

    def test_insufficient_coverage_cannot_be_promoted_to_success(self):
        summary, events, config, _ = repeated_evidence(1)
        summary["status"] = "SUCCESS"
        summary["mission"]["coverage_complete"] = True
        checks, _ = repeat_ledger_audit(summary, events, config)
        self.assertFalse(checks["round_robin_coverage_is_actual_correct_hits"])
        self.assertFalse(checks["round_robin_success_requires_empty_magazine_and_coverage"])
        self.assertFalse(checks["round_robin_no_ammo_without_coverage"])

    def test_last_score_and_all_shot_score_sum_are_distinct(self):
        summary, events, config, _ = repeated_evidence()
        self.assertEqual(summary["mission"]["total_score"], 49)
        self.assertEqual(summary["mission"]["latest_target_score_total"], 29)
        self.assertEqual(summary["mission"]["shot_score_sum"], 49)
        summary["mission"]["total_score"] = 29
        checks, _ = repeat_ledger_audit(summary, events, config)
        self.assertTrue(checks["round_robin_last_score_per_target"])
        self.assertFalse(checks["round_robin_total_score_is_all_shots"])
        self.assertFalse(checks["round_robin_shot_score_sum"])

    def test_four_and_five_actual_shots_credit_forty_and_fifty(self):
        for ammo in (4, 5):
            with self.subTest(ammo=ammo):
                summary, events, config, expected = repeated_evidence(ammo, perfect=True)
                checks, metrics = assess(summary, events, 0, config, expected)
                self.assertTrue(all(checks.values()), [key for key, value in checks.items() if not value])
                self.assertEqual(len(summary["shots"]), ammo)
                self.assertEqual(len(summary["mission"]["shots"]), ammo)
                self.assertEqual(len([event for event in events if event.get("shot_record")]), ammo)
                self.assertEqual(summary["mission"]["total_score"], ammo*10)
                self.assertEqual(summary["mission"]["latest_target_score_total"], 30)
                self.assertEqual(sum(row["score_sum"] for row in summary["mission"]["targets"]), ammo*10)
                self.assertEqual(metrics["repeat_shooting"]["total_score"], ammo*10)

    def test_reported_fifty_with_only_four_physical_shots_is_rejected(self):
        summary, events, config, expected = repeated_evidence(5, perfect=True)
        summary["shots"].pop()
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["one_fire_event_per_ledger_row"])
        self.assertFalse(checks["round_robin_complete_expected_shot_sequence"])
        self.assertFalse(checks["round_robin_per_shot_credit_matches_physical_outcome"])

    def test_latest_score_total_and_per_target_cumulative_score_are_independently_audited(self):
        for field in ("latest_target_score_total", "score_sum"):
            with self.subTest(field=field):
                summary, events, config, _ = repeated_evidence(5, perfect=True)
                if field == "latest_target_score_total":
                    summary["mission"][field] = 50
                else:
                    summary["mission"]["targets"][0][field] = 10
                checks, _ = repeat_ledger_audit(summary, events, config)
                self.assertTrue(checks["round_robin_total_score_is_all_shots"])
                self.assertFalse(checks["round_robin_last_score_per_target"]
                                 and checks["round_robin_target_score_sums"])

    def test_prior_round_stable_images_cannot_authorize_second_round_immediate_shot(self):
        summary, events, config, _ = repeated_evidence()
        fourth_time = summary["mission"]["shots"][3]["timestamp"]
        events[:] = [event for event in events if event.get("active_target_id") != 2
                     or event["time_s"] < fourth_time-.5 or event["time_s"] == fourth_time]
        checks, metrics = repeat_ledger_audit(summary, events, config)
        self.assertFalse(checks["round_robin_every_shot_has_new_independent_dwell"])
        self.assertFalse(metrics["per_shot_dwell"][3]["valid"])

    def test_second_round_invalid_feedback_revokes_whole_new_dwell(self):
        summary, events, config, _ = repeated_evidence()
        fourth_time = summary["mission"]["shots"][3]["timestamp"]
        next(event for event in events if event["time_s"] == fourth_time-.2)["feedback"]["valid"] = False
        checks, _ = repeat_ledger_audit(summary, events, config)
        self.assertFalse(checks["round_robin_every_shot_has_new_independent_dwell"])

    def test_duplicate_or_missing_visit_and_forged_counters_are_rejected(self):
        for action in ("duplicate", "missing", "rounds", "hits", "retire"):
            with self.subTest(action=action):
                summary, events, config, _ = repeated_evidence()
                row = summary["mission"]["targets"][0]
                if action == "duplicate":
                    row["visits"].append(copy.deepcopy(row["visits"][0]))
                elif action == "missing":
                    row["visits"].pop()
                elif action == "rounds":
                    summary["mission"]["completed_rounds"] = 2
                elif action == "hits":
                    row["successful_hits"] = 1
                else:
                    row["retired"] = True
                checks, _ = repeat_ledger_audit(summary, events, config)
                self.assertFalse(all(checks.values()))

    def test_predeclared_repeat_scenarios_cover_magazines_order_and_second_round_loss(self):
        base = json.loads((PACKAGE / "config/multi_fixed.json").read_text(encoding="utf-8"))
        base.setdefault("mission", {})["shooting_policy"] = "round_robin"
        base["simulation"]["ammo_count"] = 3
        cases = {case[0]: case for case in scenarios(base, random_cases=0)}
        self.assertEqual(cases["three_targets"][2]["shot_order"], [2, 5, 9])
        self.assertEqual(cases["magazine_1_shots"][2]["status"], "NO_AMMO")
        self.assertEqual(cases["magazine_5_shots"][1]["simulation"]["ammo_count"], 5)
        self.assertEqual(cases["magazine_5_shots"][2]["shot_order"], [2, 5, 9, 2, 5])
        self.assertEqual(cases["magazine_6_shots"][2]["shot_order"], [2, 5, 9, 2, 5, 9])
        self.assertTrue(cases["second_round_frame_loss"][2]["require_second_round_drop"])


if __name__ == "__main__":
    unittest.main()
