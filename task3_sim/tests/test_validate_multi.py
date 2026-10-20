"""Independent validation must reject forged identities and unsafe fire logs."""

from __future__ import annotations

import copy
import json
import unittest

from task3_sim.__main__ import PACKAGE
from task3_sim.validate_multi import assess, scenarios


def evidence():
    ids = [2, 5, 9]
    config = {"targets": [{"id": value, "simulation": {"target_center_world_m": [2.7, index, .5]}}
                           for index, value in enumerate(ids)],
              "simulation": {"ammo_count": 5, "target_center_world_m": [2.7, 0, .5]},
              "control": {"max_frame_age_s": .15, "max_feedback_age_s": .15,
                          "max_gyro_rad_s": .02, "max_speed_m_s": .01, "aim_tolerance_m": .008},
              "tag": {"min_decision_margin": 40, "max_reprojection_error_px": 2}}
    rows, physical, events, targets = [], [], [], []
    for index, value in enumerate(ids):
        timestamp = float(index+1)
        point = [2.7, index+.005, .5]
        row = {"shot_index": index+1, "timestamp": timestamp, "intended_target_id": value,
               "actual_target_id": value, "correct_target_hit": True, "hit": True, "ring": 10,
               "credited_ring": 10, "point_world_m": point, "ammo_before": 5-index,
               "ammo_remaining": 4-index}
        raw = {"sequence": index+1, "timestamp_s": timestamp, "intended_target_id": value,
               "actual_target_id": value, "hit": True, "ring": 10, "point_world_m": point,
               "ammo_before": 5-index, "ammo_after": 4-index}
        event = {"time_s": timestamp, "capture_timestamp_s": timestamp, "frame_timestamp_s": timestamp,
                 "frame_age_s": 0, "frame_delivered": True, "active_target_id": value,
                 "state": "READY", "shot": {"target_id": value}, "shot_record": raw,
                 "command": {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": 0, "pitch_rate_rad_s": 0},
                 "shot_accepted": True, "geometry": {"reachable": True, "error_norm_m": .005},
                 "feedback": {"valid": True, "timestamp": timestamp, "gyro_rad_s": [0, 0, 0],
                              "velocity_m_s": [0, 0, 0]},
                 "detections": [{"tag_id": value, "hamming": 0, "decision_margin": 100,
                                 "reprojection_error_px": .1, "timestamp": timestamp}]}
        rows.append(row)
        physical.append(raw)
        events.append(event)
        targets.append({"id": value, "state": "COMPLETED", "attempts": 1, "ring": 10})
    summary = {"status": "SUCCESS", "targets": targets, "shots": physical, "shot_count": 3,
               "ammo_initial": 5, "ammo_remaining": 2, "model_sources_unchanged": True,
               "shutdown_completed": True, "perception_closed": True, "cleanup_errors": [],
               "transport": {"pushed": 3, "delivered": 3},
               "mission": {"targets": targets, "shots": rows, "total_score": 30}}
    expected = {"status": "SUCCESS", "shot_count": 3, "completed_ids": ids,
                "min_ring": 9, "max_actual_miss_m": .02}
    return summary, events, config, expected


class MultiValidationTests(unittest.TestCase):
    def test_accepts_consistent_fresh_evidence(self):
        summary, events, config, expected = evidence()
        checks, metrics = assess(summary, events, 0, config, expected)
        self.assertTrue(all(checks.values()), checks)
        self.assertLess(metrics["max_actual_miss_m"], .006)

    def test_rejects_stale_frame_even_when_marked_ready(self):
        summary, events, config, expected = evidence()
        events[0].update(frame_timestamp_s=.7, frame_age_s=.3)
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["fresh_observation_at_every_shot"])
        self.assertFalse(checks["decoded_detection_quality_at_every_shot"])

    def test_rejects_duplicate_target_and_wrong_actual_id(self):
        summary, events, config, expected = evidence()
        summary["mission"]["shots"][1]["intended_target_id"] = 2
        summary["mission"]["shots"][1]["actual_target_id"] = 42
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["no_duplicate_or_unknown_shots"])
        self.assertFalse(checks["same_id_hit_and_credit"])
        self.assertFalse(checks["shot_events_match_ledger"])

    def test_rejects_incorrect_ammo_and_false_geometric_score(self):
        summary, events, config, expected = evidence()
        summary["mission"]["shots"][0]["ammo_remaining"] = 3
        summary["mission"]["shots"][1]["point_world_m"] = [2.7, 1.2, .5]
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["ammo_ledger"])
        self.assertFalse(checks["actual_miss_limit"])

    def test_rejects_absent_duplicate_or_low_quality_detection(self):
        for change in ("absent", "duplicate", "margin", "hamming"):
            with self.subTest(change=change):
                summary, events, config, expected = evidence()
                detection = events[0]["detections"][0]
                if change == "absent":
                    events[0]["detections"] = []
                elif change == "duplicate":
                    events[0]["detections"].append(copy.deepcopy(detection))
                elif change == "margin":
                    detection["decision_margin"] = 1
                else:
                    detection["hamming"] = 1
                checks, _ = assess(summary, events, 0, config, expected)
                self.assertFalse(checks["matching_decoded_detection_at_every_shot"]
                                 and checks["decoded_detection_quality_at_every_shot"])

    def test_rejects_repeated_frame_and_moving_feedback(self):
        summary, events, config, expected = evidence()
        events[1]["frame_timestamp_s"] = events[0]["frame_timestamp_s"]
        events[0]["feedback"]["gyro_rad_s"] = [0, 0, .1]
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["delivered_acquisitions_strictly_increase"])
        self.assertFalse(checks["fresh_stationary_feedback_at_every_shot"])

    def test_unknown_tag_scenario_requires_actual_observation(self):
        summary, events, config, expected = evidence()
        expected["require_unknown_id"] = 42
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["unknown_tag_actually_detected"])
        events[0]["detections"].append({"tag_id": 42})
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertTrue(all(checks.values()), checks)

    def test_no_delivery_cannot_continue_robot_motion(self):
        summary, events, config, expected = evidence()
        events.append({"time_s": 4.0, "capture_timestamp_s": 4.0,
                       "frame_timestamp_s": None, "frame_age_s": None,
                       "frame_delivered": False, "detections": [],
                       "command": {"vx_m_s": 0, "vy_m_s": 0, "yaw_rate_rad_s": .1,
                                   "pitch_rate_rad_s": 0}})
        summary["transport"]["pushed"] = 4
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["no_motion_without_fresh_observation"])
        self.assertTrue(checks["transport_delivery_counts_match_events"])

    def test_random_configurations_are_repeatable_saved_parameters(self):
        base = json.loads((PACKAGE/"config/multi_fixed.json").read_text(encoding="utf-8"))
        first = list(scenarios(base, random_cases=5))
        second = list(scenarios(base, random_cases=5))
        self.assertEqual(first, second)
        random = [case for case in first if case[4]["kind"] == "seeded_random"]
        self.assertEqual(len(random), 5)
        for _, config, expected, _, parameters in random:
            self.assertEqual(config["simulation"]["initial_position_m"][:2],
                             [parameters["initial_x_m"], parameters["initial_y_m"]])
            self.assertEqual(config["simulation"]["initial_rpy_rad"][1:],
                             [parameters["initial_pitch_rad"], parameters["initial_yaw_rad"]])
            self.assertEqual(expected["status"], "SUCCESS")

    def test_missing_surround_target_is_not_just_placed_behind_robot(self):
        base = json.loads((PACKAGE/"config/multi_fixed.json").read_text(encoding="utf-8"))
        cases = {case[0]: case for case in scenarios(base, random_cases=0)}
        _, missing, expected, deadline, _ = cases["middle_target_missing"]
        self.assertEqual(missing["targets"][1]["simulation"]["target_center_world_m"],
                         base["targets"][1]["simulation"]["target_center_world_m"])
        self.assertAlmostEqual(missing["targets"][1]["simulation"]["target_yaw_rad"], 3.141592653589793)
        self.assertEqual(expected["require_unseen_id"], base["targets"][1]["id"])
        self.assertGreater(deadline, base["mission"]["target_timeout_s"]*3)

    def test_missing_target_must_actually_be_absent_from_detections(self):
        summary, events, config, expected = evidence()
        expected["require_unseen_id"] = 42
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertTrue(checks["missing_id_never_decoded"])
        events[0]["detections"].append({"tag_id": 42})
        checks, _ = assess(summary, events, 0, config, expected)
        self.assertFalse(checks["missing_id_never_decoded"])


if __name__ == "__main__":
    unittest.main()
