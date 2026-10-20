"""Direction-prior search is independent of rendered identity and fire order."""
from __future__ import annotations

import copy
from itertools import permutations
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from task3_sim.backend import MujocoRobot
from task3_sim.multi_config import public_mission_config
from task3_sim.multi_perception import MultiTagPerception
from task3_sim.multi_scene import build_multi_scene
from task3_sim.validate_multi import direction_route_audit, scenarios, tag_only_distractor


PACKAGE = Path(__file__).resolve().parents[1]


def config():
    return json.loads((PACKAGE / "config/multi_fixed.json").read_text(encoding="utf-8"))


class FixedDirectionsRenderTests(unittest.TestCase):
    def test_native_rgb_all_six_identity_direction_arrangements(self):
        original = config()
        slots = [copy.deepcopy(entry["simulation"]) for entry in original["targets"]]
        model_root = PACKAGE.parent / "src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root / "xgb.xml").is_file():
            self.skipTest("original xgb model not present")
        for placement in permutations(range(3)):
            with self.subTest(placement=placement), tempfile.TemporaryDirectory(prefix="fixed_rgb_") as directory:
                current = copy.deepcopy(original)
                current["distractors"] = []
                for index, slot in enumerate(placement):
                    current["targets"][index]["simulation"] = copy.deepcopy(slots[slot])
                model, provenance = build_multi_scene(current, model_root, Path(directory))
                self.assertFalse(provenance["official_files_modified"])
                robot = MujocoRobot(model, current)
                robot.Connect()
                robot.StandUp()
                try:
                    with MultiTagPerception(public_mission_config(current)) as perception:
                        for entry in current["targets"]:
                            angle = entry["simulation"]["target_yaw_rad"]
                            robot.rpy[2] = angle
                            robot._sync()
                            capture = robot.capture()
                            actual = perception.detect(capture)
                            matched = [item for item in actual if item.tag_id == entry["id"]]
                            self.assertEqual(len(matched), 1, (entry["id"], [item.tag_id for item in actual]))
                            self.assertEqual(matched[0].hamming, 0)
                            self.assertGreater(matched[0].decision_margin, 40)
                            self.assertLess(matched[0].reprojection_error_px, 2)
                            self.assertGreater(float(capture.rgb[100, 480].mean()), 120)
                finally:
                    robot.close()

    def test_visible_front_distractor_is_decoded_but_not_public_target(self):
        current = config()
        self.assertEqual(current["distractors"], [])
        current["distractors"] = [tag_only_distractor()]
        model_root = PACKAGE.parent / "src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root / "xgb.xml").is_file():
            self.skipTest("original xgb model not present")
        with tempfile.TemporaryDirectory(prefix="fixed_distractor_") as directory:
            model, provenance = build_multi_scene(current, model_root, Path(directory))
            robot = MujocoRobot(model, current)
            robot.Connect()
            robot.StandUp()
            try:
                public = public_mission_config(current)
                with MultiTagPerception(public) as perception:
                    actual = {item.tag_id for item in perception.detect(robot.capture())}
                self.assertTrue({5, 42}.issubset(actual), actual)
                self.assertNotIn(42, {entry["id"] for entry in public["targets"]})
                self.assertEqual(provenance["scoring_board_count"], 3)
                self.assertEqual(provenance["visual_only_distractor_tag_count"], 1)
                self.assertEqual(provenance["target_textures"]["42"]["kind"], "tag_only")
                self.assertAlmostEqual(provenance["target_textures"]["42"]["patch_side_m"], .20)
                import mujoco
                from task3_sim.multi_scoring import MultiRayScorer
                self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "task3_target_collision_42"), -1)
                self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "task3_target_42"), -1)
                scorer = MultiRayScorer(model, robot.data, current)
                self.assertEqual({value[0] for value in scorer.boards.values()}, {2, 5, 9})
            finally:
                robot.close()

    def test_acceptance_declares_identity_swaps_and_orders_before_running(self):
        for policy, count in (("single_pass", 15), ("round_robin", 20)):
            with self.subTest(policy=policy):
                original = config()
                original.setdefault("mission", {})["shooting_policy"] = policy
                cases = list(scenarios(original, random_cases=0))
                self.assertEqual(len(cases), count)
                self.assertEqual(sum(name.startswith("identity_directions_") for name, *_ in cases), 2)
                self.assertEqual(sum(name.startswith("shooting_order_") for name, *_ in cases), 2)
                for name, current, expected, _, _ in cases:
                    self.assertEqual(len(current["distractors"]), 1 if name == "unknown_tag_ignored" else 0)
                    if name.startswith("identity_directions_"):
                        self.assertEqual([entry["id"] for entry in current["targets"]], [2, 5, 9])
                        self.assertNotEqual([entry["simulation"] for entry in current["targets"]],
                                            [entry["simulation"] for entry in original["targets"]])
                    if name.startswith("shooting_order_"):
                        self.assertNotEqual([entry["id"] for entry in current["targets"]], [2, 5, 9])
                        self.assertEqual(set(expected["completed_ids"]), {2, 5, 9})


class DirectionRouteAuditTests(unittest.TestCase):
    def evidence(self):
        current = config()
        probe = {"timestamp": 0.0, "yaw_rad": -np.pi / 2,
                 "source": "public_direction_prior_identity_unknown", "direction_index": 2}
        summary = {"mission": {"search": {"mode": "fixed_directions", "initial_yaw_rad": 0,
                    "observed_bearings": {"5": {"source": "image_pose_with_timestamped_robot_feedback"}}},
                    "targets": [{"id": 2, "search_travel_rad": .2, "search_probes": [probe]}]}}
        event = {"active_target_id": 2, "time_s": .1, "state": "SEARCHING",
                 "feedback": {"rpy_rad": [0, 0, 0]}, "command": {
                     "vx_m_s": 0, "vy_m_s": 0, "pitch_rate_rad_s": 0, "yaw_rate_rad_s": -.35}}
        return summary, [event], current

    def test_accepts_clockwise_shortest_turn_and_rejects_counterclockwise_loop(self):
        summary, events, current = self.evidence()
        checks, _ = direction_route_audit(summary, events, current, {})
        self.assertTrue(all(checks.values()), checks)
        events[0]["command"]["yaw_rate_rad_s"] = .35
        checks, _ = direction_route_audit(summary, events, current, {})
        self.assertFalse(checks["search_turns_follow_shortest_angular_error"])

    def test_rejects_hidden_identity_map_wrong_prior_and_extra_turn(self):
        summary, events, current = self.evidence()
        summary["mission"]["search"]["observed_bearings"]["42"] = {"source": "scene_truth"}
        summary["mission"]["targets"][0]["search_probes"][0]["yaw_rad"] = .25
        summary["mission"]["targets"][0]["search_travel_rad"] = 2 * np.pi + .1
        checks, _ = direction_route_audit(summary, events, current, {})
        self.assertFalse(checks["bearing_map_contains_only_declared_image_identities"])
        self.assertFalse(checks["prior_probe_goals_match_public_directions"])
        self.assertFalse(checks["no_extra_full_rotation_per_selected_target"])

    def test_probe_must_stop_and_bidirectional_requirement_is_observed(self):
        summary, events, current = self.evidence()
        events[0]["state"] = "PROBING"
        checks, _ = direction_route_audit(summary, events, current, {"require_bidirectional_search": True})
        self.assertFalse(checks["fixed_probe_is_stationary"])
        self.assertFalse(checks["both_turn_directions_actually_used"])


if __name__ == "__main__":
    unittest.main()
