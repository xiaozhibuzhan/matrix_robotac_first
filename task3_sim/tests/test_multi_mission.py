"""Identity, fresh-observation authority and bounded fixed-target sequencing."""

from dataclasses import replace
from itertools import permutations
import json
import unittest

import numpy as np

from task3_sim.interfaces import Frame, RobotFeedback, RobotMotionCommand, ShotResult
from task3_sim.mission import MultiTargetMission
from task3_sim.perception import TagDetection


def config(ids=(2, 5), **mission):
    return {
        "camera": {"rotation_base_from_camera": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                   "position_base_m": [0, 0, 0]},
        "laser": {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]},
        "control": {"stable_duration_s": 0.3, "max_frame_age_s": 0.15, "max_feedback_age_s": 0.15},
        "targets": [{"id": target_id, "tag": {"id": target_id, "size_m": 0.16,
                     "bullseye_offset_tag_m": [0, 0.2 + index * 0.2, 0]}}
                    for index, target_id in enumerate(ids)],
        "mission": {"target_timeout_s": 2.0, "lost_timeout_s": 0.2,
                    "switch_pause_s": 0.15, **mission},
    }


def detection(target_id, now, offset=0.2, **changes):
    result = TagDetection(target_id, np.eye(3), np.array([0, -offset, 3.0]),
                          100.0, 0, np.zeros((4, 2)), 0.1, now)
    return replace(result, **changes)


def frame(now):
    return Frame(np.zeros((8, 8, 3), dtype=np.uint8), now)


def feedback(now, **changes):
    return replace(RobotFeedback(True, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), now), **changes)


class MultiMissionTests(unittest.TestCase):
    def update(self, mission, now, *, dets=None, image=True, fb=None, ammo=5):
        if dets is None:
            dets = [detection(2, now), detection(5, now, offset=0.4)]
        image = frame(now) if image is True else None if image is False else image
        return mission.update(image, dets, feedback(now) if fb is None else fb, now, ammo)

    def arm(self, mission, start=0.0, ammo=5, target_id=2, offset=0.2):
        for elapsed in (0, 0.1, 0.2, 0.31):
            result = self.update(mission, start + elapsed,
                                 dets=[detection(target_id, start + elapsed, offset)], ammo=ammo)
        self.assertTrue(result.should_fire, result.reason)
        return result

    def assert_stopped(self, decision):
        self.assertEqual(decision.command, RobotMotionCommand())
        self.assertFalse(decision.should_fire)

    def test_sequential_ids_use_own_bullseye_offset_and_new_dwell(self):
        mission = MultiTargetMission(config())
        first = self.arm(mission)
        self.assertEqual(first.target_id, 2)
        self.assertTrue(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4))
        self.assertEqual(mission.target_states, {2: "COMPLETED", 5: "PENDING"})
        self.assert_stopped(self.update(mission, 0.4, ammo=4))
        selected = self.update(mission, 0.47, ammo=4)
        self.assertEqual(selected.target_id, 5)
        self.assertEqual(selected.state, "SETTLING")
        self.assertFalse(selected.should_fire)
        for now in (0.57, 0.67, 0.78):
            ready = self.update(mission, now, ammo=4)
        self.assertTrue(ready.should_fire)
        self.assertAlmostEqual(ready.geometry.error_norm_m, 0.0)
        self.assertTrue(mission.record_shot(5, ShotResult(5, 9, True), 0.78, 3))
        self.assertTrue(mission.done)
        self.assertEqual(mission.status, "SUCCESS")
        result = mission.summary()
        self.assertEqual(result["total_score"], 19)
        self.assertEqual(result["shot_score_sum"], 19)
        self.assertEqual(result["latest_target_score_total"], 19)
        self.assertEqual({row["id"]: row["score_sum"] for row in result["targets"]}, {2: 10, 5: 9})
        self.assertEqual([shot["intended_target_id"] for shot in result["shots"]], [2, 5])
        self.assertEqual([shot["ammo_remaining"] for shot in result["shots"]], [4, 3])
        self.assert_stopped(self.update(mission, 1.0, ammo=3))
        json.dumps(result, allow_nan=False)
        json.dumps(ready.to_dict(), allow_nan=False)

    def test_wrong_offset_for_second_identity_cannot_reuse_first_aim(self):
        mission = MultiTargetMission(config())
        self.arm(mission)
        mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4)
        result = self.update(mission, 0.47, dets=[detection(5, 0.47, offset=0.2)], ammo=4)
        self.assertEqual(result.target_id, 5)
        self.assertEqual(result.state, "AIMING")
        self.assertAlmostEqual(result.geometry.error_norm_m, 0.2)

    def test_distractor_cannot_be_selected_or_create_a_fire_token(self):
        mission = MultiTargetMission(config())
        for now in (0.0, 0.1, 0.2, 0.31):
            result = self.update(mission, now, dets=[detection(42, now)])
            self.assertEqual(result.target_id, 2)
            self.assertFalse(result.should_fire)
        self.assertFalse(mission.record_shot(42, ShotResult(42, 10, True), 0.31, 4))
        self.assertEqual(mission.summary()["shots"], [])

    def test_duplicate_same_id_stops_and_restarts_dwell(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0)
        self.update(mission, 0.1)
        self.update(mission, 0.2)
        result = self.update(mission, 0.31, dets=[detection(2, 0.31), detection(2, 0.31, decision_margin=1)])
        self.assertEqual(result.state, "AMBIGUOUS")
        self.assert_stopped(result)
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4))
        for now in (0.41, 0.51, 0.62):
            self.assertFalse(self.update(mission, now).should_fire)
        self.assertTrue(self.update(mission, 0.72).should_fire)

    def test_stale_frames_replays_and_bad_feedback_revoke_ready_and_reset_dwell(self):
        cases = (
            {"image": frame(0.1), "dets": [detection(2, 0.1)]},
            {"image": frame(0.5), "dets": [detection(2, 0.5)]},
            {"fb": feedback(0.0)},
            {"fb": feedback(0.35, valid=False)},
            {"fb": feedback(0.35, rpy_rad=np.array([0.0, 0.0, np.nan]))},
        )
        for case in cases:
            with self.subTest(case=case):
                mission = MultiTargetMission(config())
                self.arm(mission)
                result = self.update(mission, 0.35, **case)
                self.assert_stopped(result)
                self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.35, 4))
                self.assertFalse(self.update(mission, 0.4).should_fire)

    def test_short_delivery_gaps_pause_dwell_without_earning_silent_time(self):
        source = config()
        source["control"]["stable_duration_s"] = 0.4
        mission = MultiTargetMission(source)
        self.update(mission, 0)
        # An image every 100 ms with an empty intervening 50 ms queue poll:
        # 0.8 seconds of wall time supplies only 0.4 seconds of valid-image
        # dwell. Every empty poll stops and cannot authorize a shot.
        for step in range(1, 16):
            now = round(step * 0.05, 8)
            delivered = step % 2 == 0
            result = self.update(mission, now, image=delivered)
            self.assertFalse(result.should_fire)
            if not delivered:
                self.assert_stopped(result)
                self.assertIn("paused", result.reason)
                self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), now, 4))
                self.assertIsNone(mission.summary()["targets"][0]["missing_since"])
            self.assertEqual(mission.target_states[2], "ACTIVE")
        ready = self.update(mission, 0.81)
        self.assertTrue(ready.should_fire)
        self.assertAlmostEqual(mission._controller.stable_since, 0.4)
        self.assertEqual([entry["to"] for entry in mission.summary()["targets"][0]["history"]], ["ACTIVE"])

    def test_short_empty_delivery_revokes_ready_but_requires_new_frame_to_rearm(self):
        mission = MultiTargetMission(config())
        self.arm(mission)
        waiting = self.update(mission, 0.35, image=False)
        self.assert_stopped(waiting)
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.35, 4))
        self.assertTrue(self.update(mission, 0.4).should_fire)

    def test_prolonged_nondelivery_resets_after_last_acquisition_becomes_stale(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0)
        self.update(mission, 0.1)
        self.assert_stopped(self.update(mission, 0.15, image=False))
        self.assertIsNotNone(mission._controller.stable_since)
        self.assert_stopped(self.update(mission, 0.3, image=False))
        self.assertIsNone(mission._controller.stable_since)
        for now in (0.35, 0.45, 0.55):
            self.assertFalse(self.update(mission, now).should_fire)
        self.assertTrue(self.update(mission, 0.66).should_fire)

    def test_nondelivery_with_motion_or_invalid_feedback_resets_stability(self):
        cases = (
            feedback(0.25, gyro_rad_s=np.array([0, 0, 0.04])),
            feedback(0.25, velocity_m_s=np.array([0.02, 0, 0])),
            feedback(0.25, valid=False),
            feedback(0.0),
        )
        for state in cases:
            with self.subTest(feedback=state):
                mission = MultiTargetMission(config())
                for now in (0, 0.1, 0.2):
                    self.update(mission, now)
                self.assert_stopped(self.update(mission, 0.25, image=False, fb=state))
                self.assertIsNone(mission._controller.stable_since)
                self.assertFalse(self.update(mission, 0.3).should_fire)

    def test_short_nondelivery_does_not_recover_previously_absent_target(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0.0)
        self.update(mission, 0.1, dets=[])
        result = self.update(mission, 0.15, image=False)
        self.assert_stopped(result)
        self.assertNotIn("paused", result.reason)
        self.assertEqual(mission.summary()["targets"][0]["missing_since"], 0.1)
        self.assertIsNone(mission._controller.stable_since)

    def test_replayed_frame_cannot_extend_dwell_or_rearm_across_targets(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0.0)
        result = self.update(mission, 0.05, image=frame(0), dets=[detection(2, 0)])
        self.assert_stopped(result)
        self.assertIn("repeated", result.reason)
        self.assertFalse(self.update(mission, 0.1).should_fire)
        self.assertFalse(self.update(mission, 0.2).should_fire)
        self.assertFalse(self.update(mission, 0.31).should_fire)
        self.assertTrue(self.update(mission, 0.41).should_fire)

    def test_silent_long_capture_gap_does_not_count_as_stable_observation(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0)
        self.assertFalse(self.update(mission, 0.4).should_fire)
        self.assertFalse(self.update(mission, 0.5).should_fire)
        self.assertFalse(self.update(mission, 0.6).should_fire)
        self.assertTrue(self.update(mission, 0.71).should_fire)

    def test_loss_state_reacquisition_starts_new_stable_window(self):
        mission = MultiTargetMission(config())
        self.update(mission, 0)
        self.update(mission, 0.1, dets=[])
        self.update(mission, 0.31, dets=[])
        self.assertEqual(mission.target_states[2], "LOST")
        for now in (0.4, 0.5, 0.6):
            self.assertFalse(self.update(mission, now).should_fire)
        self.assertEqual(mission.target_states[2], "ACTIVE")
        self.assertTrue(self.update(mission, 0.71).should_fire)
        states = [entry["to"] for entry in mission.summary()["targets"][0]["history"]]
        self.assertEqual(states, ["ACTIVE", "LOST", "ACTIVE"])

    def test_missing_target_deadline_continues_to_next_without_spending_ammo(self):
        mission = MultiTargetMission(config(target_timeout_s=0.5))
        self.update(mission, 0.0, dets=[])
        self.update(mission, 0.2, dets=[])
        result = self.update(mission, 0.51, dets=[])
        self.assertEqual(result.state, "SWITCHING")
        self.assertEqual(mission.target_states[2], "FAILED")
        self.arm(mission, start=0.7, target_id=5, offset=0.4)
        self.assertTrue(mission.record_shot(5, ShotResult(5, 10, True), 1.01, 4))
        self.assertEqual(mission.status, "PARTIAL")
        first, second = mission.summary()["targets"]
        self.assertEqual(first["attempts"], 0)
        self.assertEqual(first["reason"], "target_timeout")
        self.assertEqual(second["state"], "COMPLETED")

    def test_visual_search_is_bounded_by_feedback_yaw_and_never_pitches(self):
        mission = MultiTargetMission(config(search_span_rad=0.5))
        first = self.update(mission, 0, dets=[])
        self.assertGreater(first.command.yaw_rate_rad_s, 0)
        high = self.update(mission, 0.1, dets=[], fb=feedback(0.1, rpy_rad=np.array([0, 0, 0.6])))
        self.assertLess(high.command.yaw_rate_rad_s, 0)
        low = self.update(mission, 0.2, dets=[], fb=feedback(0.2, rpy_rad=np.array([0, 0, -0.6])))
        self.assertGreater(low.command.yaw_rate_rad_s, 0)
        self.assertEqual(low.command.pitch_rate_rad_s, 0)
        self.assert_stopped(self.update(mission, 0.3, dets=[], image=False))

    def test_full_rotation_search_unwraps_pi_boundary_and_stops_at_its_budget(self):
        mission = MultiTargetMission(config(search_mode="full_rotation", target_timeout_s=45,
                                            search_yaw_rate_rad_s=0.35))
        start_yaw = 3.1
        initial = self.update(mission, 0, dets=[],
                              fb=feedback(0, rpy_rad=np.array([0, 0, start_yaw])))
        self.assertGreater(initial.command.yaw_rate_rad_s, 0)
        # Public feedback wraps at pi, while the actual turn is positive.
        for step in range(1, 14):
            travelled = step * 0.5
            now = travelled / 0.35
            wrapped = np.arctan2(np.sin(start_yaw + travelled), np.cos(start_yaw + travelled))
            result = self.update(mission, now, dets=[],
                                  fb=feedback(now, rpy_rad=np.array([0, 0, wrapped])))
            self.assertFalse(result.should_fire)
            self.assertEqual(result.command.pitch_rate_rad_s, 0)
            if travelled < 2 * np.pi:
                self.assertGreater(result.command.yaw_rate_rad_s, 0)
        self.assertEqual(result.state, "SEARCH_EXHAUSTED")
        self.assert_stopped(result)
        record = mission.summary()["targets"][0]
        self.assertAlmostEqual(record["search_origin_yaw_rad"], start_yaw)
        self.assertAlmostEqual(record["search_travel_rad"], 6.5)
        self.assertTrue(record["search_exhausted"])
        # A completed visual sweep stops rather than starting endless cycles;
        # the existing target deadline still determines the failure reason.
        self.assertEqual(mission.target_states[2], "LOST")
        timeout = self.update(mission, 45.01, dets=[])
        self.assertEqual(timeout.state, "SWITCHING")
        self.assertEqual(mission.summary()["targets"][0]["reason"], "target_timeout")

    def test_full_rotation_is_also_valid_with_unbounded_backend_yaw(self):
        mission = MultiTargetMission(config(search_mode="full_rotation", target_timeout_s=45))
        for index, yaw in enumerate((6.0, 6.2, 6.4, 6.6)):
            now = index * 0.7
            result = self.update(mission, now, dets=[],
                                  fb=feedback(now, rpy_rad=np.array([0, 0, yaw])))
            self.assertGreater(result.command.yaw_rate_rad_s, 0)
        self.assertAlmostEqual(mission.summary()["targets"][0]["search_travel_rad"], 0.6)

    def test_full_rotation_stops_without_current_unique_image_or_feedback(self):
        mission = MultiTargetMission(config(search_mode="full_rotation"))
        self.update(mission, 0.0, dets=[])
        self.assert_stopped(self.update(mission, 0.05, dets=[], image=False))
        self.assert_stopped(self.update(mission, 0.1, dets=[], image=frame(0.0)))
        self.assert_stopped(self.update(mission, 0.2, dets=[], image=frame(0.01)))
        self.assert_stopped(self.update(mission, 0.25, dets=[], fb=feedback(0.25, valid=False)))
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.25, 4))

    def test_full_rotation_resets_sweep_only_on_target_switch_not_reacquisition(self):
        mission = MultiTargetMission(config(search_mode="full_rotation", search_max_rotation_rad=0.5))
        self.arm(mission)
        mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4)
        self.update(mission, 0.47, dets=[], ammo=4,
                    fb=feedback(0.47, rpy_rad=np.array([0, 0, 1.4])))
        second = mission.summary()["targets"][1]
        self.assertAlmostEqual(second["search_origin_yaw_rad"], 1.4)
        self.assertEqual(second["search_travel_rad"], 0)
        self.update(mission, 0.57, dets=[], ammo=4,
                    fb=feedback(0.57, rpy_rad=np.array([0, 0, 1.7])))
        exhausted = self.update(mission, 0.67, dets=[], ammo=4,
                                fb=feedback(0.67, rpy_rad=np.array([0, 0, 2.0])))
        self.assert_stopped(exhausted)
        self.assertEqual(exhausted.state, "SEARCH_EXHAUSTED")
        reacquired = self.update(mission, 0.77, ammo=4,
                                 fb=feedback(0.77, rpy_rad=np.array([0, 0, 2.0])))
        self.assertEqual(reacquired.state, "SETTLING")
        self.assertFalse(reacquired.should_fire)
        self.assertTrue(mission.summary()["targets"][1]["search_exhausted"])
        self.assertAlmostEqual(mission.summary()["targets"][1]["search_travel_rad"], 0.6)
        for now in (0.87, 0.97, 1.08):
            ready = self.update(mission, now, ammo=4,
                                fb=feedback(now, rpy_rad=np.array([0, 0, 2.0])))
        self.assertTrue(ready.should_fire)
        self.assertTrue(mission.record_shot(5, ShotResult(5, 10, True), 1.08, 3))
        self.assertEqual(mission.status, "SUCCESS")

    def test_full_rotation_missing_target_deadline_advances_without_shooting(self):
        mission = MultiTargetMission(config(search_mode="full_rotation", target_timeout_s=0.5))
        self.update(mission, 0, dets=[])
        self.update(mission, 0.2, dets=[])
        self.assertEqual(self.update(mission, 0.51, dets=[]).state, "SWITCHING")
        self.arm(mission, start=0.7, target_id=5, offset=0.4)
        mission.record_shot(5, ShotResult(5, 10, True), 1.01, 4)
        self.assertEqual(mission.status, "PARTIAL")
        self.assertEqual(mission.summary()["targets"][0]["attempts"], 0)
        self.assertEqual(mission.summary()["targets"][0]["reason"], "target_timeout")
        self.assertEqual(mission.summary()["targets"][1]["attempts"], 1)

    def test_search_mode_and_one_cycle_limit_are_validated(self):
        with self.assertRaises(ValueError):
            MultiTargetMission(config(search_mode="scene_heading_shortcut"))
        with self.assertRaises(ValueError):
            MultiTargetMission(config(search_mode="full_rotation", search_max_rotation_rad=4 * np.pi))
        self.assertEqual(MultiTargetMission(config()).search_mode, "bounded")

    def test_fixed_directions_all_id_positions_and_shooting_orders_are_visual(self):
        ids = (2, 5, 9)
        directions = (0.0, np.pi / 2, -np.pi / 2)
        for placement in permutations(ids):
            locations = {identity: angle for identity, angle in zip(placement, directions)}
            for order in permutations(ids):
                with self.subTest(placement=placement, order=order):
                    source = config(ids=order, search_mode="fixed_directions", target_timeout_s=35,
                                    search_yaw_rate_rad_s=0.35, direction_probe_s=0.2)
                    mission = MultiTargetMission(source)
                    yaw, rate, ammo = 0.0, 0.0, 5
                    for step in range(1600):
                        now = step * 0.05
                        yaw += rate * 0.05
                        dets = []
                        for index, identity in enumerate(order):
                            angle = locations[identity] - yaw
                            if abs(np.arctan2(np.sin(angle), np.cos(angle))) < 0.42:
                                # Ideal image-derived pose: tags are only
                                # delivered inside the physical camera FOV.
                                dets.append(detection(identity, now, offset=0.2 + index * 0.2,
                                                      t_camera_from_tag=np.array([
                                                          -3 * np.sin(angle),
                                                          -0.2 - index * 0.2,
                                                          3 * np.cos(angle)])))
                        decision = mission.update(frame(now), dets, feedback(now,
                            rpy_rad=np.array([0, 0, yaw]), gyro_rad_s=np.array([0, 0, rate])), now, ammo)
                        if decision.should_fire:
                            ammo -= 1
                            self.assertTrue(mission.record_shot(decision.target_id,
                                ShotResult(decision.target_id, 10, True), now, ammo))
                        rate = decision.command.yaw_rate_rad_s
                        if mission.done:
                            break
                    self.assertEqual(mission.status, "SUCCESS", mission.summary())
                    report = mission.summary()
                    self.assertEqual([shot["intended_target_id"] for shot in report["shots"]], list(order))
                    self.assertEqual(report["ammo_remaining"], 2)
                    self.assertTrue(all(record["attempts"] == 1 for record in report["targets"]))
                    # An initially unknown identity can require front->left
                    # ->right, but never an extra full turn per selection.
                    self.assertLess(max(record["search_travel_rad"] for record in report["targets"]), 5.0)
                    self.assertTrue(all(entry["source"] == "image_pose_with_timestamped_robot_feedback"
                                        for entry in report["search"]["observed_bearings"].values()))

    def test_fixed_direction_switch_uses_cached_identity_shortest_clockwise_turn(self):
        mission = MultiTargetMission(config(search_mode="fixed_directions", target_timeout_s=20))
        # ID5 is first observed in front while ID2 is searched for to the left.
        self.update(mission, 0, dets=[detection(5, 0, offset=0.4)])
        for now in (0.1, 0.2, 0.3, 0.4, 0.51):
            selected = self.update(mission, now, dets=[detection(2, now)],
                                   fb=feedback(now, rpy_rad=np.array([0, 0, np.pi / 2])))
        self.assertTrue(selected.should_fire)
        self.assertTrue(mission.record_shot(2, ShotResult(2, 10, True), 0.51, 4))
        next_target = self.update(mission, 0.7, dets=[], ammo=4,
                                 fb=feedback(0.7, rpy_rad=np.array([0, 0, np.pi / 2])))
        self.assertEqual(next_target.target_id, 5)
        self.assertLess(next_target.command.yaw_rate_rad_s, 0)
        self.assertIn("image_derived_bearing", next_target.reason)
        self.assertFalse(next_target.should_fire)

    def test_fixed_bearing_uses_acquisition_feedback_interpolated_across_pi(self):
        mission = MultiTargetMission(config(search_mode="fixed_directions"))
        self.update(mission, 0, dets=[], fb=feedback(0, rpy_rad=np.array([0, 0, 3.1])))
        capture = frame(0.05)
        result = self.update(mission, 0.1, dets=[detection(5, 0.05, offset=0.4)], image=capture,
                             fb=feedback(0.1, rpy_rad=np.array([0, 0, -3.1])))
        self.assertFalse(result.should_fire)
        cached = mission.summary()["search"]["observed_bearings"][5]
        self.assertAlmostEqual(abs(cached["yaw_rad"]), np.pi)
        self.assertEqual(cached["timestamp"], 0.05)

    def test_fixed_bearing_does_not_interpolate_across_an_unobserved_feedback_gap(self):
        mission = MultiTargetMission(config(search_mode="fixed_directions"))
        self.update(mission, 0, dets=[])
        self.update(mission, 0.5, image=frame(0.45), dets=[detection(5, 0.45, offset=0.4)],
                    fb=feedback(0.5, rpy_rad=np.array([0, 0, 1.0])))
        self.assertEqual(mission.summary()["search"]["observed_bearings"], {})

    def test_fixed_cache_rejects_uncovered_timestamps_duplicates_quality_and_unknown_ids(self):
        mission = MultiTargetMission(config(search_mode="fixed_directions"))
        # A capture before the first feedback cannot be assigned a bearing.
        self.update(mission, 0.1, image=frame(0.05), dets=[detection(5, 0.05, offset=0.4)])
        self.assertEqual(mission.summary()["search"]["observed_bearings"], {})
        self.update(mission, 0.2, dets=[detection(5, 0.2), detection(5, 0.2), detection(42, 0.2)])
        self.assertEqual(mission.summary()["search"]["observed_bearings"], {})
        self.update(mission, 0.3, dets=[detection(5, 0.3, decision_margin=1)])
        self.assertEqual(mission.summary()["search"]["observed_bearings"], {})
        self.update(mission, 0.4, dets=[detection(5, 0.4, offset=0.4)])
        self.assertEqual(set(mission.summary()["search"]["observed_bearings"]), {5})
        # Replayed frames and invalid feedback cannot replace that identity.
        self.update(mission, 0.45, image=frame(0.4), dets=[detection(5, 0.4)])
        self.update(mission, 0.5, dets=[detection(5, 0.5)], fb=feedback(0.5, valid=False))
        self.assertEqual(mission.summary()["search"]["observed_bearings"][5]["timestamp"], 0.4)

    def test_fixed_probe_empty_or_stale_images_and_motion_do_not_earn_probe_time(self):
        mission = MultiTargetMission(config(ids=(2,), search_mode="fixed_directions",
                                           search_directions_rad=[0], direction_probe_s=0.2))
        self.update(mission, 0, dets=[])
        self.update(mission, 0.1, dets=[])
        self.assertAlmostEqual(mission._probe_elapsed, 0.1)
        self.assert_stopped(self.update(mission, 0.15, dets=[], image=False))
        self.update(mission, 0.2, dets=[])
        self.assertEqual(mission._probe_elapsed, 0)
        self.assert_stopped(self.update(mission, 0.25, dets=[], image=frame(0.2)))
        self.assertEqual(mission._probe_elapsed, 0)
        self.update(mission, 0.3, dets=[], fb=feedback(0.3, gyro_rad_s=np.array([0, 0, 0.1])))
        self.assertEqual(mission._probe_elapsed, 0)
        for now in (0.4, 0.5, 0.61):
            self.update(mission, now, dets=[])
        stopped = self.update(mission, 0.71, dets=[])
        self.assertEqual(stopped.state, "SEARCH_EXHAUSTED")
        self.assert_stopped(stopped)
        self.assertEqual(mission.summary()["targets"][0]["attempts"], 0)

    def test_fixed_bearing_cache_expires_after_age_or_measured_translation(self):
        for overrides, now, position in (({"bearing_max_age_s": 0.2}, 0.3, np.zeros(3)),
                                         ({}, 0.1, np.array([0.3, 0, 0]))):
            with self.subTest(overrides=overrides):
                mission = MultiTargetMission(config(search_mode="fixed_directions", **overrides))
                self.update(mission, 0, dets=[detection(5, 0, offset=0.4)])
                self.assertIsNotNone(mission._usable_bearing(5, feedback(0), 0))
                self.assertIsNone(mission._usable_bearing(5, feedback(now, position_m=position), now))

    def test_fixed_direction_prior_validation_and_no_id_direction_assumption(self):
        for bad in ([], [0, 2 * np.pi], [0, np.nan], [True, 0], "left/front/right"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                MultiTargetMission(config(search_mode="fixed_directions", search_directions_rad=bad))
        for overrides in ({"direction_tolerance_rad": np.pi / 4},
                          {"local_search_span_rad": np.pi / 2}, {"direction_probe_s": 0}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                MultiTargetMission(config(search_mode="fixed_directions", **overrides))
        mission = MultiTargetMission(config(ids=(97, 56, 11), search_mode="fixed_directions"))
        decision = self.update(mission, 0, dets=[detection(42, 0)])
        self.assertEqual(decision.target_id, 97)
        self.assertEqual(decision.state, "PROBING")
        self.assertEqual(mission.summary()["search"]["observed_bearings"], {})

    def test_external_control_transition_revokes_ready_preserves_ledger_and_requires_new_capture(self):
        mission = MultiTargetMission(config(search_mode="fixed_directions"))
        self.arm(mission)
        before = mission.summary()
        mission.invalidate_control_observation(0.35)
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.35, 4))
        after = mission.summary()
        for field in ("active_target_id", "ammo_remaining", "targets", "shots", "total_score", "search"):
            self.assertEqual(before[field], after[field])
        # This capture is newer than the last consumed frame and only 30 ms
        # old, but predates completion of the external control transition.
        waiting = self.update(mission, 0.36, image=frame(0.33), dets=[detection(2, 0.33)])
        self.assert_stopped(waiting)
        self.assertIn("control transition", waiting.reason)
        for now in (0.4, 0.5, 0.6):
            self.assertFalse(self.update(mission, now).should_fire)
        self.assertTrue(self.update(mission, 0.71).should_fire)
        self.assertTrue(mission.record_shot(2, ShotResult(2, 10, True), 0.71, 4))
        self.assertEqual(mission.summary()["shots"][0]["shot_index"], 1)

    def test_control_observation_invalidation_resets_search_probe_and_rejects_bad_clock(self):
        mission = MultiTargetMission(config(ids=(2,), search_mode="fixed_directions", search_directions_rad=[0]))
        self.update(mission, 0, dets=[])
        self.update(mission, 0.1, dets=[])
        self.assertGreater(mission._probe_elapsed, 0)
        mission.invalidate_control_observation(0.15, "actuator mode changed")
        self.assertEqual(mission._probe_elapsed, 0)
        for bad in (float("nan"), 0.0):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                mission.invalidate_control_observation(bad)

    def test_wrong_target_keeps_actual_score_but_credits_neither_target(self):
        mission = MultiTargetMission(config())
        self.arm(mission)
        self.assertTrue(mission.record_shot(2, ShotResult(5, 10, True, reason="wrong_target"), 0.31, 4))
        self.assertEqual(mission.target_states, {2: "FAILED", 5: "PENDING"})
        report = mission.summary()
        self.assertEqual(report["total_score"], 0)
        self.assertEqual(report["shot_score_sum"], 0)
        self.assertEqual(report["latest_target_score_total"], 0)
        self.assertTrue(all(row["score_sum"] == 0 for row in report["targets"]))
        self.assertEqual(report["shots"][0]["actual_target_id"], 5)
        self.assertEqual(report["shots"][0]["ring"], 10)
        self.assertEqual(report["shots"][0]["credited_ring"], 0)
        self.assertEqual(report["shots"][0]["reason"], "wrong_target_hit")

    def test_missed_shot_is_terminal_with_exactly_one_attempt(self):
        for reason, actual_id in (("occluded", None), ("no_surface", None), ("outside_rings", 2)):
            with self.subTest(reason=reason):
                mission = MultiTargetMission(config(ids=(2,)))
                self.arm(mission)
                self.assertTrue(mission.record_shot(2, ShotResult(actual_id, 3, False, reason=reason), 0.31, 4))
                self.assertEqual(mission.status, "FAILED")
                self.assertEqual(mission.summary()["targets"][0]["attempts"], 1)
                self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 3))

    def test_fail_active_revokes_ready_without_spending_ammo_and_continues_next_target(self):
        mission = MultiTargetMission(config())
        self.arm(mission)
        failed = mission.fail_active(.35, "aim_response_stalled")
        self.assert_stopped(failed)
        self.assertEqual(failed.state, "SWITCHING")
        self.assertEqual(failed.target_id, 2)
        self.assertEqual(mission.target_states, {2: "FAILED", 5: "PENDING"})
        report = mission.summary()
        self.assertEqual(report["ammo_remaining"], 5)
        self.assertEqual(report["shots"], [])
        self.assertEqual(report["total_score"], 0)
        self.assertEqual(report["targets"][0]["reason"], "aim_response_stalled")
        self.assertEqual(report["targets"][0]["attempts"], 0)
        self.assertEqual(report["targets"][0]["finished_at"], .35)
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), .35, 4))
        reversed_time = self.update(mission, .34)
        self.assert_stopped(reversed_time)
        self.assertEqual(reversed_time.state, "WAIT")
        self.assertIn("non-monotonic", reversed_time.reason)
        self.assert_stopped(self.update(mission, .4))
        next_target = self.update(mission, .51)
        self.assertEqual(next_target.target_id, 5)
        self.assertFalse(next_target.should_fire)
        self.arm(mission, start=.61, target_id=5, offset=.4)
        self.assertTrue(mission.record_shot(5, ShotResult(5, 10, True), .92, 4))
        self.assertEqual(mission.status, "PARTIAL")
        self.assertEqual(mission.summary()["ammo_remaining"], 4)
        self.assertEqual(mission.summary()["total_score"], 10)
        self.assertEqual(len(mission.summary()["shots"]), 1)

    def test_fail_active_validates_time_and_active_target_without_mutating_on_error(self):
        mission = MultiTargetMission(config(ids=(2,)))
        with self.assertRaisesRegex(ValueError, "no active"):
            mission.fail_active(0., "aim_response_stalled")
        self.arm(mission)
        before = mission.summary()
        for bad in (float("nan"), float("inf"), .3):
            with self.subTest(time=bad), self.assertRaises(ValueError):
                mission.fail_active(bad, "aim_response_stalled")
            self.assertEqual(mission.summary(), before)
        result = mission.fail_active(.35, "aim_response_stalled")
        self.assert_stopped(result)
        self.assertEqual(result.state, "FAILED")
        self.assertTrue(mission.done)
        self.assertEqual(mission.status, "FAILED")
        with self.assertRaisesRegex(ValueError, "no active"):
            mission.fail_active(.4, "again")
        self.assertEqual(mission.summary()["shots"], [])

    def test_shot_callback_requires_live_intended_token_and_ammo_decrement(self):
        mission = MultiTargetMission(config())
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0, 4))
        self.arm(mission)
        self.assertFalse(mission.record_shot(5, ShotResult(5, 10, True), 0.31, 4))
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 5))
        self.assertTrue(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4))
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 3))
        self.assertEqual(len(mission.summary()["shots"]), 1)

    def test_expired_callback_and_backwards_time_never_fire(self):
        for callback_time in (0.30, 0.6):
            with self.subTest(callback_time=callback_time):
                mission = MultiTargetMission(config())
                self.arm(mission)
                self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), callback_time, 4))
        mission = MultiTargetMission(config())
        self.arm(mission)
        result = self.update(mission, 0.30)
        self.assert_stopped(result)
        self.assertFalse(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4))

    def test_no_ammo_preserves_completed_and_marks_remaining_failed(self):
        mission = MultiTargetMission(config())
        self.arm(mission, ammo=1)
        self.assertTrue(mission.record_shot(2, ShotResult(2, 10, True), 0.31, 0))
        self.assertEqual(mission.status, "NO_AMMO")
        self.assertEqual(mission.target_states, {2: "COMPLETED", 5: "FAILED"})
        self.assertEqual(mission.summary()["targets"][1]["reason"], "no_ammo")
        self.assertEqual(mission.summary()["total_score"], 10)
        empty = MultiTargetMission(config())
        self.assert_stopped(self.update(empty, 0, ammo=0))
        self.assertEqual(empty.status, "NO_AMMO")
        self.assertEqual(empty.summary()["shots"], [])
        # Last round completes every target: SUCCESS takes precedence.
        single = MultiTargetMission(config(ids=(2,)))
        self.arm(single, ammo=1)
        single.record_shot(2, ShotResult(2, 10, True), 0.31, 0)
        self.assertEqual(single.status, "SUCCESS")

    def test_timeout_and_cancel_preserve_completed_history(self):
        for method, status, reason in (("finish_timeout", "PARTIAL", "global_timeout"),
                                       ("cancel", "CANCELLED", "cancelled by operator")):
            with self.subTest(method=method):
                mission = MultiTargetMission(config())
                self.arm(mission)
                mission.record_shot(2, ShotResult(2, 10, True), 0.31, 4)
                getattr(mission, method)(0.4)
                self.assertEqual(mission.status, status)
                self.assertEqual(mission.target_states, {2: "COMPLETED", 5: "FAILED"})
                self.assertEqual(mission.summary()["targets"][1]["reason"], reason)

    def test_duplicate_or_inconsistent_declared_id_fails_early(self):
        bad = config(ids=(2, 2))
        with self.assertRaises(ValueError):
            MultiTargetMission(bad)
        bad = config()
        bad["targets"][1]["tag"]["id"] = 2
        with self.assertRaises(ValueError):
            MultiTargetMission(bad)

    def test_manager_does_not_retain_scene_truth_or_mutable_input(self):
        source = config()
        source["simulation"] = {"target_center_world_m": object()}
        source["targets"][0]["position_world_m"] = object()
        mission = MultiTargetMission(source)
        source["targets"][0]["tag"]["bullseye_offset_tag_m"] = [100, 100, 100]
        self.arm(mission)
        self.assertFalse(hasattr(mission, "simulation"))
        self.assertNotIn("simulation", mission._base)
        self.assertNotIn("position_world_m", mission._tag_configs[2])


if __name__ == "__main__":
    unittest.main()
