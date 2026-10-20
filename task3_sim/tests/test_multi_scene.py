"""Independent multi-board scene geometry, textures, and truth boundary."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import mujoco
import numpy as np

from task3_sim.appearance import asset_path, read_image
from task3_sim.backend import MujocoRobot
from task3_sim.multi_config import expand_target, public_mission_config, validate_multi_config
from task3_sim.multi_scene import build_multi_scene, _target_texture
from task3_sim.perception import AprilTagPerception


PACKAGE = Path(__file__).resolve().parents[1]


def config():
    return json.loads((PACKAGE/"config/multi_fixed.json").read_text(encoding="utf-8"))


class MultiConfigTests(unittest.TestCase):
    def test_valid_default_and_no_truth_in_public_contract(self):
        c = config()
        validate_multi_config(c)
        public = public_mission_config(c)
        self.assertEqual([s["id"] for s in public["targets"]], [2, 5, 9])
        serialized = json.dumps(public)
        for forbidden in ("simulation", "target_center_world_m", "target_yaw_rad", "distractors", "ring_ellipses_m"):
            self.assertNotIn(forbidden, serialized)
        public["targets"][0]["tag"]["size_m"] = .3
        self.assertEqual(c["tag"]["size_m"], .16)

    def test_default_clock_directions_and_surround_layout_are_explicit(self):
        c = config()
        self.assertEqual(c["simulation"]["initial_rpy_rad"][2], 0)
        expected = {2: ([0, 2.7, .5], np.pi/2), 5: ([2.7, 0, .5], 0), 9: ([0, -2.7, .5], -np.pi/2)}
        for target in c["targets"]:
            position, yaw = expected[target["id"]]
            np.testing.assert_allclose(target["simulation"]["target_center_world_m"], position)
            self.assertAlmostEqual(target["simulation"]["target_yaw_rad"], yaw)
        self.assertEqual(c["appearance"]["background"]["layout"], "surround")
        c["appearance"]["background"]["layout"] = "invalid"
        with self.assertRaisesRegex(ValueError, "background.layout"):
            validate_multi_config(c)

    def test_ids_unique_across_mission_and_distractors(self):
        c = config()
        c["distractors"] = [{"id": 5}]
        with self.assertRaises(ValueError):
            validate_multi_config(c)

    def test_tag_only_is_boolean_and_reserved_for_non_scoring_distractors(self):
        c = config()
        c["targets"][0]["tag_only"] = True
        with self.assertRaisesRegex(ValueError, "non-scoring distractors"):
            validate_multi_config(c)
        c = config()
        c["distractors"] = [{"id": 42, "tag_only": "true"}]
        with self.assertRaisesRegex(ValueError, "tag_only must be a boolean"):
            validate_multi_config(c)
        c = config()
        c["targets"][0]["id"] = 2.5
        with self.assertRaises(ValueError):
            validate_multi_config(c)

    def test_each_board_validates_its_own_finite_geometry(self):
        for change in ({"target_yaw_rad": float("nan")}, {"board_height_m": float("inf")}, {"target_center_world_m": [2, float("nan"), .5]}):
            c = config()
            c["targets"][1]["simulation"].update(change)
            with self.assertRaises(ValueError):
                validate_multi_config(c)

    def test_nonfinite_calibration_and_repeat_attempts_rejected(self):
        c = config()
        c["camera"]["fx"] = c["camera"]["fy"] = float("inf")
        with self.assertRaises(ValueError):
            validate_multi_config(c)
        c = config()
        c["mission"]["max_target_attempts"] = 2
        with self.assertRaises(ValueError):
            validate_multi_config(c)

    def test_malformed_mapping_sections_have_clear_validation_errors(self):
        with self.assertRaisesRegex(ValueError, "config must be an object"):
            validate_multi_config([])
        for section in ("camera", "laser", "tag", "control", "simulation", "appearance"):
            c = config()
            c[section] = []
            with self.assertRaisesRegex(ValueError, f"config.{section} must be an object"):
                validate_multi_config(c)
        for section in ("tag", "simulation", "appearance"):
            c = config()
            c["targets"][1][section] = None
            with self.assertRaisesRegex(ValueError, f"board.{section} must be an object"):
                validate_multi_config(c)

    def test_photo_metric_dimensions_cannot_silently_diverge_from_calibration(self):
        for overrides in ({"board_width_m": .645*1.1, "board_height_m": .6815*1.1},
                          {"board_center_down_m": -.03}):
            c = config()
            c["targets"][0]["simulation"].update(overrides)
            with self.assertRaisesRegex(ValueError, "differs from its calibration"):
                validate_multi_config(c)
    def test_synthetic_override_discards_photo_conics_and_uses_own_offset(self):
        c = config()
        spec = c["targets"][1]
        spec["appearance"] = {"target": {"kind": "synthetic"}}
        spec["tag"] = {"bullseye_offset_tag_m": [.025, .2, 0]}
        expanded = expand_target(c, spec)
        self.assertNotIn("ring_ellipses_m", expanded["simulation"])
        self.assertNotIn("scoring_polygon_m", expanded["simulation"])
        self.assertEqual(expanded["tag"]["bullseye_offset_tag_m"], [.025, .2, 0])
        self.assertIn("ring_ellipses_m", c["simulation"])
        validate_multi_config(c)

    def test_generated_markers_decode_actual_ids_without_replacing_original_id2(self):
        c = config()
        source = asset_path(c["appearance"]["target"]["asset"])
        original_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        decoder = cv2.aruco.ArucoDetector(dictionary)
        with tempfile.TemporaryDirectory() as directory:
            for spec in c["targets"] + c["distractors"]:
                path = Path(directory)/f'{spec["id"]}.png'
                report = _target_texture(expand_target(c, spec), path)
                _, ids, _ = decoder.detectMarkers(read_image(path))
                self.assertEqual(ids.reshape(-1).tolist(), [spec["id"]])
                self.assertEqual(report["marker_replaced"], spec["id"] != 2)
                if spec["id"] == 2:
                    self.assertEqual(path.read_bytes(), source.read_bytes())
                else:
                    # No edit outside the marker replacement rectangle.
                    before, after = read_image(source), read_image(path)
                    np.testing.assert_array_equal(before[550:], after[550:])
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), original_hash)

    def test_shifted_resized_markers_keep_continuous_white_quiet_zone(self):
        c = config()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        decoder = cv2.aruco.ArucoDetector(dictionary)
        overrides = {5: {"size_m": .14, "bullseye_offset_tag_m": [-.06, .19, 0]},
                     9: {"size_m": .18, "bullseye_offset_tag_m": [.05, .215, 0]}}
        with tempfile.TemporaryDirectory() as directory:
            for spec in c["targets"][1:]:
                spec["tag"] = overrides[spec["id"]]
                path = Path(directory)/f'{spec["id"]}.png'
                report = _target_texture(expand_target(c, spec), path)
                image = read_image(path)
                _, ids, _ = decoder.detectMarkers(image)
                self.assertEqual(ids.reshape(-1).tolist(), [spec["id"]])
                (x0, y0), (x1, y1) = report["quiet_zone_patch_px"]
                for edge in (image[y0, x0:x1+1], image[y1, x0:x1+1], image[y0:y1+1, x0], image[y0:y1+1, x1]):
                    self.assertTrue(np.all(edge == 255))
                # Outside the declared old/new patch regions the art is exact.
                source = read_image(asset_path(c["appearance"]["target"]["asset"]))
                changed = np.any(image != source, axis=2)
                changed[y0:y1+1, x0:x1+1] = False
                (x0, y0), (x1, y1) = report["erased_original_patch_px"]
                changed[y0:y1+1, x0:x1+1] = False
                self.assertFalse(np.any(changed))


class MultiRenderedSceneTests(unittest.TestCase):
    def test_both_default_configs_have_exactly_one_real_board_per_clock_direction(self):
        model_root = PACKAGE.parent/"src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root/"xgb.xml").is_file():
            self.skipTest("original xgb model not present")
        for filename in ("multi_fixed.json", "multi_official.json"):
            with self.subTest(config=filename), tempfile.TemporaryDirectory(prefix="three_boards_") as directory:
                c = json.loads((PACKAGE/"config"/filename).read_text(encoding="utf-8"))
                self.assertEqual(c["distractors"], [])
                self.assertEqual(len(c["targets"]), 3)
                model, _ = build_multi_scene(c, model_root, Path(directory))
                names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) for index in range(model.ngeom)]
                self.assertEqual(sum(bool(name and name.startswith("task3_board_visual_")) for name in names), 3)
                self.assertEqual(sum(bool(name and name.startswith("task3_target_collision_")) for name in names), 3)
                self.assertFalse(any(name and name.startswith("task3_distractor_") for name in names))
                directions = []
                for entry in c["targets"]:
                    center = entry["simulation"]["target_center_world_m"]
                    directions.append(float(np.arctan2(center[1], center[0])))
                np.testing.assert_allclose(sorted(directions), [-np.pi/2, 0, np.pi/2])

    def test_native_renderer_quarter_turns_see_clock_direction_tags_and_four_walls(self):
        c = config()
        model_root = PACKAGE.parent/"src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root/"xgb.xml").is_file():
            self.skipTest("original xgb model not present")
        with tempfile.TemporaryDirectory(prefix="multi_scene_") as directory:
            model, provenance = build_multi_scene(c, model_root, Path(directory))
            self.assertTrue(provenance["source_files_sha256"])
            self.assertFalse(provenance["official_files_modified"])
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
            self.assertEqual(len([name for name in names if name and name.startswith("task3_background_visual_")]), 4)
            placement = provenance["background_placement"]
            self.assertEqual(placement["layout"], "surround")
            self.assertEqual(len(placement["walls"]), 4)
            self.assertGreater(placement["room_half_extent_m"], placement["board_max_xy_corner_extent_m"])
            self.assertGreater(placement["room_half_extent_m"], placement["robot_max_xy_geometry_extent_m"])
            self.assertGreaterEqual(placement["wall_height_m"], 4.5)
            for direction in ("front", "left", "back", "right"):
                geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"task3_background_visual_{direction}")
                self.assertEqual(model.geom_group[geom], 1)
            for identity in (2, 5, 9):
                self.assertIn(f"task3_target_collision_{identity}", names)
            self.assertEqual(sum(bool(name and name.startswith("task3_target_collision_")) for name in names), 3)
            self.assertEqual(provenance["scoring_board_count"], 3)
            self.assertEqual(provenance["visual_only_distractor_tag_count"], 0)
            robot = MujocoRobot(model, c)
            try:
                robot.Connect()
                robot.StandUp()
                with AprilTagPerception(c) as perception:
                    for yaw, identity in ((0, 5), (np.pi/2, 2), (-np.pi/2, 9), (np.pi, None)):
                        robot.rpy[2] = yaw
                        robot._sync()
                        frame = robot.capture()
                        detections = perception.detect(frame)
                        if identity is not None:
                            self.assertEqual({d.tag_id for d in detections}, {identity})
                        self.assertNotIn(42, {d.tag_id for d in detections})
                        # The upper-center ray meets the supplied pastel wall,
                        # rather than the dark sky, in each cardinal direction.
                        self.assertGreater(float(frame.rgb[100, 480].mean()), 120)
                        for detection in detections:
                            self.assertGreater(detection.t_camera_from_tag[2], 1)
            finally:
                robot.close()

    def test_board_behind_robot_cannot_pull_background_in_front_of_valid_targets(self):
        c = config()
        # Old row configs deliberately have no surround selector; preserve
        # their single-wall behavior and the missing-board regression.
        c["appearance"]["background"].pop("layout")
        for target, y in zip(c["targets"], (-.85, 0, .85)):
            target["simulation"].update(target_center_world_m=[2.7, y, .5], target_yaw_rad=0)
        c["distractors"] = [{"id": 42, "simulation": {"target_center_world_m": [2.8, 1.65, .5], "target_yaw_rad": 0}}]
        c["targets"][1]["simulation"]["target_center_world_m"] = [-2.7, 0, .5]
        model_root = PACKAGE.parent/"src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root/"xgb.xml").is_file():
            self.skipTest("original xgb model not present")
        with tempfile.TemporaryDirectory(prefix="multi_scene_missing_") as directory:
            model, provenance = build_multi_scene(c, model_root, Path(directory))
            placement = provenance["background_placement"]
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
            self.assertEqual(names.count("task3_background_visual"), 1)
            normal = np.asarray(placement["normal_world"])
            front_projection = float(np.asarray(placement["anchor_world_m"]) @ normal) + placement["distance_behind_boards_m"]
            self.assertGreater(front_projection, placement["furthest_board_surface_projection_m"])
            self.assertGreater(front_projection, 2.8)
            robot = MujocoRobot(model, c)
            try:
                robot.Connect()
                robot.StandUp()
                with AprilTagPerception(c) as perception:
                    actual_ids = {d.tag_id for d in perception.detect(robot.capture())}
                self.assertTrue({2, 9}.issubset(actual_ids), actual_ids)
                self.assertNotIn(5, actual_ids)
            finally:
                robot.close()


if __name__ == "__main__":
    unittest.main()
