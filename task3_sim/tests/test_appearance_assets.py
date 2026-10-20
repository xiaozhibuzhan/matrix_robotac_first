"""Calibrated artwork, portable assets and genuine rendered tag regression."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import mujoco
import numpy as np

from task3_sim.appearance import PACKAGE, asset_path, photo_target_texture, read_image, validate_appearance
from task3_sim.backend import MujocoRobot
from task3_sim.geometry import estimate_aim
from task3_sim.perception import AprilTagPerception
from task3_sim.scene import build_scene, validate_config


class ArtworkTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((PACKAGE / "config/photo_target.json").read_text(encoding="utf-8"))
        self.provenance = json.loads((PACKAGE / "assets/provenance.json").read_text(encoding="utf-8"))

    def test_original_images_and_derived_image_match_provenance(self):
        for name, record in self.provenance["source_files"].items():
            data = (PACKAGE / "assets" / name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), record["sha256"])
            self.assertEqual(len(data), record["bytes"])
        self.assertFalse(self.provenance["target_tag"]["marker_replaced"])

    def test_whole_bitmap_rectification_preserves_id_and_square_marker(self):
        source_corners = np.array(self.provenance["target_tag"]["black_square_corners_px"])
        homography = np.array(self.provenance["rectification"]["source_to_rectified_px_homography"])
        expected = cv2.perspectiveTransform(source_corners[None], homography)[0]
        edges = np.linalg.norm(np.roll(expected, -1, axis=0) - expected, axis=1)
        np.testing.assert_allclose(edges, np.repeat(320.0, 4), atol=1e-4)
        with AprilTagPerception(self.config) as perception:
            source = read_image(PACKAGE / "assets/fixed_target_source.png")
            rectified = read_image(PACKAGE / "assets/fixed_target_rectified.png")
            for image in (source, rectified):
                results = perception.detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), estimate_tag_pose=False)
                self.assertEqual([(result.tag_id, result.hamming) for result in results], [(2, 0)])
            # Detection is LB,RB,RT,LT, while calibration is LT,RT,RB,LB.
            # Compare in source pixels, not arbitrary enlarged texture pixels:
            # interpolating this 320px input moves edge refinement by about one
            # source pixel (4x cubic resize alone measured 1.026px maximum).
            # Allow one sampling pixel plus 0.25px subpixel refinement. This is
            # an artwork calibration uncertainty of about 2.7mm, not metrology.
            inverse = np.linalg.inv(homography)
            measured_source = cv2.perspectiveTransform(results[0].corners[None], inverse)[0]
            source_error = np.linalg.norm(measured_source - source_corners[[3, 2, 1, 0]], axis=1)
            self.assertLessEqual(float(source_error.max()), 1.25)

    def test_config_dimensions_offset_and_scoring_use_same_calibration(self):
        validate_config(self.config)
        rectification = self.provenance["rectification"]
        sim = self.config["simulation"]
        for key in ("board_width_m", "board_height_m", "board_center_down_m"):
            self.assertEqual(sim[key], rectification[key])
        np.testing.assert_allclose(self.config["tag"]["bullseye_offset_tag_m"], self.provenance["bullseye"]["offset_tag_m"])
        self.assertEqual(sim["ring_ellipses_m"], self.provenance["printed_ring_fit"]["target_body_ring_ellipses_m"])

    def test_portable_assets_and_guarded_paths(self):
        self.assertEqual(asset_path("assets/fixed_target_source.png"), PACKAGE / "assets/fixed_target_source.png")
        for invalid in ("../README.md", "/tmp/image.png", "C:/image.png", "assets\\image.png"):
            with self.subTest(path=invalid), self.assertRaises(ValueError):
                asset_path(invalid)

    def test_brightness_changes_only_derived_texture(self):
        source = asset_path(self.config["appearance"]["target"]["asset"])
        before = hashlib.sha256(source.read_bytes()).digest()
        config = deepcopy(self.config)
        config["appearance"]["target"]["brightness"] = 0.5
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "target.png"
            report = photo_target_texture(config, output)
            np.testing.assert_array_equal(read_image(output), (read_image(source).astype(np.float32) * 0.5).astype(np.uint8))
            self.assertEqual(report["brightness"], 0.5)
        self.assertEqual(before, hashlib.sha256(source.read_bytes()).digest())

    def test_bad_appearance_parameters_are_rejected(self):
        for location, key, value in (("background", "brightness", float("nan")),
                                     ("background", "tile", [1, -1]),
                                     ("background", "distance_behind_target_m", 0),
                                     ("target", "kind", "replace_marker"),
                                     ("lighting", "ambient", 2)):
            config = deepcopy(self.config)
            config["appearance"][location][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_appearance(config)


class RenderedArtworkTests(unittest.TestCase):
    def test_original_marker_is_detected_from_mujoco_pixels_and_backdrop_cannot_block_shots(self):
        model_root = PACKAGE.parent / "src/robot_mujoco/zsibot_robots/xgb"
        if not (model_root / "xgb.xml").is_file():
            self.skipTest("official model not present; source-only package test")
        config = json.loads((PACKAGE / "config/photo_target.json").read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as folder:
            model, provenance = build_scene(config, model_root, Path(folder))
            robot = MujocoRobot(model, config)
            try:
                frame = robot.capture()
                with AprilTagPerception(config) as perception:
                    detections = perception.detect(frame)
                self.assertEqual([d.tag_id for d in detections], [2])
                self.assertEqual(detections[0].hamming, 0)
                self.assertGreater(detections[0].decision_margin, 40)
                estimated = estimate_aim(detections[0], config["camera"], config["laser"], config["tag"])
                # Perspective and the real bitmap share the metric calibration.
                truth_base = np.array(config["simulation"]["target_center_world_m"]) - robot.position
                from task3_sim.scene import rpy_matrix
                truth_base = rpy_matrix(robot.rpy).T @ truth_base
                self.assertLess(np.linalg.norm(estimated.bullseye_base_m - truth_base), 0.025)
                background_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "task3_background_visual")
                self.assertEqual(model.geom_group[background_id], 1)
                self.assertEqual(model.geom_contype[background_id], 0)
                self.assertEqual(model.geom_conaffinity[background_id], 0)
                # Explicit ray ignores the visual-only backdrop behind the board.
                geom_id = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(model, robot.data, np.array([0, 2, 1.0]), np.array([1., 0, 0]),
                                         np.array([1, 0, 1, 0, 1, 1], dtype=np.uint8), 1, -1, geom_id)
                self.assertEqual(distance, -1)
                self.assertEqual(provenance["appearance"]["target"]["asset"], "assets/fixed_target_rectified.png")
            finally:
                robot.close()


if __name__ == "__main__":
    unittest.main()
