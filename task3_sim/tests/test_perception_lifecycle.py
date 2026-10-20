"""Regression coverage for measured pose and native AprilTag resource ownership."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import textwrap
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from task3_sim.interfaces import Frame
from task3_sim.perception import (
    AprilTagPerception,
    _release_native_detector,
    estimate_tag_pose_from_corners,
)


PARAMS = (650.0, 650.0, 480.0, 360.0)
SIZE = 0.16
CONFIG = {
    "camera": dict(zip(("fx", "fy", "cx", "cy"), PARAMS)),
    "tag": {"size_m": SIZE, "max_reprojection_error_px": 2.0},
}


def projected_corners(rvec, translation):
    half = SIZE / 2
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=float)
    matrix = np.array([[650, 0, 480], [0, 650, 360], [0, 0, 1]], dtype=float)
    return cv2.projectPoints(points, np.asarray(rvec, dtype=float),
                             np.asarray(translation, dtype=float), matrix, None)[0].reshape(4, 2)


class PoseTests(unittest.TestCase):
    def test_front_oblique_and_rotated_tag_preserve_camera_and_tag_axes(self):
        translation = np.array([0.12, -0.2, 2.4])
        for rvec in ([0, 0, 0], [0.1, -0.25, 0.04],
                     [0.23, 0.33, -1.2], [0, 0, np.pi]):
            with self.subTest(rvec=rvec):
                corners = projected_corners(rvec, translation)
                pose = estimate_tag_pose_from_corners(corners, PARAMS, SIZE)
                self.assertIsNotNone(pose)
                rotation, measured_translation, error = pose
                expected_rotation = cv2.Rodrigues(np.asarray(rvec, dtype=float))[0]
                np.testing.assert_allclose(rotation, expected_rotation, atol=1e-7)
                np.testing.assert_allclose(measured_translation, translation, atol=1e-7)
                # Positive tag Y must continue to move toward the bullseye.
                np.testing.assert_allclose(rotation @ [0, 0.2, 0],
                                           expected_rotation @ [0, 0.2, 0], atol=1e-7)
                self.assertLess(error, 1e-7)

    def test_degenerate_or_nonfinite_pixels_cannot_produce_a_pose(self):
        for corners in (np.zeros((4, 2)), np.full((4, 2), np.nan),
                        np.array([[0, 0], [1, 0], [2, 0], [3, 0]])):
            self.assertIsNone(estimate_tag_pose_from_corners(corners, PARAMS, SIZE))


class NativeOwnershipTests(unittest.TestCase):
    def test_detector_is_destroyed_while_family_lives_and_close_is_idempotent(self):
        events = []
        pointer, family_pointer = object(), object()

        def destroy_detector(value):
            self.assertIs(value, pointer)
            self.assertEqual(events, [])  # family must still exist
            events.append("detector")

        def destroy_family(value):
            self.assertIs(value, family_pointer)
            self.assertEqual(events, ["detector"])
            events.append("family")

        library = SimpleNamespace(apriltag_detector_destroy=destroy_detector,
                                  tag36h11_destroy=destroy_family)
        detector = SimpleNamespace(tag_detector_ptr=pointer, libc=library,
                                   tag_families={"tag36h11": family_pointer})
        _release_native_detector(detector)
        _release_native_detector(detector)
        self.assertEqual(events, ["detector", "family"])
        self.assertIsNone(detector.tag_detector_ptr)
        self.assertEqual(detector.tag_families, {})

    @unittest.skipUnless(importlib.util.find_spec("pupil_apriltags"), "pupil-apriltags not installed")
    def test_real_detector_returns_ids_and_corners_without_native_pose_call(self):
        corners = projected_corners([0.1, -0.25, 0.04], [0.12, -0.2, 2.4])
        result = SimpleNamespace(tag_id=0, decision_margin=100, hamming=0, corners=corners)
        with AprilTagPerception(CONFIG) as perception:
            perception.detector.detect = Mock(return_value=[result])
            observations = perception.detect(Frame(np.zeros((8, 8, 3), np.uint8), 1.25))
            self.assertEqual(len(observations), 1)
            self.assertEqual(observations[0].tag_id, 0)
            self.assertEqual(observations[0].timestamp, 1.25)
            self.assertEqual(perception.detector.detect.call_args.kwargs, {"estimate_tag_pose": False})
        perception.close()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            perception.detect(Frame(np.zeros((8, 8, 3), np.uint8), 2))

    @unittest.skipUnless(importlib.util.find_spec("pupil_apriltags"), "pupil-apriltags not installed")
    def test_bad_reprojection_is_rejected_before_control(self):
        corners = projected_corners([0.1, -0.25, 0.04], [0.12, -0.2, 2.4])
        corners[0] += [20, 40]
        result = SimpleNamespace(tag_id=0, decision_margin=100, hamming=0, corners=corners)
        with AprilTagPerception(CONFIG) as perception:
            perception.detector.detect = Mock(return_value=[result])
            self.assertEqual(perception.detect(Frame(np.zeros((8, 8, 3), np.uint8), 0)), [])

    @unittest.skipUnless(importlib.util.find_spec("pupil_apriltags"), "pupil-apriltags not installed")
    def test_repeated_real_detection_and_native_teardown_exit_cleanly(self):
        # Use a subprocess: a glibc/CRT heap error must fail this test by its
        # exit status instead of aborting the rest of the suite.
        script = textwrap.dedent("""
            import cv2, gc, numpy as np
            from task3_sim.interfaces import Frame
            from task3_sim.perception import AprilTagPerception
            config = {'camera': {'fx':650,'fy':650,'cx':320,'cy':240},
                      'tag': {'size_m':.16,'detector_threads':2}}
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            marker = cv2.rotate(cv2.aruco.generateImageMarker(dictionary,0,120),cv2.ROTATE_180)
            image = np.full((480,640,3),255,np.uint8)
            image[180:300,260:380] = marker[:,:,None]
            for cycle in range(20):
                perception = AprilTagPerception(config)
                for frame in range(10):
                    observations = perception.detect(Frame(image,float(frame)))
                    assert len(observations)==1 and observations[0].tag_id==0
                    assert observations[0].reprojection_error_px < 2
                if cycle % 2 == 0:
                    perception.close()
                    perception.close()
                del perception  # also exercise implicit managed finalization
                gc.collect()
            # Exercise a live detector during interpreter shutdown as well.
            final_detector = AprilTagPerception(config)
            print('PASS: 200 real detections and 20 native lifecycles')
        """)
        result = subprocess.run([sys.executable, "-c", script],
                                cwd=Path(__file__).resolve().parents[2],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("200 real detections", result.stdout)
        self.assertNotIn("minima", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
