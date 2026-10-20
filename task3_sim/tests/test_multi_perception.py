"""Each decoded ID must use its own tag scale, without trusting scene truth."""
import unittest
import cv2
import numpy as np

from task3_sim.interfaces import Frame
from task3_sim.multi_perception import MultiTagPerception


class MultiPerceptionTests(unittest.TestCase):
    def test_same_pixel_size_uses_individual_metric_calibration(self):
        config = {"camera": {"fx": 650, "fy": 650, "cx": 320, "cy": 240},
                  "tag": {"family": "tag36h11", "size_m": 0.16},
                  "targets": [{"id": 2, "tag": {"size_m": 0.10}}, {"id": 5, "tag": {"size_m": 0.20}}]}
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for target_id, x in ((2, 100), (5, 390)):
            marker = cv2.rotate(cv2.aruco.generateImageMarker(dictionary, target_id, 100), cv2.ROTATE_180)
            image[180:280, x:x+100] = marker[:, :, None]
        with MultiTagPerception(config) as perception:
            detections = {detection.tag_id: detection for detection in perception.detect(Frame(image, 2.5))}
        self.assertEqual(set(detections), {2, 5})
        self.assertAlmostEqual(float(detections[5].t_camera_from_tag[2]/detections[2].t_camera_from_tag[2]), 2.0, delta=0.02)
        self.assertTrue(all(detection.timestamp == 2.5 for detection in detections.values()))


if __name__ == "__main__":
    unittest.main()
