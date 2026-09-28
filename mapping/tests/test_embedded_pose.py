from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from embedded_pose import decode_embedded_pose
from field_mapper_node import rotate_vector


class EmbeddedPoseTests(unittest.TestCase):
    def test_decodes_signs_and_strips_all_metadata(self):
        # Encoded p1=(100,-200,30) cm -> (1,2,.3) m.
        # Encoded p2=(roll,-pitch,-yaw)=(10,-20,-30) degrees.
        points = np.array([[100, -200, 30], [10, -20, -30], [1, 2, 3], [4, 5, 6]], float)
        intensity = np.array([111, 111, 4, 111], float)
        result = decode_embedded_pose(points, intensity)
        self.assertEqual(result.reason, "ok")
        self.assertEqual(result.metadata_mask.tolist(), [True, True, False, True])
        np.testing.assert_allclose(result.points, [[1, 2, 3]])
        self.assertIsNotNone(result.pose)
        pose = result.pose
        np.testing.assert_allclose(pose[:3], [1, 2, .3])
        expected = Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_quat()
        np.testing.assert_allclose(pose[3:], expected, atol=1e-12)
        np.testing.assert_allclose(rotate_vector((1, 0, 0), pose[3:]),
                                   Rotation.from_euler("xyz", [10, 20, 30], degrees=True).apply([1, 0, 0]))

    def test_missing_marker_leaves_geometry_unchanged(self):
        points = np.array([[1, 2, 3], [4, 5, 6]], dtype=float)
        result = decode_embedded_pose(points, [1, 2])
        self.assertEqual(result.reason, "missing_embedded_pose")
        self.assertIsNone(result.pose)
        np.testing.assert_array_equal(result.points, points)
        self.assertFalse(result.metadata_mask.any())

    def test_invalid_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_embedded_pose([[1, 2]], [111])


if __name__ == "__main__":
    unittest.main()
