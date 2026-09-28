"""Output byte transport and recoverable backlogs must preserve measurements."""

from array import array
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_field_mapper import mapper, Odometry, PointCloud2


class OutputQueueTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.node = mapper.FieldMapperNode(mapper.build_parser().parse_args([
            "--pcd-file", str(Path(temporary.name) / "map.pcd"),
            "--outlier-radius", "0", "--disable-ground"]))

    def test_ordered_queue_preserves_three_observations_during_short_backlog(self):
        for tick in range(480, 530):
            self.node.odom_callback(Odometry(tick * .02))
        with mock.patch.object(mapper.time, "monotonic", return_value=100):
            for stamp in (10.0, 10.1, 10.2):
                self.node.cloud_callback(PointCloud2([[2.005, 1.005, .5]], stamp))
            for _ in range(3):
                self.node.process_pending()
        self.assertEqual(self.node.frames, 3)
        self.assertEqual(len(self.node.voxels), 1)
        self.assertFalse(self.node.cloud_queue)
        self.assertFalse(self.node.rejection_reasons)

    def test_uint8_output_uses_array_fast_path_and_exact_xyz_payload(self):
        expected = np.asarray([[1.2, -3.4, .006], [20.01, 0, 2.3]], dtype="<f4")
        for xyz in (expected, expected[:0]):
            cloud = self.node.cloud_message(xyz)
            self.assertEqual(type(cloud.data).__name__, "array")
            self.assertEqual(cloud.data.typecode, "B")
            self.assertEqual(bytes(cloud.data), xyz.tobytes())
            self.assertEqual(len(cloud.data), cloud.row_step * cloud.height)
            np.testing.assert_array_equal(np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3), xyz)


if __name__ == "__main__":
    unittest.main()
