"""PGM/map_server occupancy export tests."""

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from occupancy_map import build_occupancy


class OccupancyMapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def test_writes_binary_pgm_and_ros_map_yaml_with_lower_left_origin(self):
        # Ground is free, upper wall is occupied, and the gap stays unknown.
        points = np.array([
            [0.02, 0.02, 0.00], [0.08, 0.02, 0.01],
            [0.02, 0.13, 0.30], [0.08, 0.13, 0.40],
        ])
        result = build_occupancy(points, image_path=self.root / "field_map.pgm",
                                 resolution=.05, ground_band=.05, obstacle_height=.15,
                                 padding=0)
        self.assertEqual((result.width, result.height), (2, 3))
        self.assertEqual((result.free_cells, result.occupied_cells, result.unknown_cells), (2, 2, 2))
        payload = (self.root / "field_map.pgm").read_bytes()
        header, raster = payload.split(b"255\n", 1)
        self.assertTrue(header.startswith(b"P5\n2 3\n"))
        self.assertEqual(len(raster), 6)
        # PGM rows run from max-Y to min-Y; the ground row is at the bottom.
        self.assertEqual(list(raster), [0, 0, 205, 205, 254, 254])
        yaml = (self.root / "field_map.yaml").read_text(encoding="utf-8")
        self.assertIn("image: field_map.pgm\n", yaml)
        self.assertIn("resolution: 0.05\n", yaml)
        self.assertIn("origin: [0.02, 0.02, 0.0]\n", yaml)
        self.assertIn("negate: 0\n", yaml)

    def test_obstacle_wins_when_a_cell_has_both_ground_and_structure(self):
        result = build_occupancy([[0, 0, 0], [0, 0, 1]], image_path=self.root / "map.pgm",
                                 resolution=1, ground_band=.1, obstacle_height=.15, padding=0)
        self.assertEqual((result.free_cells, result.occupied_cells), (0, 1))
        self.assertEqual((self.root / "map.pgm").read_bytes().split(b"255\n", 1)[1], b"\x00")

    def test_empty_or_invalid_inputs_fail_without_creating_files(self):
        with self.assertRaises(ValueError):
            build_occupancy([], image_path=self.root / "empty.pgm")
        with self.assertRaises(ValueError):
            build_occupancy([[0, 0, np.nan]], image_path=self.root / "nan.pgm")
        self.assertFalse((self.root / "empty.pgm").exists())
        self.assertFalse((self.root / "nan.pgm").exists())


if __name__ == "__main__":
    unittest.main()
