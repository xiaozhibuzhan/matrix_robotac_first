"""Offline cleaner tests; input and output files exist only in temporary dirs."""

from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


MAPPING = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("clean_pcd_test_subject", MAPPING / "clean_pcd.py")
cleaner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cleaner)


def pcd_bytes(points):
    count = len(points)
    return (("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
             f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n").encode("ascii")
            + b"".join(struct.pack("<fff", *point) for point in points))


class CleanPCDTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.source = Path(self.temporary.name) / "source.pcd"
        self.output = Path(self.temporary.name) / "cleaned.pcd"
        path_patch = mock.patch.object(sys, "path", [str(MAPPING), *sys.path])
        path_patch.start()
        self.addCleanup(path_patch.stop)

    def test_strict_reader_returns_little_endian_xyz(self):
        self.source.write_bytes(pcd_bytes([(1, 2, 3), (-4, 5, 6)]))
        np.testing.assert_array_equal(cleaner.read_pcd(self.source), [(1, 2, 3), (-4, 5, 6)])

    def test_reader_rejects_unsupported_or_inconsistent_format(self):
        original = pcd_bytes([(1, 2, 3)])
        malformed = [
            original.replace(b"DATA binary", b"DATA ascii"),
            original.replace(b"DATA binary", b"DATA binary_compressed"),
            original.replace(b"FIELDS x y z", b"FIELDS x y z intensity"),
            original.replace(b"FIELDS x y z", b"FIELDS z y x"),
            original.replace(b"SIZE 4 4 4", b"SIZE 8 8 8"),
            original.replace(b"TYPE F F F", b"TYPE I I I"),
            original.replace(b"COUNT 1 1 1", b"COUNT 2 1 1"),
            original.replace(b"HEIGHT 1", b"HEIGHT 2"),
            original.replace(b"WIDTH 1", b"WIDTH 2"),
            original.replace(b"POINTS 1", b"POINTS -1"),
            original.replace(b"VERSION 0.7", b"VERSION 0.6"),
            original.replace(b"HEIGHT 1", b"HEIGHT 1\nHEIGHT 1"),
            original.replace(b"VIEWPOINT 0 0 0 1 0 0 0", b"VIEWPOINT nan 0 0 1 0 0 0"),
            original[:-1],
            original + b"extra",
            original.split(b"DATA", 1)[0],
        ]
        for index, content in enumerate(malformed):
            with self.subTest(index=index):
                self.source.write_bytes(content)
                with self.assertRaises(ValueError):
                    cleaner.read_pcd(self.source)

    def test_cleaning_preserves_supported_pole_far_from_world_origin(self):
        pole = [(1000, 2000, z / 10) for z in range(11)]
        points = np.asarray([*pole, (2000, 3000, 4000), (float("nan"), 0, 0)], dtype=np.float32)
        cleaned, stats = cleaner.clean_points(points, radius=0.21, min_neighbors=2)
        np.testing.assert_array_equal(cleaned, np.asarray(pole, dtype=np.float32))
        self.assertEqual(stats, {"nonfinite": 1, "outside_bounds": 0, "isolated": 1})

    def test_bounds_are_explicit_and_inclusive(self):
        points = np.asarray([(0, 0, 0), (1, 1, 1), (2, 2, 2)], dtype=np.float32)
        cleaned, stats = cleaner.clean_points(points, radius=0, bounds=[0, 1, 0, 1, 0, 1])
        np.testing.assert_array_equal(cleaned, points[:2])
        self.assertEqual(stats["outside_bounds"], 1)
        for bounds in ([1, 0, 0, 1, 0, 1], [0, 1], [0, float("inf"), 0, 1, 0, 1]):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                cleaner.clean_points(points, bounds=bounds)

    def test_inspect_only_reports_and_never_creates_output(self):
        self.source.write_bytes(pcd_bytes([(1, 2, 3), (float("nan"), 0, 0)]))
        original = self.source.read_bytes()
        stream = io.StringIO()
        with redirect_stdout(stream):
            self.assertEqual(cleaner.main([str(self.source), "--inspect"]), 0)
        report = json.loads(stream.getvalue())
        self.assertEqual(report["source_summary"]["nonfinite_points"], 1)
        self.assertEqual(report["source_summary"]["bounds"], {"min": [1, 2, 3], "max": [1, 2, 3]})
        self.assertNotIn("output", report)
        self.assertEqual(list(self.source.parent.iterdir()), [self.source])
        self.assertEqual(self.source.read_bytes(), original)

    def test_clean_cli_creates_valid_copy_without_modifying_source(self):
        points = [(100, 200, z / 10) for z in range(11)]
        self.source.write_bytes(pcd_bytes([*points, (1000, 2000, 3000)]))
        original = self.source.read_bytes()
        stream = io.StringIO()
        with redirect_stdout(stream):
            result = cleaner.main([str(self.source), "--output", str(self.output), "--radius", "0.21"])
        self.assertEqual(result, 0)
        self.assertEqual(self.source.read_bytes(), original)
        np.testing.assert_array_equal(cleaner.read_pcd(self.output), np.asarray(points, dtype=np.float32))
        report = json.loads(stream.getvalue())
        self.assertEqual(report["removed"]["isolated"], 1)
        self.assertFalse(report["world_origin_range_filter"])

    def test_same_source_or_existing_output_is_rejected(self):
        self.source.write_bytes(pcd_bytes([(1, 2, 3)]))
        self.output.write_bytes(b"existing output")
        original = self.source.read_bytes()
        for output in (self.source, self.output):
            with self.subTest(output=output), mock.patch.object(sys, "stderr", io.StringIO()):
                with self.assertRaises(SystemExit) as failure:
                    cleaner.main([str(self.source), "--output", str(output)])
                self.assertEqual(failure.exception.code, 2)
        self.assertEqual(self.source.read_bytes(), original)
        self.assertEqual(self.output.read_bytes(), b"existing output")

    def test_exclusive_writer_rejects_existing_destination(self):
        self.output.write_bytes(b"existing output")
        with self.assertRaises(FileExistsError):
            cleaner.write_pcd_exclusive(self.output, np.empty((0, 3), dtype=np.float32))
        self.assertEqual(self.output.read_bytes(), b"existing output")

    def test_empty_cloud_roundtrips_and_has_null_bounds(self):
        cleaner.write_pcd_exclusive(self.output, np.empty((0, 3), dtype=np.float32))
        cloud = cleaner.read_pcd(self.output)
        self.assertEqual(cloud.shape, (0, 3))
        self.assertIsNone(cleaner.point_summary(cloud)["bounds"])


if __name__ == "__main__":
    unittest.main()
