"""Ground evidence must separate surface orientation and protect source data."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import inspect_ground_pcd as inspector
from clean_pcd import write_pcd_exclusive


def plane(horizontal: bool) -> np.ndarray:
    u, v = np.meshgrid(np.linspace(-1, 1, 61), np.linspace(-1, 1, 61))
    if horizontal:
        return np.column_stack((u.ravel(), v.ravel(), np.zeros(u.size)))
    return np.column_stack((u.ravel(), np.zeros(u.size), v.ravel() + 1.0))


class GroundGeometryTests(unittest.TestCase):
    def test_horizontal_ground_supported_but_wall_bases_are_not_ground(self):
        ground_report, low, supported = inspector.analyze(plane(True))
        self.assertTrue(supported.all())
        self.assertGreater(len(low), 3000)
        self.assertEqual(ground_report["local_horizontal_surface_evidence"]
                         ["height_band_points_with_horizontal_patch_support"], len(low))
        wall_report, low, supported = inspector.analyze(plane(False))
        self.assertGreater(len(low), 0)
        self.assertFalse(supported.any())
        self.assertGreater(wall_report["local_horizontal_surface_evidence"]
                           ["height_band_points_with_neighbor_support"], 0)

    def test_sparse_low_points_do_not_claim_a_supported_ground_surface(self):
        sparse = plane(True) * [30, 30, 1]
        report, low, supported = inspector.analyze(sparse)
        self.assertEqual(len(low), len(sparse))
        self.assertFalse(supported.any())
        self.assertEqual(report["local_horizontal_surface_evidence"]
                         ["height_band_points_with_neighbor_support"], 0)
        _, low, supported = inspector.analyze([[0, 0, 0], [0.01, 0, 0]])
        self.assertEqual(len(low), 2)
        self.assertFalse(supported.any())

    def test_empty_and_nonfinite_clouds_return_empty_evidence(self):
        for points in ([], np.empty((0, 3)), [[np.nan, 0, 0], [0, np.inf, 0]]):
            with self.subTest(points=points):
                report, low, supported = inspector.analyze(points)
                self.assertEqual(report["finite_points"], 0)
                self.assertIsNone(report["bounds_xyz_m"])
                self.assertEqual(low.shape, (0, 3))
                self.assertEqual(len(supported), 0)
                self.assertEqual(report["ground_height_band"]["height_band_xy_cells"], 0)

    def test_custom_ground_height_is_respected_without_mutating_points(self):
        points = plane(True) + [0, 0, 2]
        original = points.copy()
        _, default_low, _ = inspector.analyze(points)
        _, shifted_low, supported = inspector.analyze(points, ground_z=2)
        self.assertEqual(len(default_low), 0)
        self.assertEqual(len(shifted_low), len(points))
        self.assertTrue(supported.all())
        np.testing.assert_array_equal(points, original)

    def test_invalid_parameters_and_shapes_are_rejected(self):
        for parameters in ({"ground_z": np.nan}, {"tolerance": 0}, {"cell_size": -1},
                           {"radius": np.inf}, {"neighbors": 2}, {"neighbors": 3.5},
                           {"neighbors": True}):
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                inspector.analyze([], **parameters)
        for points in ([1, 2, 3], [[1, 2]], np.empty((0, 2))):
            with self.subTest(points=points), self.assertRaises(ValueError):
                inspector.analyze(points)


class GroundExportTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.source = self.directory / "source.pcd"
        write_pcd_exclusive(self.source, plane(True))
        self.original = self.source.read_bytes()

    def assert_cli_error(self, argv, message):
        stderr = io.StringIO()
        with redirect_stderr(stderr), self.assertRaises(SystemExit) as failure:
            inspector.main(argv)
        self.assertEqual(failure.exception.code, 2)
        self.assertIn(message, stderr.getvalue())
        self.assertNotIn("Traceback", stderr.getvalue())

    def test_existing_directory_or_source_as_output_is_rejected_before_analysis(self):
        output = self.directory / "existing"
        output.mkdir()
        report = output / "ground_report.json"
        image = output / "ground_evidence.png"
        report.write_bytes(b"existing report")
        image.write_bytes(b"existing image")
        for destination in (output, self.directory, self.source, report, image):
            with self.subTest(destination=destination), mock.patch.object(inspector, "read_pcd") as read:
                self.assert_cli_error([str(self.source), "--output-dir", str(destination)],
                                      "new, nonexistent directory")
                read.assert_not_called()
        self.assertEqual(self.source.read_bytes(), self.original)
        self.assertEqual(report.read_bytes(), b"existing report")
        self.assertEqual(image.read_bytes(), b"existing image")

    def test_read_only_mode_reports_json_and_creates_nothing(self):
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            self.assertEqual(inspector.main([str(self.source)]), 0)
        report = json.loads(stdout.getvalue())
        self.assertEqual(report["finite_points"], len(plane(True)))
        self.assertEqual(list(self.directory.iterdir()), [self.source])
        self.assertEqual(self.source.read_bytes(), self.original)

    def test_new_output_directory_exports_report_without_touching_source(self):
        output = self.directory / "new" / "inspection"
        def write_image(_points, _low, _horizontal, _report, destination):
            with destination.open("xb") as image:
                image.write(b"test image")
        with redirect_stdout(io.StringIO()), mock.patch.object(inspector, "plot_report", side_effect=write_image):
            self.assertEqual(inspector.main([str(self.source), "--output-dir", str(output)]), 0)
        report = json.loads((output / "ground_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["finite_points"], len(plane(True)))
        self.assertEqual((output / "ground_evidence.png").read_bytes(), b"test image")
        self.assertEqual(self.source.read_bytes(), self.original)

    def test_bad_input_and_missing_plot_dependency_have_concise_cli_errors(self):
        self.assert_cli_error([str(self.directory / "missing.pcd")], "missing.pcd")
        self.source.write_bytes(b"not a PCD\n")
        self.assert_cli_error([str(self.source)], "PCD")
        self.source.write_bytes(self.original)
        with mock.patch.object(inspector, "plot_report", side_effect=ImportError("No module named matplotlib")):
            self.assert_cli_error([str(self.source), "--output-dir", str(self.directory / "new")], "matplotlib")
        self.assertEqual(self.source.read_bytes(), self.original)


if __name__ == "__main__":
    unittest.main()
