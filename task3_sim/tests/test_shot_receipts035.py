"""Visible shot receipts and CSV preserve real debits, including zero scores."""
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np

from task3_sim.interfaces import Frame, ShotResult
from task3_sim.multi import (annotate_multi, print_shot_receipt, save_shot_reports,
                             shot_progress_lines, update_viewer_progress)
from task3_sim.tests.test_round_robin_mission import RoundDriver


class ShotReceiptTests(unittest.TestCase):
    def four_shots(self):
        driver = RoundDriver(ammo=5)
        for identity in (2, 5, 9, 2):
            decision = driver.shoot(identity)
        return driver, decision

    def test_four_receipts_show_40_and_each_debit_without_inferred_shot_count(self):
        driver, _ = self.four_shots()
        report = driver.mission.summary()
        text = "\n".join(shot_progress_lines(report, driver.ammo))
        for expected in ("Shots fired: 4", "Total score: 40", "Ammo remaining: 1",
                         "ID 2: shots=2 sum=20 latest=10", "#4 R2 ID2 -> 2  +10", "ammo 2->1  total=40"):
            self.assertIn(expected, text)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            print_shot_receipt(report, driver.ammo)
        self.assertIn("TASK3_SHOT index=4 round=2 intended_id=2 actual_id=2 ring=10 credited=10 total_score=40", output.getvalue())
        self.assertIn("ammo_before=2 ammo_remaining=1", output.getvalue())

    def test_csv_has_four_rows_even_if_the_run_later_ends_in_error(self):
        driver, _ = self.four_shots()
        report = driver.mission.summary()
        physical = [{"sequence": index+1} for index in range(4)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            save_shot_reports(output, report, physical)
            with (output/"shots.csv").open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            self.assertEqual([row["intended_target_id"] for row in rows], ["2", "5", "9", "2"])
            self.assertEqual([row["cumulative_score_after"] for row in rows], ["10", "20", "30", "40"])
            self.assertEqual([row["ammo_remaining"] for row in rows], ["4", "3", "2", "1"])
            self.assertEqual(json.loads((output/"mission.json").read_text(encoding="utf-8")), report)
            self.assertEqual(json.loads((output/"shots.json").read_text(encoding="utf-8")), physical)

    def test_miss_counts_as_one_shot_without_adding_any_score(self):
        driver = RoundDriver(ammo=5)
        driver.shoot(2)
        driver.shoot(5, result=ShotResult(None, 3, False, reason="occluded"))
        report = driver.mission.summary()
        text = "\n".join(shot_progress_lines(report, driver.ammo))
        self.assertIn("Shots fired: 2", text)
        self.assertIn("Total score: 10", text)
        self.assertIn("#2 R1 ID5 -> None  +0", text)

    def test_rejected_mission_commit_still_exports_the_actual_physical_debit(self):
        physical = [{"sequence": 1, "intended_target_id": 2, "actual_target_id": 2,
                     "ring": 10, "ammo_before": 3, "ammo_after": 2, "timestamp_s": 1.5}]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            save_shot_reports(output, {"shots": [], "total_score": 0}, physical)
            with (output/"shots.csv").open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["mission_accepted"], "False")
            self.assertEqual(rows[0]["credit_status"], "unconfirmed")
            self.assertEqual(rows[0]["ring"], "10")
            self.assertEqual(rows[0]["credited_ring"], "")
            self.assertEqual(rows[0]["ammo_remaining"], "2")
            self.assertEqual(rows[0]["cumulative_score_after"], "0")

    def test_camera_sidebar_and_native_viewer_use_committed_receipts_without_mutating_raw_image(self):
        driver, decision = self.four_shots()
        report = driver.mission.summary()
        rgb = np.full((720, 960, 3), 80, dtype=np.uint8)
        camera = {**driver.source["camera"], "fx": 650, "fy": 650, "cx": 480, "cy": 360}
        image = annotate_multi(Frame(rgb, driver.now), [], decision, camera, report, 1)
        self.assertEqual(image.shape, (720, 1330, 3))
        self.assertTrue(np.all(rgb == 80))
        self.assertTrue(np.any(image[:, 960:] != 20))
        viewer = Mock()
        update_viewer_progress(viewer, report, 1)
        overlay = viewer.set_texts.call_args.args[0][2]
        self.assertIn("Shots fired: 4", overlay)
        self.assertIn("#4 R2 ID2", overlay)

    def test_production_default_is_three_and_test_magazines_are_explicit(self):
        package = Path(__file__).resolve().parents[1]
        for name in ("multi_fixed.json", "multi_official.json"):
            config = json.loads((package/"config"/name).read_text(encoding="utf-8"))
            self.assertEqual(config["simulation"]["ammo_count"], 3)
            self.assertEqual(config["mission"]["shooting_policy"], "round_robin")


if __name__ == "__main__":
    unittest.main()
