"""Integration checks against the exact official runtime; no running UE needed."""
import copy
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from scene_tools import treasure_layout as tool


class TreasureLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = tool.read_layout(tool.DEFAULT_LAYOUT)
        project = Path(os.environ.get("TREASURE_TEST_PROJECT", Path(__file__).resolve().parents[2]))
        stem = project / cls.layout["relative_stem"]
        if not stem.with_suffix(".uexp").is_file():
            raise unittest.SkipTest("Official runtime required; set TREASURE_TEST_PROJECT")
        cls.umap = stem.with_suffix(".umap").read_bytes()
        source = bytearray(stem.with_suffix(".uexp").read_bytes())
        if tool.inspect(cls.umap, source, cls.layout)["state"] == "PATCHED":
            for box in cls.layout["boxes"]:
                struct.pack_into("<dd", source, box["xyz_offset"], *box["original_ue_cm"][:2])
        cls.original = bytes(source)
        assert tool.inspect(cls.umap, cls.original, cls.layout)["state"] == "ORIGINAL"

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.project = Path(self.temporary.name)
        stem = self.project / self.layout["relative_stem"]
        stem.parent.mkdir(parents=True)
        self.umap_path, self.uexp_path = stem.with_suffix(".umap"), stem.with_suffix(".uexp")
        self.umap_path.write_bytes(self.umap)
        self.uexp_path.write_bytes(self.original)
        self.backup = self.project / "scene_tools/backups" / self.layout["layout_id"]
        self.guard = patch.object(tool, "ensure_simulator_stopped")
        self.guard.start()
        self.addCleanup(self.guard.stop)

    def operate(self, action):
        return tool.operate(self.project, action, self.layout)

    def test_apply_changes_only_five_xy_fields_and_preserves_other_bytes(self):
        self.assertEqual(self.operate("apply")["state"], "PATCHED")
        current = self.uexp_path.read_bytes()
        self.assertEqual(len(current), len(self.original))
        self.assertEqual(self.umap_path.read_bytes(), self.umap)
        allowed = {i for b in self.layout["boxes"] for i in range(b["xyz_offset"], b["xyz_offset"] + 16)}
        changed = {i for i, (a, b) in enumerate(zip(self.original, current)) if a != b}
        self.assertTrue(changed)
        self.assertFalse(changed - allowed)
        for box in self.layout["boxes"]:
            off = box["xyz_offset"]
            self.assertEqual(list(struct.unpack_from("<ddd", current, off)), box["patched_ue_cm"])
            self.assertEqual(current[off + 16:off + 24], self.original[off + 16:off + 24])
        self.assertEqual((self.backup / self.uexp_path.name).read_bytes(), self.original)

    def test_apply_is_idempotent_and_preserves_backup(self):
        self.operate("apply")
        patched = self.uexp_path.read_bytes()
        self.operate("apply")
        self.assertEqual(patched, self.uexp_path.read_bytes())
        self.assertEqual((self.backup / self.uexp_path.name).read_bytes(), self.original)

    def test_restore_round_trip_recovers_exact_original(self):
        self.operate("apply")
        self.assertEqual(self.operate("restore")["state"], "ORIGINAL")
        self.assertEqual(self.uexp_path.read_bytes(), self.original)
        self.assertEqual(self.umap_path.read_bytes(), self.umap)
        self.assertEqual(self.operate("restore")["state"], "ORIGINAL")

    def test_status_does_not_create_backup_or_change_resource(self):
        self.assertEqual(self.operate("status")["state"], "ORIGINAL")
        self.assertFalse(self.backup.exists())
        self.assertEqual(self.uexp_path.read_bytes(), self.original)

    def test_unknown_umap_is_rejected_before_backup_or_write(self):
        damaged = b"x" + self.umap[1:]
        self.umap_path.write_bytes(damaged)
        with self.assertRaisesRegex(ValueError, "umap version"):
            self.operate("apply")
        self.assertFalse(self.backup.exists())
        self.assertEqual(self.uexp_path.read_bytes(), self.original)

    def test_unknown_export_content_is_rejected_before_write(self):
        damaged = self.original[:-1] + bytes([self.original[-1] ^ 1])
        self.uexp_path.write_bytes(damaged)
        with self.assertRaisesRegex(ValueError, "uexp hash"):
            self.operate("apply")
        self.assertFalse(self.backup.exists())
        self.assertEqual(self.uexp_path.read_bytes(), damaged)

    def test_existing_bad_backup_prevents_apply(self):
        self.backup.mkdir(parents=True)
        (self.backup / self.uexp_path.name).write_bytes(b"not an original")
        with self.assertRaisesRegex(ValueError, "Backup differs"):
            self.operate("apply")
        self.assertEqual(self.uexp_path.read_bytes(), self.original)

    def test_corrupt_backup_prevents_restore(self):
        self.operate("apply")
        current = self.uexp_path.read_bytes()
        (self.backup / self.uexp_path.name).write_bytes(b"corrupt")
        with self.assertRaisesRegex(ValueError, "uexp size"):
            self.operate("restore")
        self.assertEqual(self.uexp_path.read_bytes(), current)

    def test_patched_copy_without_backup_can_create_verified_original_backup(self):
        self.uexp_path.write_bytes(tool.make_patch(self.umap, self.original, self.layout))
        self.operate("apply")
        self.assertEqual((self.backup / self.uexp_path.name).read_bytes(), self.original)
        self.operate("restore")
        self.assertEqual(self.uexp_path.read_bytes(), self.original)

    def test_wrong_export_offset_is_rejected(self):
        invalid = copy.deepcopy(self.layout)
        invalid["boxes"][0]["xyz_offset"] += 1
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            tool.operate(self.project, "apply", invalid)
        self.assertEqual(self.uexp_path.read_bytes(), self.original)

    def test_running_simulator_prevents_mutation(self):
        with patch.object(tool, "ensure_simulator_stopped", side_effect=ValueError("UE simulator still running")):
            with self.assertRaisesRegex(ValueError, "still running"):
                self.operate("apply")
        self.assertFalse(self.backup.exists())
        self.assertEqual(self.uexp_path.read_bytes(), self.original)


if __name__ == "__main__":
    unittest.main()
