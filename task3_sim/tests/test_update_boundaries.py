"""Exercise migration boundaries using disposable files, never the real project."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from task3_sim.package_update import make_package
from task3_sim.preserve import inventory


class UpdateBoundaryTests(unittest.TestCase):
    def test_archive_keeps_local_environment_and_original_sources_out(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "task3_sim"
            for filename in ("main.py", ".gitignore", "config/single_target.json", "local/calibration.json",
                             "config/camera_local.json", ".venv/python.py", "runs/result.json",
                             "updates/old.py", "__pycache__/cached.py", "runs_ubuntu/summary.json"):
                target = source / filename
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text("{}", encoding="utf-8")
            official = root / "src/official.py"
            official.parent.mkdir()
            official.write_text("original", encoding="utf-8")
            output = root / "update.zip"
            make_package(source, output)
            with zipfile.ZipFile(output) as archive:
                self.assertEqual(set(archive.namelist()), {"task3_sim/main.py", "task3_sim/.gitignore", "task3_sim/config/single_target.json",
                                                         "task3_sim/PACKAGE_MANIFEST.json"})
                manifest = json.loads(archive.read("task3_sim/PACKAGE_MANIFEST.json"))
                self.assertEqual(len(manifest["files"]), 3)
            self.assertEqual(official.read_text(encoding="utf-8"), "original")

    def test_checksum_and_shell_scripts_use_unix_newlines_on_every_host(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "task3_sim"
            source.mkdir()
            shell_source = b"#!/usr/bin/env bash\r\nset -euo pipefail\r\n"
            (source / "run_task3.sh").write_bytes(shell_source)
            output = root / "update.zip"
            result = make_package(source, output)
            digest = hashlib.sha256(output.read_bytes()).hexdigest()
            self.assertEqual(result["sha256"], digest)
            self.assertEqual(output.with_suffix(".zip.sha256").read_bytes(),
                             f"{digest}  update.zip\n".encode("ascii"))
            self.assertEqual((source / "run_task3.sh").read_bytes(), shell_source)
            with zipfile.ZipFile(output) as archive:
                data = archive.read("task3_sim/run_task3.sh")
                self.assertEqual(data, shell_source.replace(b"\r\n", b"\n"))
                manifest = json.loads(archive.read("task3_sim/PACKAGE_MANIFEST.json"))
                self.assertEqual(manifest["files"]["task3_sim/run_task3.sh"],
                                 hashlib.sha256(data).hexdigest())

    def test_preservation_detects_content_change_even_with_same_file_size(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "model.xml").write_text("first", encoding="utf-8")
            before = inventory(root, ["model.xml"])
            (root / "model.xml").write_text("later", encoding="utf-8")
            after = inventory(root, ["model.xml"])
            self.assertEqual(before["files"]["model.xml"]["size"], after["files"]["model.xml"]["size"])
            self.assertNotEqual(before["files"], after["files"])


if __name__ == "__main__":
    unittest.main()
