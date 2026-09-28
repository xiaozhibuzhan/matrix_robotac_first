"""ROS-free checks for lossless-input capture command and startup failures."""

import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture_mapping as capture


class CaptureMappingTests(unittest.TestCase):
    def test_capture_help_does_not_require_ros_or_numeric_packages(self):
        result = subprocess.run([sys.executable, "-B", "-S", str(Path(capture.__file__)), "--help"],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--bag-dir", result.stdout)

    def test_existing_capture_is_never_overwritten_or_started(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(capture.shutil, "which", return_value="/usr/bin/ros2"), \
                mock.patch.object(capture.os, "execv") as execute, \
                mock.patch("sys.stderr", new_callable=io.StringIO) as errors:
            bag = Path(temporary) / "raw_bag"
            bag.mkdir()
            existing = bag / "map_0.db3"
            existing.write_bytes(b"keep original capture")
            self.assertEqual(capture.main(["--bag-dir", str(bag)]), 1)
            execute.assert_not_called()
            self.assertEqual(existing.read_bytes(), b"keep original capture")
            self.assertFalse((bag.parent / "capture_qos.yaml").exists())
            self.assertIn("Refusing to overwrite", errors.getvalue())

    def test_record_command_matches_best_effort_sensors_and_bounds_recording_cache(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "raw_bag"
            command = capture.record_command(bag, "/custom/cloud", "/custom/odom", "/usr/bin/ros2")
            self.assertEqual(command[:5], ["/usr/bin/ros2", "bag", "record", "--storage", "sqlite3"])
            self.assertEqual(command[-2:], ["/custom/cloud", "/custom/odom"])
            self.assertEqual(command[command.index("--output") + 1], str(bag))
            self.assertEqual(command[command.index("--max-cache-size") + 1], "67108864")
            qos = json.loads(Path(command[command.index("--qos-profile-overrides-path") + 1]).read_text(encoding="utf-8"))
            self.assertEqual(set(qos), {"/custom/cloud", "/custom/odom"})
            for topic, depth in (("/custom/cloud", 100), ("/custom/odom", 2000)):
                self.assertEqual(qos[topic], {"history": "keep_last", "depth": depth,
                                             "reliability": "best_effort", "durability": "volatile"})
            self.assertFalse(bag.exists(), "rosbag must create its output itself")

    def test_recorder_replaces_its_pid_so_launcher_signals_reach_rosbag_directly(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(capture.shutil, "which", return_value="/usr/bin/ros2"), \
                mock.patch.object(capture.os, "execv") as execute, \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            bag = Path(temporary) / "new_run/raw_bag"
            self.assertEqual(capture.main(["--bag-dir", str(bag), "--topic", "/scan", "--odom-topic", "/pose"]), 0)
            executable, command = execute.call_args.args
            self.assertEqual(executable, "/usr/bin/ros2")
            self.assertEqual(command[0], executable)
            self.assertEqual(command[-2:], ["/scan", "/pose"])
            self.assertEqual(command[command.index("--output") + 1], str(bag.resolve()))
            self.assertTrue((bag.parent / "capture_qos.yaml").is_file())
            execute.assert_called_once()

    @unittest.skipUnless(os.name == "posix", "the Ubuntu exec/SIGINT contract requires POSIX")
    def test_real_exec_preserves_pid_and_sigint_flushes_the_recorder(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ros2 = root / "ros2"
            ros2.write_text(
                "#!/usr/bin/env python3\n"
                "import os, signal, sys, time\n"
                "from pathlib import Path\n"
                "bag = Path(sys.argv[sys.argv.index('--output') + 1])\n"
                "bag.mkdir()\n"
                "def stop(_signum, _frame):\n"
                "    (bag / 'metadata.yaml').write_text('flushed\\n')\n"
                "    raise SystemExit(0)\n"
                "signal.signal(signal.SIGINT, stop)\n"
                "(bag / 'ready').write_text(str(os.getpid()))\n"
                "while True: time.sleep(0.01)\n",
                encoding="utf-8",
            )
            ros2.chmod(0o755)
            bag = root / "raw_bag"
            environment = dict(os.environ, PATH=str(root) + os.pathsep + os.environ.get("PATH", ""))
            child = subprocess.Popen([sys.executable, "-B", str(Path(capture.__file__)), "--bag-dir", str(bag)],
                                     env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     start_new_session=True, text=True)
            try:
                deadline = time.monotonic() + 5
                while not (bag / "ready").exists() and child.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue((bag / "ready").exists(), "recorder did not finish startup")
                self.assertEqual(int((bag / "ready").read_text()), child.pid)
                child.send_signal(signal.SIGINT)
                stdout, stderr = child.communicate(timeout=5)
                self.assertEqual(child.returncode, 0, stdout + stderr)
                self.assertEqual((bag / "metadata.yaml").read_text(), "flushed\n")
            finally:
                if child.poll() is None:
                    child.kill()
                child.communicate(timeout=5)

    def test_missing_recorder_fails_before_allocating_capture(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(capture.shutil, "which", return_value=None), \
                mock.patch.object(capture.os, "execv") as execute, \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            bag = Path(temporary) / "uncreated/raw_bag"
            self.assertEqual(capture.main(["--bag-dir", str(bag)]), 1)
            self.assertFalse(bag.parent.exists())
            execute.assert_not_called()

    def test_invalid_or_duplicate_topics_cannot_start_capture(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(capture.shutil, "which", return_value="/usr/bin/ros2"), \
                mock.patch.object(capture.os, "execv") as execute, \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            bag = Path(temporary) / "raw_bag"
            for topics in (("scan", "/pose"), ("/scan", "pose"), ("/same", "/same")):
                with self.subTest(topics=topics):
                    self.assertEqual(capture.main(["--bag-dir", str(bag), "--topic", topics[0],
                                                   "--odom-topic", topics[1]]), 1)
            execute.assert_not_called()

    def test_exec_failure_returns_error_without_traceback_or_success(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(capture.shutil, "which", return_value="/usr/bin/ros2"), \
                mock.patch.object(capture.os, "execv", side_effect=OSError("cannot execute recorder")), \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO) as errors:
            self.assertEqual(capture.main(["--bag-dir", str(Path(temporary) / "raw_bag")]), 1)
            self.assertIn("Cannot start raw capture", errors.getvalue())
            self.assertNotIn("Traceback", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
