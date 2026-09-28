"""ROS-free checks for the combined mapper/RViz launcher."""

from contextlib import ExitStack
import hashlib
import io
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_mapping_rviz as launcher


class MappingLauncherTests(unittest.TestCase):
    @staticmethod
    def finish_capture(_mapper, _rviz, _root, *, log_directory, save_timeout):
        metadata = json.loads((log_directory / "run.json").read_text(encoding="utf-8"))
        if metadata["capture_complete"] is not False:
            raise AssertionError("capture must remain incomplete while the recorder runs")
        bag = log_directory / "raw_bag"
        bag.mkdir()
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
        return 0

    def test_missing_numeric_dependencies_reports_install_command_without_traceback(self):
        result = subprocess.run(
            [sys.executable, "-B", "-S", str(Path(launcher.__file__)), "--help"],
            capture_output=True, text=True, check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("python3-numpy", result.stderr)
        self.assertIn("python3-scipy", result.stderr)
        self.assertNotIn("Traceback", result.stderr)

    def test_default_output_paths_are_unique_and_preserve_existing_map(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            existing = root / "field_map.pcd"
            existing.write_bytes(b"previous map")
            first = launcher.output_path(root, None)
            second = launcher.output_path(root, None)
            self.assertNotEqual(first.parent, second.parent)
            self.assertEqual(first.parent.parent, root / "maps")
            self.assertTrue(first.parent.name.startswith("run_"))
            self.assertTrue(first.parent.is_dir())
            self.assertEqual(existing.read_bytes(), b"previous map")
            self.assertFalse(first.exists())

    def test_explicit_output_does_not_create_run_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            selected = launcher.output_path(root, "custom/map.pcd")
            self.assertEqual(selected, (root / "custom/map.pcd").resolve())
            self.assertFalse((root / "maps").exists())

    def test_default_run_metadata_is_beside_pcd_and_has_only_selected_environment(self):
        with tempfile.TemporaryDirectory() as temporary:
            pcd = launcher.output_path(Path(temporary), None)
            selected_environment = {"ROS_DISTRO": "humble", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                                    "ROS_DOMAIN_ID": "7", "SDK_CLIENT_IP": "192.168.1.10"}
            with mock.patch.dict(launcher.os.environ, {**selected_environment, "PRIVATE_TOKEN": "do-not-record"}, clear=True):
                directory = launcher.prepare_logs(pcd, False, ["python", "mapper.py"], ["rviz2", "-f", "odom"])
            self.assertEqual(directory, pcd.parent)
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["environment"], selected_environment)
            self.assertEqual(metadata["pcd_path"], str(pcd))
            self.assertEqual(metadata["mapper_command"], ["python", "mapper.py"])
            self.assertEqual(metadata["rviz_command"], ["rviz2", "-f", "odom"])
            self.assertTrue(metadata["started_at"].endswith("+00:00"))
            self.assertNotIn("PRIVATE_TOKEN", (directory / "run.json").read_text(encoding="utf-8"))
            with self.assertRaises(FileExistsError):
                launcher.prepare_logs(pcd, False, ["another mapper"], ["rviz2"])

    def test_explicit_pcd_gets_unique_log_directory_without_overwriting_logs(self):
        with tempfile.TemporaryDirectory() as temporary:
            pcd = Path(temporary) / "custom/map.pcd"
            first = launcher.prepare_logs(pcd, True, ["mapper"], ["rviz"])
            existing_log = first / "mapper.log"
            existing_log.write_text("previous output", encoding="utf-8")
            second = launcher.prepare_logs(pcd, True, ["mapper"], ["rviz"])
            self.assertNotEqual(first, second)
            self.assertEqual(first.parent, pcd.parent)
            self.assertTrue(first.name.startswith("logs_run_"))
            self.assertEqual(existing_log.read_text(encoding="utf-8"), "previous output")

    def test_run_manifest_records_runtime_and_only_performance_environment_allowlist(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "mapping").mkdir()
            node = root / "mapping/field_mapper_node.py"
            node.write_bytes(b"# measured runtime\n")
            pcd = launcher.output_path(root, None)
            performance = {"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "2", "MKL_NUM_THREADS": "3"}
            with mock.patch.dict(launcher.os.environ, {**performance, "PRIVATE_TOKEN": "secret"}, clear=True):
                directory = launcher.prepare_logs(pcd, False, ["python", str(node)], ["rviz2"], root)
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["performance_environment"], performance)
            self.assertEqual(metadata["python"], sys.version)
            self.assertEqual(metadata["files_sha256"]["mapping/field_mapper_node.py"],
                             hashlib.sha256(node.read_bytes()).hexdigest())
            self.assertIsNone(metadata["files_sha256"]["mapping/cloud_io.py"])
            self.assertNotIn("PRIVATE_TOKEN", json.dumps(metadata))

    def test_mapper_arguments_pass_through_and_rviz_frame_matches(self):
        arguments = ["--topic", "/custom/cloud", "--global-frame", "world", "--voxel", "0.1",
                     "--sensor-z", "0.4", "--pcd-file", "old.pcd"]
        args = launcher.build_parser().parse_args(arguments)
        root = Path("test-project").resolve()
        selected = root / "new.pcd"
        mapper, rviz = launcher.child_commands(root, "rviz2", arguments, selected, args.global_frame,
                                              mode="online")
        self.assertEqual(mapper[3:3 + len(arguments)], arguments)
        self.assertEqual(mapper[3 + len(arguments):], [
            "--live-display", "none", "--save-policy", "final",
            "--pcd-file", str(selected),
        ])
        # The official matrix.rviz is fixed to the lidar frame and only shows
        # the live front-lidar scan, so the launcher's global-frame argument
        # must not override it.
        self.assertEqual(rviz[-2:], ["-f", "lidar"])
        self.assertEqual(rviz[2], str(root / "rviz/matrix.rviz"))
        self.assertNotIn(str(root / "rviz/matrix_mapping.rviz"), rviz)

    def test_default_capture_records_selected_topics_without_running_online_fusion(self):
        arguments = ["--topic=/custom/cloud", "--odom-topic", "/custom/pose", "--global-frame", "world"]
        root = Path("test-project").resolve()
        pcd = root / "maps/run_test/field_map.pcd"
        mapper, rviz = launcher.child_commands(root, "rviz2", arguments, pcd, "world")
        self.assertEqual(mapper, [
            sys.executable, "-u", str(root / "mapping/capture_mapping.py"),
            "--bag-dir", str(pcd.parent / "raw_bag"),
            "--topic", "/custom/cloud", "--odom-topic", "/custom/pose",
        ])
        self.assertEqual(rviz, ["rviz2", "-d", str(root / "rviz/matrix.rviz"), "-f", "lidar"])
        self.assertEqual(launcher.build_parser().parse_args([]).mode, "capture")

    def test_online_mode_and_explicit_display_save_options_preserve_both_argument_forms(self):
        root = Path("test-project").resolve()
        selected = root / "field_map.pcd"
        for options in (["--mode", "online", "--live-display", "scan", "--save-policy", "periodic"],
                        ["--mode=online", "--live-display=scan", "--save-policy=periodic"]):
            with self.subTest(options=options):
                mapper, _ = launcher.child_commands(root, "rviz2", options, selected, "world", mode="online")
                parsed = launcher.field_mapper_node.build_parser().parse_args(mapper[3:])
                self.assertEqual(parsed.live_display, "scan")
                self.assertEqual(parsed.save_policy, "periodic")
                self.assertEqual(parsed.pcd_file, str(selected))
                self.assertFalse(any(argument.startswith("--mode") for argument in mapper))

    def test_capture_preflight_only_requires_recording_cli_and_official_view(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher.field_mapper_node, "ROS_IMPORT_ERROR", ImportError("mapper ROS absent")), \
                mock.patch.object(launcher.shutil, "which", side_effect=lambda name: "/usr/bin/" + name):
            root = Path(temporary)
            (root / "mapping").mkdir()
            (root / "rviz").mkdir()
            (root / "mapping/capture_mapping.py").write_text("# recorder\n", encoding="utf-8")
            (root / "rviz/matrix.rviz").write_text("# official view\n", encoding="utf-8")
            self.assertEqual(launcher.preflight(root, "capture"), "/usr/bin/rviz2")
            with self.assertRaisesRegex(RuntimeError, "ROS2 Python dependencies"):
                launcher.preflight(root, "online")

    def test_capture_preflight_reports_missing_ros2_cli_before_allocating_run(self):
        with mock.patch.object(launcher.shutil, "which", side_effect=lambda name: "rviz2" if name == "rviz2" else None):
            with self.assertRaisesRegex(RuntimeError, "install rosbag2"):
                launcher.preflight(Path("missing-project"), "capture")

    def test_mapping_view_exposes_ground_with_latched_qos_and_world_coordinates(self):
        config = (launcher.PROJECT_ROOT / "rviz/matrix_mapping.rviz").read_text(encoding="utf-8")
        displays_text = config.split("  Displays:\n", 1)[1].split("\n  Enabled:", 1)[0]
        displays = re.split(r"(?m)(?=^    - )", displays_text)
        ground = [display for display in displays if "        Value: /field_map/ground\n" in display]
        self.assertEqual(len(ground), 1, "The mapper's ground topic must have exactly one display")
        for property_line in ("Class: rviz_default_plugins/PointCloud2", "Enabled: true",
                              "Color Transformer: FlatColor", "Position Transformer: XYZ",
                              "Color: 255; 235; 80", "Style: Flat Squares", "Size (m): 0.12",
                              "Use Fixed Frame: true", "Value: true"):
            self.assertIn("      " + property_line + "\n", ground[0])
        for property_line in ("Depth: 1", "Durability Policy: Transient Local",
                              "History Policy: Keep Last", "Reliability Policy: Reliable"):
            self.assertIn("        " + property_line + "\n", ground[0])
        self.assertIn("    Fixed Frame: world\n", config)
        self.assertNotIn("      Reference Frame:", ground[0])

    def test_mapping_view_shows_recent_accepted_scans_without_latched_backlog(self):
        config = (launcher.PROJECT_ROOT / "rviz/matrix_mapping.rviz").read_text(encoding="utf-8")
        displays_text = config.split("  Displays:\n", 1)[1].split("\n  Enabled:", 1)[0]
        displays = re.split(r"(?m)(?=^    - )", displays_text)
        scans = [display for display in displays if "        Value: /field_map/current_scan\n" in display]
        self.assertEqual(len(scans), 1)
        for property_line in ("Class: rviz_default_plugins/PointCloud2", "Enabled: true",
                              "Position Transformer: XYZ", "Color Transformer: FlatColor",
                              "Decay Time: 0.5", "Style: Points", "Size (Pixels): 2",
                              "Use Fixed Frame: true", "Value: true"):
            self.assertIn("      " + property_line + "\n", scans[0])
        for property_line in ("Depth: 1", "Durability Policy: Volatile",
                              "History Policy: Keep Last", "Reliability Policy: Best Effort"):
            self.assertIn("        " + property_line + "\n", scans[0])

    def test_mapping_view_has_no_image_plugins_or_saved_image_windows(self):
        config = (launcher.PROJECT_ROOT / "rviz/matrix_mapping.rviz").read_text(encoding="utf-8")
        self.assertNotIn("rviz_default_plugins/Image", config)
        self.assertNotIn("/Image1", config)
        self.assertNotIn("/Image2", config)
        self.assertNotIn("\n  Image:", config)
        self.assertNotIn("QMainWindow State:", config)
        for topic in ("/front_camera/image/compressed", "/front_depth/image/compressed"):
            self.assertNotIn(topic, config)

    def test_ground_view_preserves_global_map_classification_and_official_config(self):
        root = launcher.PROJECT_ROOT
        config = (root / "rviz/matrix_mapping.rviz").read_text(encoding="utf-8")
        for topic in ("/field_map", "/field_map/passable", "/field_map/structure"):
            self.assertIn("        Value: " + topic + "\n", config)
        official = (root / "rviz/matrix.rviz").read_text(encoding="utf-8")
        self.assertIn("    Fixed Frame: lidar\n", official)
        self.assertIn("        Value: /front_lidar\n", official)
        self.assertNotIn("/field_map", official)

    def test_preflight_failure_does_not_allocate_output_or_start_children(self):
        with mock.patch.object(launcher, "preflight", side_effect=RuntimeError("missing ROS")), \
                mock.patch.object(launcher, "output_path") as allocate, \
                mock.patch.object(launcher, "supervise") as run, \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            self.assertEqual(launcher.main([]), 1)
            allocate.assert_not_called()
            run.assert_not_called()

    def test_main_passes_default_log_directory_to_supervisor(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher, "PROJECT_ROOT", Path(temporary)), \
                mock.patch.object(launcher, "preflight", return_value="rviz2"), \
                mock.patch.object(launcher, "supervise", side_effect=self.finish_capture) as run, \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            root = Path(temporary)
            (root / "rviz").mkdir()
            original = root / "rviz/matrix.rviz"
            original.write_bytes(b"# runtime config\n")
            self.assertEqual(launcher.main([]), 0)
            directory = run.call_args.kwargs["log_directory"]
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            pcd = Path(metadata["pcd_path"])
            self.assertEqual(directory, pcd.parent)
            self.assertTrue((directory / "run.json").is_file())
            snapshot = directory / "matrix.rviz"
            self.assertEqual(snapshot.read_bytes(), original.read_bytes())
            self.assertEqual(run.call_args.args[1][2], str(snapshot))
            self.assertEqual(metadata["rviz_command"], run.call_args.args[1])
            self.assertEqual(metadata["rviz_config"], str(snapshot))
            self.assertEqual(metadata["rviz_config_sha256"], hashlib.sha256(original.read_bytes()).hexdigest())
            self.assertEqual(metadata["mode"], "capture")
            self.assertEqual(metadata["bag_path"], str(directory / "raw_bag"))
            self.assertEqual(metadata["finalize_command"], ["bash", "mapping/finalize_pcd.sh", str(directory)])
            self.assertEqual(metadata["mapping_args"]["pcd_file"], str(pcd))
            self.assertEqual(metadata["mapping_args"]["live_display"], "none")
            self.assertEqual(metadata["mapping_args"]["save_policy"], "final")
            self.assertTrue(metadata["capture_complete"])
            self.assertEqual(metadata["capture_exit_code"], 0)
            self.assertEqual(run.call_args.kwargs["save_timeout"], 60.0)
            snapshot.write_bytes(b"# user saves the window layout\n")
            self.assertEqual(original.read_bytes(), b"# runtime config\n")

    def test_main_snapshots_rviz_beside_explicit_run_logs(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher, "PROJECT_ROOT", Path(temporary)), \
                mock.patch.object(launcher, "preflight", return_value="rviz2"), \
                mock.patch.object(launcher, "supervise", side_effect=self.finish_capture) as run, \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            root = Path(temporary)
            (root / "rviz").mkdir()
            (root / "rviz/matrix.rviz").write_text("# official config\n", encoding="utf-8")
            self.assertEqual(launcher.main(["--pcd-file", "custom.pcd", "--global-frame", "odom"]), 0)
            directory = run.call_args.kwargs["log_directory"]
            self.assertTrue(directory.name.startswith("logs_run_"))
            self.assertEqual(run.call_args.args[1], ["rviz2", "-d", str(directory / "matrix.rviz"), "-f", "lidar"])
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["bag_path"], str(directory / "raw_bag"))
            self.assertIn(str(directory / "raw_bag"), run.call_args.args[0])
            self.assertEqual(metadata["mapping_args"]["global_frame"], "odom")

    def test_main_online_records_effective_options_without_capture_finalizer(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher, "PROJECT_ROOT", Path(temporary)), \
                mock.patch.object(launcher, "preflight", return_value="rviz2") as preflight, \
                mock.patch.object(launcher, "supervise", return_value=0) as run, \
                mock.patch("sys.stdout", new_callable=io.StringIO) as terminal:
            root = Path(temporary)
            (root / "rviz").mkdir()
            (root / "rviz/matrix.rviz").write_text("# official config\n", encoding="utf-8")
            self.assertEqual(launcher.main(["--mode=online", "--global-frame", "odom"]), 0)
            preflight.assert_called_once_with(root, "online")
            directory = run.call_args.kwargs["log_directory"]
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["mode"], "online")
            self.assertIsNone(metadata["bag_path"])
            self.assertIsNone(metadata["finalize_command"])
            self.assertEqual(metadata["mapping_args"]["live_display"], "none")
            self.assertIn("RViz fixed frame: lidar", terminal.getvalue())
            self.assertNotIn("RViz fixed frame: odom", terminal.getvalue())

    def test_main_cannot_report_success_without_flushed_bag_metadata(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher, "PROJECT_ROOT", Path(temporary)), \
                mock.patch.object(launcher, "preflight", return_value="rviz2"), \
                mock.patch.object(launcher, "supervise", return_value=0) as run, \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO) as errors:
            root = Path(temporary)
            (root / "rviz").mkdir()
            (root / "rviz/matrix.rviz").write_text("# official config\n", encoding="utf-8")
            self.assertEqual(launcher.main([]), 1)
            directory = run.call_args.kwargs["log_directory"]
            metadata = json.loads((directory / "run.json").read_text(encoding="utf-8"))
            self.assertFalse(metadata["capture_complete"])
            self.assertEqual(metadata["capture_exit_code"], 1)
            self.assertIn("metadata.yaml is missing", errors.getvalue())

    def test_snapshot_failure_prevents_both_children_from_starting(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher, "PROJECT_ROOT", Path(temporary)), \
                mock.patch.object(launcher, "preflight", return_value="rviz2"), \
                mock.patch.object(launcher, "supervise") as run, \
                mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO) as errors:
            self.assertEqual(launcher.main([]), 1)
            run.assert_not_called()
            self.assertIn("Required file is missing", errors.getvalue())

    def run_with_children(self, children, sleep_effect=None):
        with ExitStack() as stack:
            start = stack.enter_context(mock.patch.object(launcher.subprocess, "Popen", side_effect=children))
            stack.enter_context(mock.patch.object(launcher.signal, "signal", return_value=signal.SIG_DFL))
            stack.enter_context(mock.patch.object(launcher.time, "sleep", side_effect=sleep_effect))
            stack.enter_context(mock.patch("sys.stderr", new_callable=io.StringIO))
            stack.enter_context(mock.patch("sys.stdout", new_callable=io.StringIO))
            result = launcher.supervise(["mapper"], ["rviz"], Path.cwd())
        return result, start

    def test_rviz_start_failure_interrupts_and_waits_for_mapper(self):
        mapper = mock.Mock()
        mapper.poll.return_value = None
        result, _ = self.run_with_children([mapper, OSError("RViz failed")])
        self.assertEqual(result, 1)
        mapper.send_signal.assert_called_once_with(signal.SIGINT)
        mapper.wait.assert_called_once_with(timeout=15.0)
        mapper.terminate.assert_not_called()

    def test_rviz_normal_close_saves_mapper(self):
        mapper = mock.Mock()
        mapper.poll.return_value = None
        rviz = mock.Mock()
        rviz.poll.return_value = 0
        result, start = self.run_with_children([mapper, rviz])
        self.assertEqual(result, 0)
        mapper.send_signal.assert_called_once_with(signal.SIGINT)
        mapper.wait.assert_called_once_with(timeout=15.0)
        rviz.send_signal.assert_not_called()
        self.assertNotIn("env", start.call_args.kwargs)

    def test_rviz_normal_close_does_not_hide_worker_flush_failure(self):
        mapper = mock.Mock()
        mapper.poll.return_value = None
        mapper.wait.return_value = 5
        rviz = mock.Mock()
        rviz.poll.return_value = 0
        result, _ = self.run_with_children([mapper, rviz])
        self.assertEqual(result, 5)

    def test_rviz_normal_close_does_not_hide_forced_worker_termination(self):
        mapper = mock.Mock()
        mapper.poll.return_value = None
        mapper.wait.side_effect = [subprocess.TimeoutExpired("recorder", 15), 0]
        rviz = mock.Mock()
        rviz.poll.return_value = 0
        result, _ = self.run_with_children([mapper, rviz])
        self.assertEqual(result, 1)
        mapper.terminate.assert_called_once()

    def test_zero_exit_during_worker_startup_is_still_a_launch_failure(self):
        mapper = mock.Mock()
        mapper.poll.return_value = 0
        result, start = self.run_with_children([mapper])
        self.assertEqual(result, 1)
        self.assertEqual(start.call_count, 1)

    def test_logging_tees_both_child_outputs_to_files_and_original_terminal(self):
        mapper = mock.Mock()
        mapper.poll.return_value = None
        mapper.stdout = io.StringIO("mapper startup\nmapper saved\n")
        rviz = mock.Mock()
        rviz.poll.return_value = 0
        rviz.stdout = io.BytesIO("RViz\n中文输出\n".encode("utf-8"))
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher.subprocess, "Popen", side_effect=[mapper, rviz]) as start, \
                mock.patch.object(launcher.signal, "signal", return_value=signal.SIG_DFL), \
                mock.patch("sys.stdout", new_callable=io.StringIO) as terminal:
            directory = Path(temporary)
            self.assertEqual(launcher.supervise(["mapper"], ["rviz"], directory, directory), 0)
            self.assertEqual((directory / "mapper.log").read_text(encoding="utf-8"), "mapper startup\nmapper saved\n")
            self.assertEqual((directory / "rviz.log").read_text(encoding="utf-8"), "RViz\n中文输出\n")
            self.assertIn("mapper saved", terminal.getvalue())
            self.assertIn("中文输出", terminal.getvalue())
            self.assertEqual(start.call_args.kwargs["stdout"], subprocess.PIPE)
            self.assertEqual(start.call_args.kwargs["stderr"], subprocess.STDOUT)
            self.assertEqual(start.call_args.kwargs["start_new_session"], launcher.os.name == "posix")
        self.assertTrue(mapper.stdout.closed)
        self.assertTrue(rviz.stdout.closed)

    def test_existing_log_is_never_overwritten_or_child_started(self):
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(launcher.subprocess, "Popen") as start, \
                mock.patch.object(launcher.signal, "signal", return_value=signal.SIG_DFL), \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            directory = Path(temporary)
            (directory / "mapper.log").write_text("keep this", encoding="utf-8")
            self.assertEqual(launcher.supervise(["mapper"], ["rviz"], directory, directory), 1)
            start.assert_not_called()
            self.assertEqual((directory / "mapper.log").read_text(encoding="utf-8"), "keep this")

    def test_log_shutdown_does_not_wait_for_pipe_held_by_grandchild(self):
        release = threading.Event()
        reading = threading.Event()

        class HeldPipe:
            def read(self, _size):
                reading.set()
                release.wait(timeout=5.0)
                return b""

            def close(self):
                pass

        log = io.StringIO()
        capture = launcher.OutputTee(HeldPipe(), log, io.StringIO(), "mapper")
        try:
            self.assertTrue(reading.wait(timeout=1.0))
            start = time.monotonic()
            with mock.patch("sys.stderr", new_callable=io.StringIO):
                capture.finish(timeout=0.02)
            self.assertLess(time.monotonic() - start, 0.5)
            self.assertTrue(log.closed)
            self.assertTrue(capture.thread.daemon)
        finally:
            release.set()
            capture.thread.join(timeout=1.0)
        self.assertFalse(capture.thread.is_alive())

    def test_log_shutdown_does_not_wait_for_blocked_console(self):
        release = threading.Event()
        writing = threading.Event()

        class BlockedConsole:
            def write(self, _text):
                writing.set()
                release.wait(timeout=5.0)

            def flush(self):
                pass

        log = io.StringIO()
        capture = launcher.OutputTee(io.BytesIO(b"saved map\n"), log, BlockedConsole(), "mapper")
        try:
            self.assertTrue(writing.wait(timeout=1.0))
            self.assertEqual(log.getvalue(), "saved map\n")
            start = time.monotonic()
            with mock.patch("sys.stderr", new_callable=io.StringIO):
                capture.finish(timeout=0.02)
            self.assertLess(time.monotonic() - start, 0.5)
            self.assertTrue(log.closed)
        finally:
            release.set()
            capture.thread.join(timeout=1.0)
        self.assertFalse(capture.thread.is_alive())

    def test_log_shutdown_does_not_wait_for_blocked_disk_write(self):
        release = threading.Event()
        writing = threading.Event()

        class BlockedLog(io.StringIO):
            def write(self, text):
                writing.set()
                release.wait(timeout=5.0)
                return super().write(text)

        log = BlockedLog()
        capture = launcher.OutputTee(io.BytesIO(b"saved map\n"), log, io.StringIO(), "mapper")
        try:
            self.assertTrue(writing.wait(timeout=1.0))
            start = time.monotonic()
            with mock.patch("sys.stderr", new_callable=io.StringIO):
                capture.finish(timeout=0.02)
            self.assertLess(time.monotonic() - start, 0.5)
            self.assertFalse(log.closed)
        finally:
            release.set()
            capture.thread.join(timeout=1.0)
        self.assertTrue(log.closed)
        self.assertFalse(capture.thread.is_alive())

    def test_real_child_pipe_drains_beyond_pipe_capacity_and_preserves_final_output(self):
        script = "import sys; sys.stdout.buffer.write(('雷达数据\\n'*10000+'final PCD saved\\n').encode('utf-8')); sys.stdout.flush()"
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "mapper.log"
            child = subprocess.Popen([sys.executable, "-u", "-c", script],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            capture = launcher.OutputTee(child.stdout, path.open("w", encoding="utf-8"), io.StringIO(), "mapper")
            try:
                self.assertEqual(child.wait(timeout=5.0), 0)
                capture.finish(timeout=2.0)
                content = path.read_text(encoding="utf-8")
                self.assertEqual(content.count("雷达数据\n"), 10000)
                self.assertTrue(content.endswith("final PCD saved\n"))
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5.0)
                capture.finish(timeout=1.0)

    def test_mapper_startup_exit_prevents_rviz_start(self):
        mapper = mock.Mock()
        mapper.poll.return_value = 2
        result, start = self.run_with_children([mapper])
        self.assertEqual(result, 2)
        self.assertEqual(start.call_count, 1)

    def test_mapper_later_failure_stops_rviz(self):
        mapper = mock.Mock()
        mapper.poll.side_effect = [None, 7, 7]
        rviz = mock.Mock()
        rviz.poll.return_value = None
        result, _ = self.run_with_children([mapper, rviz])
        self.assertEqual(result, 7)
        rviz.send_signal.assert_called_once_with(signal.SIGINT)
        mapper.send_signal.assert_not_called()

    def test_ctrl_c_saves_mapper_then_stops_rviz_and_restores_handlers(self):
        mapper = mock.Mock()
        rviz = mock.Mock()
        mapper.poll.return_value = rviz.poll.return_value = None
        handlers = {}
        events = []

        def install_handler(sig, handler):
            handlers[sig] = handler
            return signal.SIG_DFL

        def interrupt(_duration):
            handlers[signal.SIGINT](signal.SIGINT, None)

        mapper.send_signal.side_effect = lambda _sig: events.append("mapper")
        rviz.send_signal.side_effect = lambda _sig: events.append("rviz")
        with mock.patch.object(launcher.subprocess, "Popen", side_effect=[mapper, rviz]), \
                mock.patch.object(launcher.signal, "signal", side_effect=install_handler), \
                mock.patch.object(launcher.time, "sleep", side_effect=interrupt), \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            result = launcher.supervise(["mapper"], ["rviz"], Path.cwd())
        self.assertEqual(result, 130)
        self.assertEqual(events, ["mapper", "rviz"])
        self.assertEqual(handlers[signal.SIGINT], signal.SIG_DFL)
        self.assertEqual(handlers[signal.SIGTERM], signal.SIG_DFL)

    def test_unresponsive_child_escalates_only_after_save_timeout(self):
        child = mock.Mock()
        child.poll.return_value = None
        child.wait.side_effect = [subprocess.TimeoutExpired("mapper", 15),
                                  subprocess.TimeoutExpired("mapper", 3), 0]
        with mock.patch("sys.stdout", new_callable=io.StringIO), \
                mock.patch("sys.stderr", new_callable=io.StringIO):
            launcher.stop_child(child, "mapper")
        self.assertEqual(child.mock_calls, [
            mock.call.poll(), mock.call.send_signal(signal.SIGINT), mock.call.wait(timeout=15.0),
            mock.call.terminate(), mock.call.wait(timeout=3.0), mock.call.kill(), mock.call.wait(),
        ])


if __name__ == "__main__":
    unittest.main()
