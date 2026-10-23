"""Host-side architecture contracts and real pixels; no ARM binary is executed."""
from contextlib import ExitStack, contextmanager
import copy
import io
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from task3_sim import hardware, sdk_platform
from task3_sim.official_sdk import OfficialSDKError, load_official_sdk_module
from task3_sim.package_update import make_package
from task3_sim.perception import create_managed_detector


@contextmanager
def linux310(arch="aarch64"):
    with ExitStack() as stack:
        stack.enter_context(patch.object(sys, "platform", "linux"))
        stack.enter_context(patch.object(sys, "version_info", (3, 10, 12)))
        stack.enter_context(patch("platform.machine", return_value=arch))
        stack.enter_context(patch("platform.python_implementation", return_value="CPython"))
        stack.enter_context(patch("task3_sim.sdk_platform.struct.calcsize", return_value=8))
        yield


def elf_fixture(root, arch):
    files = sdk_platform.sdk_files(root, arch)
    header = b"\x7fELF\x02\x01" + b"\0" * 12 + struct.pack("<H", sdk_platform.ELF_MACHINES[arch])
    for path in files.values():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(header)
    return files


def profile():
    return json.loads((hardware.PACKAGE/"config/hardware_rk3588.json").read_text(encoding="utf-8"))


class SDKArchitectureTests(unittest.TestCase):
    def test_aliases_and_32bit_rejection(self):
        for value, expected in (("ARM64", "aarch64"), ("aarch64", "aarch64"),
                                ("AMD64", "x86_64"), ("x86_64", "x86_64")):
            self.assertEqual(sdk_platform.normalize_arch(value), expected)
        for value in ("armv7l", "i686", "riscv64"):
            with self.assertRaises(ValueError):
                sdk_platform.normalize_arch(value)

    def test_real_elf_layout_selects_only_host_arch(self):
        with tempfile.TemporaryDirectory() as folder:
            for arch in ("aarch64", "x86_64"):
                files = elf_fixture(folder, arch)
                with linux310(arch):
                    report = sdk_platform.inspect_sdk(folder)
                self.assertTrue(report["ready"], report["errors"])
                self.assertEqual(report["files"]["binding"], str(files["binding"]))
                self.assertFalse(report["sdk_initialized"])

    def test_wrong_elf_machine_rejected_before_native_load(self):
        with tempfile.TemporaryDirectory() as folder:
            files = elf_fixture(folder, "aarch64")
            x86 = elf_fixture(folder, "x86_64")
            files["binding"].write_bytes(x86["binding"].read_bytes())
            with linux310(), patch("ctypes.CDLL") as native:
                with self.assertRaisesRegex(OfficialSDKError, "aarch64 ELF"):
                    load_official_sdk_module(folder)
            native.assert_not_called()

    def test_missing_arm_files_do_not_fall_back_to_x86(self):
        with tempfile.TemporaryDirectory() as folder:
            elf_fixture(folder, "x86_64")
            with linux310():
                report = sdk_platform.inspect_sdk(folder)
            self.assertFalse(report["ready"])
            self.assertEqual(report["sha256"], {})
            self.assertIn("aarch64", report["files"]["binding"])

    def test_incorrect_python_abi_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder, linux310():
            elf_fixture(folder, "aarch64")
            with patch.object(sys, "version_info", (3, 11, 9)):
                report = sdk_platform.inspect_sdk(folder)
            self.assertFalse(report["ready"])
            self.assertTrue(any("CPython 3.10" in error for error in report["errors"]))

    def test_binding_import_never_constructs_or_connects_client(self):
        for arch in ("aarch64", "x86_64"):
            with self.subTest(arch=arch), tempfile.TemporaryDirectory() as folder:
                files = elf_fixture(folder, arch)
                client = Mock()
                module = SimpleNamespace(__file__=str(files["binding"]), HighLevel=client)
                old_path = list(sys.path)
                with linux310(arch), patch("ctypes.CDLL") as native, \
                        patch("task3_sim.official_sdk.importlib.import_module", return_value=module), \
                        patch.dict(sys.modules, {"mc_sdk_zsl_1_py": module}):
                    self.assertIs(load_official_sdk_module(folder), module)
                native.assert_called_once()
                self.assertEqual(native.call_args.args[0], str(files["library"]))
                client.assert_not_called()
                self.assertEqual(sys.path, old_path)

    def test_preloaded_module_from_other_sdk_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            elf_fixture(folder, "aarch64")
            module = SimpleNamespace(__file__=str(Path(folder)/"other.so"))
            with linux310(), patch("ctypes.CDLL") as native, \
                    patch.dict(sys.modules, {"mc_sdk_zsl_1_py": module}):
                with self.assertRaisesRegex(OfficialSDKError, "already loaded"):
                    load_official_sdk_module(folder)
            native.assert_not_called()

    def test_hardware_import_does_not_require_simulator_or_sdk(self):
        script = """
import sys
class NoSimulation:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco', 'rclpy', 'mc_sdk_zsl_1_py', 'numpy', 'cv2'}:
            raise RuntimeError('unexpected import: '+fullname)
sys.meta_path.insert(0, NoSimulation())
import task3_sim.hardware
print('hardware entry imports stdlib only')
"""
        result = subprocess.run([sys.executable, "-B", "-c", script],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_preflight_ldd_uses_selected_sdk_directory_without_global_env_change(self):
        with tempfile.TemporaryDirectory() as folder:
            files = elf_fixture(folder, "aarch64")
            with linux310():
                report = sdk_platform.inspect_sdk(folder)
            env_before = dict(os.environ)
            with patch.object(hardware, "inspect_sdk", return_value=report), \
                    patch.object(hardware.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout="ok", stderr="")) as ldd, \
                    patch("task3_sim.official_sdk.load_official_sdk_module") as load:
                checked = hardware.preflight(folder)
            self.assertTrue(checked["ready"], checked["errors"])
            self.assertTrue(checked["apriltag_native_smoke"])
            load.assert_not_called()
            self.assertEqual(ldd.call_count, 2)
            self.assertTrue(ldd.call_args.kwargs["env"]["LD_LIBRARY_PATH"].startswith(str(files["library"].parent)))
            self.assertEqual(dict(os.environ), env_before)
            self.assertFalse(checked["sdk_initialized"])
            self.assertFalse(checked["autonomous_hardware_ready"])

    def test_unresolved_library_prevents_binding_import(self):
        with tempfile.TemporaryDirectory() as folder:
            elf_fixture(folder, "aarch64")
            with linux310():
                report = sdk_platform.inspect_sdk(folder)
            with patch.object(hardware, "inspect_sdk", return_value=report), \
                    patch.object(hardware.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout="libfoo => not found", stderr="")), \
                    patch("task3_sim.official_sdk.load_official_sdk_module") as load:
                checked = hardware.preflight(folder, check_import=True)
            self.assertFalse(checked["ready"])
            load.assert_not_called()


class RealVideoTests(unittest.TestCase):
    def args(self, output):
        return SimpleNamespace(rtsp="rtsp://camera.invalid:8554/test", config=hardware.PACKAGE/"config/hardware_rk3588.json",
                               show=False, output=output, seconds=10)

    def test_profile_validation_rejects_wrong_types_and_sim_config(self):
        hardware.validate_observer_config(profile())
        cases = [[], {}, {**profile(), "target_ids": [[2]]}, {**profile(), "target_ids": [True]},
                 {**profile(), "target_ids": [2, 2]}, {**profile(), "tag": None},
                 {**profile(), "video": None}, {**profile(), "backend": "official"}]
        for config in cases:
            with self.subTest(config=config), self.assertRaises(ValueError):
                hardware.validate_observer_config(config)

    def test_real_pixels_decode_multiple_ids_and_mark_interference(self):
        image = np.full((360, 640), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image[120:240, 80:200] = cv2.aruco.generateImageMarker(dictionary, 2, 120)
        image[120:240, 400:520] = cv2.aruco.generateImageMarker(dictionary, 12, 120)
        detector = create_managed_detector(profile()["tag"])
        try:
            records = hardware.identity_records(detector.detect(image, estimate_tag_pose=False), profile())
        finally:
            detector.close()
        self.assertEqual({row["tag_id"] for row in records}, {2, 12})
        self.assertEqual({row["tag_id"] for row in records if row["declared_target"]}, {2})
        self.assertTrue(all("t_camera_from_tag" not in row for row in records))

    def test_quality_filter_drops_nonfinite_and_unreliable_corners(self):
        row = SimpleNamespace(decision_margin=100, hamming=0, tag_id=2,
                              corners=[[0, 0], [1, 0], [1, 1], [0, 1]])
        bad_margin, bad_corner, bad_hamming = (copy.deepcopy(row) for _ in range(3))
        bad_margin.decision_margin = float("nan")
        bad_corner.corners[0][0] = float("nan")
        bad_hamming.hamming = 1
        records = hardware.identity_records([bad_margin, bad_corner, bad_hamming, row], profile())
        self.assertEqual(len(records), 1)

    def run_capture(self, output, reads, times):
        cap = Mock()
        cap.isOpened.return_value = True
        cap.read.side_effect = reads
        detector = Mock()
        detector.detect.return_value = []
        with patch.object(cv2, "VideoCapture", return_value=cap) as capture, \
                patch("task3_sim.perception.create_managed_detector", return_value=detector), \
                patch.object(hardware.time, "monotonic", side_effect=times), \
                patch("task3_sim.official_sdk.load_official_sdk") as sdk, \
                patch("sys.stdout", new_callable=io.StringIO), patch("sys.stderr", new_callable=io.StringIO):
            result = hardware.observe_video(self.args(output))
        sdk.assert_not_called()
        cap.release.assert_called_once()
        detector.close.assert_called_once()
        capture.assert_called_once()
        detector.detect.assert_called_with(unittest.mock.ANY, estimate_tag_pose=False)
        return result, json.loads((output/"summary.json").read_text(encoding="utf-8"))

    def test_success_logs_decode_time_without_fabricating_capture_time_or_pose(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/"observation"
            result, summary = self.run_capture(output, [(True, np.zeros((64, 64, 3), np.uint8))], [0, 0, 1, 11, 12])
            self.assertEqual(result, 0)
            self.assertEqual(summary["status"], "OBSERVATION_COMPLETED")
            self.assertFalse(summary["sdk_initialized"])
            self.assertFalse(summary["pose_estimated"])
            self.assertFalse(summary["shooting_enabled"])
            row = json.loads((output/"detections.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(row["decoded_at_monotonic_s"], 1)
            self.assertIsNone(row["source_acquisition_timestamp_s"])
            self.assertFalse(row["usable_for_motion_or_fire"])
            self.assertTrue((output/"first_frame.jpg").is_file())

    def test_disconnect_stops_and_closes_native_resources(self):
        with tempfile.TemporaryDirectory() as folder:
            result, summary = self.run_capture(Path(folder)/"out",
                [(True, np.zeros((64, 64, 3), np.uint8)), (False, None)], [0, 0, 1, 2, 3, 4])
            self.assertEqual(result, 2)
            self.assertEqual(summary["status"], "ERROR")
            self.assertEqual(summary["frames"], 1)
            self.assertIn("read failed", summary["error"])

    def test_refuses_existing_data_and_non_rtsp_sources(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output/"keep.txt").write_text("previous results", encoding="utf-8")
            args = self.args(output)
            with self.assertRaisesRegex(ValueError, "nonempty"), patch.object(cv2, "VideoCapture") as cap:
                hardware.observe_video(args)
            cap.assert_not_called()
            args.rtsp = "synthetic://frames"
            with self.assertRaisesRegex(ValueError, "actual camera"):
                hardware.observe_video(args)

    def test_package_excludes_hardware_venv_and_local_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/"task3_sim"
            root.mkdir()
            (root/"hardware.py").write_text("# code", encoding="utf-8")
            for name in (".venv_hardware", "local", "runs"):
                (root/name).mkdir()
                (root/name/"private.json").write_text("{}", encoding="utf-8")
            report = make_package(root, Path(folder)/"update.zip")
            self.assertEqual(report["source_files"], 1)


if __name__ == "__main__":
    unittest.main()
