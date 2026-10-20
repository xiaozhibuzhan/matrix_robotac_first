"""Stage/lifecycle contracts; fake children never execute official Linux ELF."""

from __future__ import annotations

import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from task3_sim.official_runtime import OfficialRuntime, OfficialRuntimeError, sha256


def elf():
    return b"\x7fELF\x02\x01" + bytes(12) + b"\x3e\x00" + b"fake-test-binary"


class FakeChild:
    def __init__(self, pid):
        self.pid = pid
        self.returncode = None
        self.terminated = False
        self.killed = False
        self.timeout_once = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        if not self.timeout_once:
            self.returncode = -15

    def kill(self):
        self.killed = True
        self.returncode = -9

    def wait(self, timeout):
        if self.timeout_once:
            self.timeout_once = False
            raise subprocess.TimeoutExpired("owned-fake", timeout)
        return self.returncode


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        self.source = self.repo / "src/robot_mujoco"
        self.mc = self.repo / "src/robot_mc/build/export"
        self.write(self.source / "simulate/build/robot_mujoco", elf())
        self.write(self.source / "simulate/config.yaml", 'robot: "xgb"\nrobot_scene: "unreal.xml"\nrobot_path: "zsibot_robots"\nprint_scene_information: 1\nenable_elastic_band: 0\n')
        self.write(self.source / "zsibot_robots/xgb/xgb.xml", "<mujoco model='official'/>")
        self.write(self.source / "zsibot_robots/xgb/assets/model.stl", b"official-mesh")
        self.write(self.mc / "mc/bin/mc_ctrl", elf())
        self.write(self.mc / "mc/bin/librobot.so", b"official-lib")
        self.write(self.mc / "config/xg-user-parameters.yaml", "motor_platform_type: 8 # original\nbody_height: 0.32\n")
        self.write(self.mc / "config/sdk_config.yaml", 'target_ip: "127.0.0.1"\ntarget_port: 43988\n')
        self.write(self.mc / "onnx_model_crypto/policy.bin", b"official-policy")
        self.write(self.repo / "deps/zsibot_sdk/lib/zsl-1/x86_64/libmc_sdk_zsl_1_x86_64.so", elf())
        self.write(self.repo / "deps/zsibot_sdk/lib/zsl-1/x86_64/mc_sdk_zsl_1_py.cpython-310-x86_64-linux-gnu.so", elf())
        self.scene = self.repo / "task3_sim/generated/scene.xml"
        self.write(self.scene, "<mujoco model='task3-physical'/>")
        self.bridge = self.repo / "task3_sim/local/native/task3_official_state_bridge"
        self.write(self.bridge, elf())
        # Linux preflight correctly checks X_OK. These bytes are never launched
        # (Popen is injected), but the fixture must have a real bridge's mode.
        self.bridge.chmod(self.bridge.stat().st_mode | stat.S_IXUSR)
        self.original_hashes = {str(p): sha256(p) for p in (self.repo / "src").rglob("*") if p.is_file()}
        self.calls = []
        self.children = []
        self.now = 0.0

    def write(self, path, content):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content.encode() if isinstance(content, str) else content)

    def wait(self, seconds):
        self.now += seconds

    def runner(self, command, **kwargs):
        if command[0] == "ps":
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        return SimpleNamespace(returncode=0, stdout="libmujoco.so.3.3.0 => /usr/lib/libmujoco.so.3.3.0\n", stderr="")

    def launcher(self, command, **kwargs):
        self.calls.append((command, kwargs))
        child = FakeChild(1000 + len(self.children))
        self.children.append(child)
        if Path(command[0]).name == "task3_official_state_bridge":
            kwargs["stdout"].write(b"TASK3_OFFICIAL_STATE_READY port=44889\n")
            kwargs["stdout"].flush()
        return child

    def runtime(self, **kwargs):
        runtime = OfficialRuntime(self.repo, self.repo / "task3_sim/runs/test/official_runtime", self.scene,
                                  run_command=self.runner, popen=self.launcher, clock=lambda: self.now,
                                  wait=self.wait, **kwargs)
        return runtime

    def assert_sources_unchanged(self):
        self.assertEqual(self.original_hashes, {str(p): sha256(p) for p in (self.repo / "src").rglob("*") if p.is_file()})

    def mock_native(self, runtime):
        self.addCleanup(runtime.close)
        native = patch.object(runtime, "_native_supported", return_value=True)
        native.start()
        self.addCleanup(native.stop)
        display = patch.dict(os.environ, {"DISPLAY": ":fake-test"})
        display.start()
        self.addCleanup(display.stop)
        lock = patch.object(runtime, "_acquire_global_lock")
        lock.start()
        self.addCleanup(lock.stop)

    def test_staging_changes_only_copied_configs_and_verifies_all_bytes(self):
        runtime = self.runtime()
        manifest = runtime.prepare()
        self.assertTrue(manifest["sources_unchanged"])
        self.assertEqual(sha256(runtime.sim_binary), sha256(self.source / "simulate/build/robot_mujoco"))
        self.assertEqual(sha256(runtime.mc_binary), sha256(self.mc / "mc/bin/mc_ctrl"))
        self.assertEqual(runtime.staged_scene.read_bytes(), self.scene.read_bytes())
        self.assertIn("motor_platform_type: 5", (runtime.workdir / "mc/build/export/config/xg-user-parameters.yaml").read_text())
        self.assertIn("body_height: 0.32", (runtime.workdir / "mc/build/export/config/xg-user-parameters.yaml").read_text())
        self.assertIn("target_port: 44888", (runtime.workdir / "mc/build/export/config/sdk_config.yaml").read_text())
        self.assertIn('robot_scene: "task3.xml"', (runtime.workdir / "simulate/config.yaml").read_text())
        self.assertEqual(sum(row["derived_config_changed"] for row in manifest["copies"]), 3)
        self.assert_sources_unchanged()

    def test_optional_plugins_and_unchanged_models_are_staged(self):
        self.write(self.source / "simulate/build/mujoco_plugin/official.so", b"official-plugin")
        runtime = self.runtime()
        runtime.prepare()
        self.assertEqual((runtime.sim_binary.parent / "mujoco_plugin/official.so").read_bytes(), b"official-plugin")
        self.assertEqual((runtime.workdir / "mc/build/export/onnx_model_crypto/policy.bin").read_bytes(), b"official-policy")

    def test_refuses_src_or_repo_root_as_workdir(self):
        for path in (self.repo, self.repo / "src/robot_mujoco", self.repo / "task3_sim"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                OfficialRuntime(self.repo, path, self.scene)

    def test_prepare_refuses_nonempty_directory_and_preserves_it(self):
        runtime = self.runtime()
        sentinel = runtime.workdir / "existing.txt"
        self.write(sentinel, "keep")
        with self.assertRaisesRegex(OfficialRuntimeError, "overwrite"):
            runtime.prepare()
        self.assertEqual(sentinel.read_text(), "keep")
        self.assert_sources_unchanged()

    def test_source_mutation_after_prepare_is_detected(self):
        runtime = self.runtime()
        runtime.prepare()
        self.write(self.mc / "mc/bin/librobot.so", b"externally-changed")
        with self.assertRaisesRegex(OfficialRuntimeError, "changed"):
            runtime.prepare()

    def test_staging_copy_corruption_is_rejected(self):
        runtime = self.runtime()
        def corrupt(source, destination, **kwargs):
            Path(destination).write_bytes(b"corrupt")
        with patch("task3_sim.official_runtime.shutil.copy2", side_effect=corrupt):
            with self.assertRaisesRegex(OfficialRuntimeError, "verification"):
                runtime.prepare()
        self.assert_sources_unchanged()

    def test_static_preflight_does_not_claim_windows_runtime_tested(self):
        runtime = self.runtime()
        with patch.object(runtime, "_native_supported", return_value=False):
            report = runtime.preflight(self.bridge)
        self.assertFalse(report["ready"])
        self.assertTrue(any("Linux" in item for item in report["errors"]))
        self.assertEqual(self.calls, [])

    def test_missing_dependencies_reported_before_any_process_starts(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        def missing(command, **kwargs):
            if command[0] == "ldd":
                return SimpleNamespace(returncode=0, stdout="libecal.so.5 => not found\n", stderr="")
            return self.runner(command, **kwargs)
        runtime.run_command = missing
        report = runtime.preflight(self.bridge)
        self.assertFalse(report["ready"])
        self.assertIn("libecal", report["libraries"]["state_bridge"])
        with self.assertRaisesRegex(OfficialRuntimeError, "preflight"):
            runtime.start(44889, self.bridge)
        self.assertEqual(self.calls, [])

    def test_existing_official_process_rejects_global_bus_start_without_killing(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        def occupied(command, **kwargs):
            if command[0] == "ps":
                return SimpleNamespace(returncode=0, stdout="789 mc_ctrl\n790 python\n791 robot_mujoco\n", stderr="")
            return self.runner(command, **kwargs)
        runtime.run_command = occupied
        report = runtime.preflight(self.bridge)
        self.assertEqual([x["pid"] for x in report["conflicting_processes"]], [789, 791])
        with self.assertRaisesRegex(OfficialRuntimeError, "active"):
            runtime.start(44889, self.bridge)
        self.assertEqual(self.calls, [])

    def test_nonexecutable_bridge_is_rejected_before_any_child_starts(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        access = os.access
        def bridge_not_executable(path, mode):
            if Path(path).resolve() == self.bridge.resolve() and mode == os.X_OK:
                return False
            return access(path, mode)
        with patch("task3_sim.official_runtime.os.access", side_effect=bridge_not_executable):
            report = runtime.preflight(self.bridge)
            self.assertTrue(any("State bridge is not executable" in error for error in report["errors"]))
            with self.assertRaisesRegex(OfficialRuntimeError, "not executable"):
                runtime.start(44889, self.bridge)
        self.assertEqual(self.calls, [])

    def test_start_subscriber_before_physics_and_controller_with_official_cwd(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        pids = runtime.start(44889, self.bridge)
        self.assertEqual(list(pids), ["official_state_bridge", "robot_mujoco", "mc_ctrl"])
        commands = [call[0] for call in self.calls]
        self.assertEqual(commands[0], [str(self.bridge.resolve()), "--port", "44889"])
        self.assertEqual(commands[1], [str(runtime.sim_binary), str(runtime.staged_scene)])
        self.assertEqual(commands[2], [str(runtime.mc_binary), "r"])
        self.assertEqual(self.calls[1][1]["cwd"], str(runtime.workdir / "simulate/build"))
        self.assertEqual(self.calls[2][1]["cwd"], str(runtime.workdir / "mc/build/export/mc/bin"))
        self.assertEqual(self.calls[2][1]["env"]["ROBOT_TYPE"], "XG")
        self.assertEqual(self.calls[2][1]["env"]["SDK_CLIENT_IP"], "127.0.0.1")
        self.assertTrue(self.calls[2][1]["env"]["LD_LIBRARY_PATH"].startswith(str(runtime.mc_binary.parent)))
        self.assertTrue(all("shell" not in kwargs for _, kwargs in self.calls))
        self.assertTrue(runtime.check_alive())
        report = runtime.report()
        self.assertTrue(report["all_official_processes_launched"])
        self.assertIsNone(report["startup_error"])
        self.assertEqual(set(report["processes"]), {"official_state_bridge", "robot_mujoco", "mc_ctrl"})
        self.assertEqual(report["official_executables_sha256"]["mc_ctrl"], sha256(self.mc / "mc/bin/mc_ctrl"))
        runtime.close()
        self.assertTrue(runtime.report()["closed"])
        self.assertTrue(all(child.terminated for child in self.children))
        self.assert_sources_unchanged()

    def test_child_loopback_override_does_not_change_hardware_host_environment(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        with patch.dict(os.environ, {"SDK_CLIENT_IP": "192.168.234.1"}):
            runtime.start(44889, self.bridge)
            self.assertTrue(all(kwargs["env"]["SDK_CLIENT_IP"] == "127.0.0.1"
                                for _, kwargs in self.calls))
            self.assertEqual(os.environ["SDK_CLIENT_IP"], "192.168.234.1")
            self.assertEqual(runtime.preflight_report["sdk_controller_bind_ip"], "127.0.0.1")
        self.assert_sources_unchanged()

    def test_controller_sdk_failure_is_not_hidden_by_a_live_process(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        runtime.start(44889, self.bridge)
        with runtime.logs["mc_ctrl"].open("ab") as log:
            log.write(b"client_ip: 192.168.234.1\nbind: Cannot assign requested address\n"
                      b"sdk init failed!\nInitialized FSM state: PASSIVE\n")
        self.assertIsNone(self.children[-1].poll())
        with self.assertRaisesRegex(OfficialRuntimeError, "SDK initialization failed.*SDK_CLIENT_IP"):
            runtime.check_alive()
        runtime.close()
        self.assertTrue(all(child.terminated for child in self.children))
        self.assert_sources_unchanged()

    def test_bridge_failure_starts_no_physics_or_controller(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        def failed_bridge(command, **kwargs):
            child = self.launcher(command, **kwargs)
            child.returncode = 2
            return child
        runtime.popen = failed_bridge
        with self.assertRaisesRegex(OfficialRuntimeError, "bridge exited"):
            runtime.start(44889, self.bridge)
        self.assertEqual(len(self.calls), 1)
        self.assertFalse(runtime.report()["all_official_processes_launched"])
        self.assertIn("bridge exited", runtime.report()["startup_error"])
        self.assert_sources_unchanged()

    def test_dead_child_reports_its_log_and_close_only_owns_its_pids(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        runtime.start(44889, self.bridge)
        self.children[-1].returncode = 12
        with self.assertRaisesRegex(OfficialRuntimeError, "mc_ctrl.*12"):
            runtime.check_alive()
        runtime.close()
        status = json.loads((runtime.workdir / "runtime_status.json").read_text())
        self.assertEqual({x["pid"] for x in status["owned_processes"].values()}, {1000, 1001, 1002})
        self.assertTrue(status["source_files_unchanged"])
        self.assertEqual(status["cleanup_errors"], [])
        self.assertEqual(status["owned_processes"]["mc_ctrl"]["exit_code_before_cleanup"], 12)
        self.assertFalse(status["owned_processes"]["mc_ctrl"]["termination_requested"])
        self.assertTrue(status["owned_processes"]["robot_mujoco"]["termination_requested"])

    def test_unresponsive_owned_child_gets_only_its_own_kill(self):
        runtime = self.runtime()
        self.mock_native(runtime)
        runtime.start(44889, self.bridge)
        self.children[-1].timeout_once = True
        runtime.close()
        self.assertTrue(self.children[-1].killed)
        status = json.loads((runtime.workdir / "runtime_status.json").read_text())
        self.assertTrue(status["owned_processes"]["mc_ctrl"]["forced_kill"])
        self.assertFalse(self.children[0].killed)
        self.assertFalse(self.children[1].killed)

    def test_identical_sdk_and_state_port_refused(self):
        runtime = self.runtime()
        with self.assertRaises(ValueError):
            runtime.start(44888, self.bridge)
        self.assertEqual(self.calls, [])


if __name__ == "__main__":
    unittest.main()
