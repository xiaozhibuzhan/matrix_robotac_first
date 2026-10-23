"""Missing native dependencies must fail before GUI creation or SDK startup."""

from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from task3_sim.multi import run_multi


class StartupDependencies037Tests(unittest.TestCase):
    def run_injected(self, report):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        config_path = root / "config.json"
        config_path.write_text(json.dumps({"backend": "official_dynamics"}), encoding="utf-8")
        args = SimpleNamespace(config=config_path, model_root=root, output=root/"run",
                               show=True, viewer=True, realtime=False, max_seconds=1, hold_seconds=0)
        runtime = Mock()
        runtime.preflight.return_value = report
        robot_constructor = Mock(side_effect=RuntimeError("injected stop after dependency check"))
        robot_constructor.motion_model = "official_native_test_fixture"
        perception = Mock()
        ui = Mock()
        renderer = Mock()
        stdout = io.StringIO()
        with ExitStack() as stack:
            for context in (
                patch("task3_sim.multi.enable_fault_log"),
                patch("task3_sim.multi_scene.build_multi_scene", return_value=(object(), {
                    "source_model": "injected model", "source_files_sha256": {}})),
                patch("task3_sim.official_scene.prepare_scene", return_value=(object(), root/"scene.xml")),
                patch("task3_sim.official_runtime.OfficialRuntime", return_value=runtime),
                patch("task3_sim.official_backend.OfficialDynamicsRobot", robot_constructor),
                patch("task3_sim.multi_perception.MultiTagPerception", perception),
                patch("task3_sim.ui.InteractiveSession", ui),
                patch("mujoco.Renderer", renderer),
                redirect_stdout(stdout),
            ):
                stack.enter_context(context)
            code, output = run_multi(args)
        summary = json.loads((output/"summary.json").read_text(encoding="utf-8"))
        written = json.loads((output/"startup_preflight.json").read_text(encoding="utf-8"))
        return code, output, summary, written, stdout.getvalue(), runtime, robot_constructor, perception, ui, renderer

    def test_missing_bridge_is_reported_without_any_graphics_robot_or_mission_startup(self):
        report = {"ready": False, "errors": ["Missing state_bridge: fixture"], "repair_actions": [
            {"cwd": "/fixture/project", "command": "bash task3_sim/build_official_bridge.sh"}]}
        code, output, summary, written, stdout, runtime, robot, perception, ui, renderer = self.run_injected(report)
        self.assertEqual(code, 2)
        self.assertEqual(summary["status"], "ERROR")
        self.assertTrue(summary["shutdown_completed"])
        self.assertFalse(summary["official_dynamics_mission_completed"])
        self.assertEqual(summary["official_startup_preflight"], report)
        self.assertEqual(written, report)
        self.assertIn("before graphics startup", summary["error"])
        self.assertIn("TASK3_REPAIR", stdout)
        self.assertIn("bash task3_sim/build_official_bridge.sh", stdout)
        for resource in (robot, perception, ui, renderer):
            resource.assert_not_called()
        runtime.start.assert_not_called()
        self.assertFalse((output/"events.jsonl").exists())
        self.assertFalse((output/"shots.csv").exists())

    def test_successful_read_only_check_allows_robot_creation_without_starting_native_processes_itself(self):
        report = {"ready": True, "errors": [], "repair_actions": []}
        code, output, summary, written, stdout, runtime, robot, perception, ui, renderer = self.run_injected(report)
        runtime.preflight.assert_called_once()
        robot.assert_called_once()
        self.assertEqual(written, report)
        self.assertEqual(summary["error"], "injected stop after dependency check")
        self.assertNotIn("TASK3_REPAIR", stdout)
        runtime.start.assert_not_called()
        for resource in (perception, ui, renderer):
            resource.assert_not_called()


if __name__ == "__main__":
    unittest.main()
