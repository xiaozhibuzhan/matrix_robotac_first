"""Runner boundary regression with injected devices, not native-gait acceptance."""

from contextlib import ExitStack
from dataclasses import asdict
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from task3_sim.interfaces import Frame, RobotFeedback, RobotMotionCommand
from task3_sim.mission import MultiTargetMission
from task3_sim.multi import run_multi, write_json


class MultiStartup033Tests(unittest.TestCase):
    def test_summary_invalid_feedback_is_strict_json_and_source_summary_stays_invalid(self):
        feedback = RobotFeedback(False, *(np.full(3, np.nan) for _ in range(4)), -math.inf)
        summary = {"status": "CANCELLED", "official_dynamics": {
            "last_execution": {"gyro": np.float64(np.nan), "speed": float("inf")},
            "feedback": asdict(feedback)}, "shots": [], "ammo_remaining": 5,
            "mission": {"search": {"observed_bearings": {2: {"yaw_rad": .3}}}}}
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "summary.json"
            write_json(path, summary)
            written = json.loads(path.read_text(encoding="utf-8"),
                                 parse_constant=lambda token: self.fail(f"nonstandard JSON value {token}"))
        self.assertFalse(written["telemetry_serialization"]["valid"])
        self.assertIn("/official_dynamics/feedback/timestamp",
                      written["telemetry_serialization"]["nonfinite_paths"])
        self.assertIsNone(written["official_dynamics"]["feedback"]["timestamp"])
        self.assertFalse(written["official_dynamics"]["feedback"]["valid"])
        self.assertIsNone(written["official_dynamics"]["last_execution"]["gyro"])
        self.assertEqual(written["mission"]["search"]["observed_bearings"], {"2": {"yaw_rad": .3}})
        self.assertEqual(written["shots"], [])
        self.assertEqual(written["ammo_remaining"], 5)
        self.assertNotIn("telemetry_serialization", summary)
        self.assertEqual(summary["official_dynamics"]["feedback"]["timestamp"], -math.inf)
        self.assertTrue(np.isnan(summary["official_dynamics"]["feedback"]["rpy_rad"]).all())
        self.assertTrue(math.isnan(summary["official_dynamics"]["last_execution"]["gyro"]))

    def test_runner_discards_cold_start_and_gap_frame_then_recovers_from_invalid_feedback(self):
        calls, observations, invalidations, decisions = [], [], [], []
        instances = {}

        class RecordingMission(MultiTargetMission):
            def __init__(self, public):
                calls.append("mission_init")
                super().__init__(public)

            def update(self, frame, detections, feedback, now, ammo):
                observations.append((frame, list(detections), feedback))
                result = super().update(frame, detections, feedback, now, ammo)
                decisions.append(result)
                return result

            def invalidate_control_observation(self, now, reason="mode transition"):
                invalidations.append((now, reason))
                return super().invalidate_control_observation(now, reason)

        class InjectedRobot:
            # Host-loop boundary test only. No official SDK, gait, or scoring
            # result is represented by this deterministic injected device.
            def __init__(self, model, config):
                instances["robot"] = self
                self.data = SimpleNamespace(time=.01)
                self.observation_epoch = 0
                self.minimum_observation_timestamp_s = -math.inf
                self.renderer = SimpleNamespace(update_scene=lambda *a, **kw: None,
                    scene=SimpleNamespace(ngeom=0, maxgeom=0), render=lambda: np.zeros((4, 4, 3), np.uint8))
                self.render_option = object()
                self.capture_count = self.feedback_count = 0
                self.last_execution = {}
                self.session = SimpleNamespace(last_feedback_meta={})
                self.feedbacks = []

            def warmup_renderer(self):
                calls.append("renderer_warmup")

            def Connect(self):
                calls.append("connect")

            def StandUp(self):
                calls.append("stand_up")

            def current_time(self):
                return self.data.time

            def capture(self):
                self.capture_count += 1
                calls.append(f"capture_{self.capture_count}")
                if self.capture_count == 2:
                    # Simulate a genuine recovery epoch. The boundary frame
                    # must be discarded rather than earning dwell time.
                    self.observation_epoch += 1
                    self.minimum_observation_timestamp_s = self.data.time
                return Frame(np.full((4, 4, 3), 7, np.uint8), self.data.time)

            def ReadFeedback(self):
                self.feedback_count += 1
                if self.feedback_count == 1:
                    result = RobotFeedback(False, *(np.full(3, np.nan) for _ in range(4)), -math.inf)
                    error = "injected source unavailable"
                else:
                    result = RobotFeedback(True, *(np.zeros(3) for _ in range(4)), self.data.time)
                    error = None
                self.feedbacks.append(result)
                self.session.last_feedback_meta = {"valid": result.valid, "error": error,
                    "source_timestamp_s": result.timestamp, "read_at_s": self.data.time}
                return result

            def ExecuteDecision(self, decision, **kwargs):
                self.last_execution = {"applied_command": asdict(decision.command),
                                       "gyro": np.linalg.norm(kwargs["feedback"].gyro_rad_s)}
                return False

            def advance(self, duration):
                # Keep the injected source acquisition clock monotonic even
                # if file I/O consumes the entire scheduled frame interval.
                self.data.time += max(.1, duration)

            def Stop(self):
                calls.append("stop")

            def report(self):
                return {"injected_test_device": True, "last_execution": self.last_execution,
                        "first_invalid_feedback": asdict(self.feedbacks[0]) if self.feedbacks else None}

            def close(self):
                calls.append("robot_close")

        class InjectedPerception:
            def __init__(self, public):
                self.frames = []
                instances["perception"] = self

            def detect(self, frame):
                self.frames.append(frame)
                calls.append("detector_warmup" if not np.any(frame.rgb) else "detect_observation")
                return []

            def close(self):
                calls.append("perception_close")

        class InjectedScorer:
            def __init__(self, *args):
                instances["scorer"] = self
                self.ammo_remaining, self.ledger, self.last_scores = 5, [], {}

            def shooting_init(self):
                calls.append("shooter_init")

            def shooting_fire(self, *args):
                raise AssertionError("discarded prewarm/invalid feedback must never authorize a shot")

            def shooting_shutdown(self):
                calls.append("shooter_close")

        class InjectedUI:
            def __init__(self, **kwargs):
                self.count, self.viewer = 0, None

            def on_viewer_key(self, *args):
                pass

            def present(self, display):
                self.count += 1
                return self.count < 3

            def hold_result(self, *args):
                return "injected test complete"

            def close(self):
                calls.append("ui_close")

        def launch_viewer(*args, **kwargs):
            calls.append("viewer_warmup")
            return SimpleNamespace(cam=object(), opt=SimpleNamespace(geomgroup=[1] * 6))

        public = {"camera": {"rotation_base_from_camera": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                             "position_base_m": [0, 0, 0]},
                  "laser": {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]},
                  "targets": [{"id": 2, "tag": {"size_m": .16, "bullseye_offset_tag_m": [0, .2, 0]}}],
                  "control": {"max_frame_age_s": .15, "max_feedback_age_s": .15}}
        config = {**public, "backend": "kinematic", "camera": {**public["camera"], "height": 4, "width": 4},
                  "simulation": {"frame_rate_hz": 10}, "observation": {}}

        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            root = Path(temporary)
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            args = SimpleNamespace(config=config_path, model_root=root, output=root / "run",
                                   show=False, viewer=True, realtime=False, max_seconds=1.0, hold_seconds=0)
            patches = [
                patch("task3_sim.multi.enable_fault_log"),
                patch("task3_sim.multi.save_rgb"),
                patch("task3_sim.multi.annotate_multi", side_effect=lambda frame, *a, **kw: frame.rgb),
                patch("task3_sim.multi.configure_multi_overview"),
                patch("task3_sim.backend.MujocoRobot", InjectedRobot),
                patch("task3_sim.mission.MultiTargetMission", RecordingMission),
                patch("task3_sim.multi_config.public_mission_config", return_value=public),
                patch("task3_sim.multi_scene.build_multi_scene", return_value=(object(), {
                    "source_model": "injected test model", "source_files_sha256": {}})),
                patch("task3_sim.multi_perception.MultiTagPerception", InjectedPerception),
                patch("task3_sim.multi_scoring.MultiRayScorer", InjectedScorer),
                patch("task3_sim.ui.InteractiveSession", InjectedUI),
                patch("mujoco.viewer.launch_passive", side_effect=launch_viewer),
            ]
            for entry in patches:
                stack.enter_context(entry)
            code, output = run_multi(args)
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"),
                                 parse_constant=lambda token: self.fail(f"invalid JSON token {token}"))
            events = [json.loads(line, parse_constant=lambda token: self.fail(f"invalid JSON token {token}"))
                      for line in (output / "events.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertFalse((output / "error.log").exists())

        self.assertEqual(code, 2)  # Deliberate UI cancellation, never native success.
        self.assertEqual(summary["status"], "CANCELLED")
        self.assertTrue(summary["shutdown_completed"])
        self.assertEqual(summary["shot_count"], 0)
        self.assertEqual(summary["ammo_remaining"], 5)
        self.assertEqual([entry["state"] for entry in events], ["WAIT", "WAIT", "SEARCHING"])
        self.assertEqual(events[0]["reason"], "robot feedback invalid")
        self.assertFalse(events[0]["feedback"]["valid"])
        self.assertIsNone(events[0]["feedback"]["timestamp"])
        self.assertIn("/feedback/timestamp", events[0]["telemetry_serialization"]["nonfinite_paths"])
        self.assertTrue(events[2]["feedback"]["valid"])
        self.assertTrue(events[2]["telemetry_serialization"]["valid"])
        self.assertFalse(events[1]["frame_delivered"])
        self.assertEqual(events[1]["detections"], [])
        self.assertEqual(events[1]["official_observation_epoch"], 1)
        self.assertEqual(len(invalidations), 1)
        self.assertIn("native feedback gap", invalidations[0][1])
        self.assertIsNone(observations[1][0])
        self.assertEqual(len(observations), 3)
        self.assertTrue(all(frame is None or np.all(frame.rgb == 7) for frame, _, _ in observations))
        self.assertFalse(any(decision.should_fire for decision in decisions))
        self.assertEqual(decisions[0].command, RobotMotionCommand())
        self.assertEqual(instances["scorer"].ledger, [])
        self.assertFalse(instances["robot"].feedbacks[0].valid)
        self.assertTrue(np.isnan(instances["robot"].feedbacks[0].gyro_rad_s).all())
        self.assertEqual(instances["robot"].feedbacks[0].timestamp, -math.inf)
        self.assertEqual(len(instances["perception"].frames), 3)  # warmup + first + recovered frame
        self.assertFalse(np.any(instances["perception"].frames[0].rgb))
        for phase in ("renderer_warmup", "detector_warmup", "viewer_warmup"):
            self.assertLess(calls.index(phase), calls.index("connect"))
            self.assertLess(calls.index(phase), calls.index("stand_up"))
        self.assertLess(calls.index("detector_warmup"), calls.index("mission_init"))
        self.assertLess(calls.index("detector_warmup"), calls.index("shooter_init"))
        self.assertLess(calls.index("stand_up"), calls.index("capture_1"))


if __name__ == "__main__":
    unittest.main()
