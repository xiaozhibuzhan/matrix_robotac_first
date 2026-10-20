"""Diagnostic JSON must record invalid feedback without making it valid."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import unittest

import numpy as np

from task3_sim.interfaces import Frame, RobotFeedback, RobotMotionCommand
from task3_sim.mission import MultiTargetMission
from task3_sim.official_sdk import OfficialSDKSession
from task3_sim.telemetry import SERIALIZATION_FIELD, sanitize_record


class _ReadSDK:
    """Injected getter contract; no proprietary controller or native runtime."""

    def __init__(self, fail=False):
        self.fail = fail

    def checkConnect(self):
        return True

    def getRPY(self):
        if self.fail:
            raise RuntimeError("injected source read failed")
        return [0.0, 0.0, 0.0]

    def getPosition(self):
        return [0.0, 0.0, 0.27]

    def getBodyGyro(self):
        return [0.0, 0.0, 0.0]

    def getWorldVelocity(self):
        return [0.0, 0.0, 0.0]


def _mission():
    return MultiTargetMission({
        "camera": {"rotation_base_from_camera": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                   "position_base_m": [0, 0, 0]},
        "laser": {"origin_base_m": [0, 0, 0], "direction_base": [1, 0, 0]},
        "targets": [{"id": 2, "tag": {"size_m": .16, "bullseye_offset_tag_m": [0, .2, 0]}}],
        "control": {"max_frame_age_s": .15, "max_feedback_age_s": .15},
    })


class TelemetryTests(unittest.TestCase):
    def roundtrip(self, record):
        return json.loads(json.dumps(sanitize_record(record), allow_nan=False, ensure_ascii=False))

    def test_sdk_getter_failure_wait_event_is_writable_without_mutating_invalid_feedback(self):
        session = OfficialSDKSession(sdk=_ReadSDK(fail=True), clock=lambda: 12.0,
                                     background_watchdog=False)
        session.connected = True
        feedback = session.ReadFeedback()
        self.assertFalse(feedback.valid)
        self.assertEqual(feedback.timestamp, -math.inf)
        self.assertTrue(np.isnan(feedback.rpy_rad).all())
        mission = _mission()
        frame = Frame(np.zeros((2, 2, 3), dtype=np.uint8), 12.0)
        decision = mission.update(frame, [], feedback, 12.0, 5)
        event = {"state": decision.state, "reason": decision.reason,
                 "command": asdict(decision.command), "feedback": asdict(feedback),
                 "sdk_feedback": session.last_feedback_meta, "ammo_remaining": 5}
        with self.assertRaises(ValueError):
            json.dumps(event, default=lambda value: value.tolist(), allow_nan=False)
        written = self.roundtrip(event)
        self.assertEqual(written["state"], "WAIT")
        self.assertEqual(written["reason"], "robot feedback invalid")
        self.assertFalse(written["feedback"]["valid"])
        self.assertIsNone(written["feedback"]["timestamp"])
        self.assertEqual(written["feedback"]["rpy_rad"], [None, None, None])
        self.assertEqual(written["command"], asdict(RobotMotionCommand()))
        self.assertEqual(written["ammo_remaining"], 5)
        self.assertEqual(mission.summary()["shots"], [])
        meta = written[SERIALIZATION_FIELD]
        self.assertFalse(meta["valid"])
        self.assertEqual(len(meta["nonfinite_paths"]), 13)
        self.assertIn({"path": "/feedback/timestamp", "value": "-Infinity"}, meta["nonfinite_values"])
        self.assertFalse(feedback.valid)
        self.assertEqual(feedback.timestamp, -math.inf)
        self.assertTrue(np.isnan(feedback.rpy_rad).all())
        self.assertTrue(np.isnan(event["feedback"]["rpy_rad"]).all())
        self.assertFalse(session.last_feedback_meta["valid"])
        self.assertEqual(session.last_feedback_meta["read_at_s"], 12.0)

    def test_invalid_robot_state_source_preserves_sdk_diagnostic_vectors_and_invalid_timestamp(self):
        source = RobotFeedback(False, *(np.full(3, np.nan) for _ in range(4)), -math.inf)
        session = OfficialSDKSession(sdk=_ReadSDK(), clock=lambda: 12.0,
                                     state_feedback_provider=lambda: source, background_watchdog=False)
        session.connected = True
        feedback = session.ReadFeedback()
        decision = _mission().update(Frame(np.zeros((2, 2, 3)), 12.0), [], feedback, 12.0, 5)
        written = self.roundtrip({"state": decision.state, "feedback": feedback,
                                  "sdk_feedback": session.last_feedback_meta})
        self.assertEqual(written["state"], "WAIT")
        self.assertFalse(written["feedback"]["valid"])
        self.assertIsNone(written["feedback"]["timestamp"])
        self.assertEqual(written["feedback"]["position_m"], [0.0, 0.0, .27])
        self.assertEqual(written[SERIALIZATION_FIELD]["nonfinite_paths"], ["/feedback/timestamp"])
        self.assertEqual(feedback.timestamp, -math.inf)
        self.assertTrue(np.isnan(source.rpy_rad).all())

    def test_numpy_scalars_arrays_and_diagnostic_nonfinite_values_have_exact_paths(self):
        value = {"detection/quality~raw": np.array([[np.nan, np.inf], [-np.inf, .25]], dtype=np.float32),
                 "scalar": np.float64(np.nan), "sequence": (np.int64(7), np.bool_(False)),
                 "output": Path("task3_sim") / "runs"}
        written = self.roundtrip(value)
        self.assertEqual(written["detection/quality~raw"], [[None, None], [None, .25]])
        self.assertEqual(written["sequence"], [7, False])
        self.assertEqual(written["output"], str(value["output"]))
        self.assertEqual(written[SERIALIZATION_FIELD]["nonfinite_values"], [
            {"path": "/detection~1quality~0raw/0/0", "value": "NaN"},
            {"path": "/detection~1quality~0raw/0/1", "value": "+Infinity"},
            {"path": "/detection~1quality~0raw/1/0", "value": "-Infinity"},
            {"path": "/scalar", "value": "NaN"},
        ])
        self.assertTrue(np.isnan(value["detection/quality~raw"][0, 0]))

    def test_finite_scores_identity_timestamps_and_valid_flags_are_not_reinterpreted(self):
        event = {"feedback": {"valid": False, "timestamp": .25}, "frame_timestamp_s": .24,
                 "intended_target_id": np.int64(9), "shot": {"ring": 0, "hit": False},
                 "ammo_remaining": 4, "reason": "invalid source", "error": None}
        written = self.roundtrip(event)
        self.assertEqual({key: value for key, value in written.items() if key != SERIALIZATION_FIELD},
                         {**event, "intended_target_id": 9})
        self.assertEqual(written[SERIALIZATION_FIELD],
                         {"schema": 1, "valid": True, "nonfinite_paths": [], "nonfinite_values": []})
        self.assertNotIn(SERIALIZATION_FIELD, event)

    def test_zero_dimensional_and_numpy_float_scalar_values_are_supported(self):
        written = self.roundtrip({"a": np.array(np.inf), "b": np.float16(.5), "c": np.longdouble(-np.inf)})
        self.assertIsNone(written["a"])
        self.assertEqual(written["b"], .5)
        self.assertIsNone(written["c"])
        self.assertEqual(written[SERIALIZATION_FIELD]["nonfinite_paths"], ["/a", "/c"])

    def test_unrecognized_values_and_nonfinite_keys_are_not_silently_rewritten(self):
        for value in ({"payload": object()}, {"complex": np.complex128(1+2j)}, {float("nan"): "value"}):
            with self.subTest(value=value), self.assertRaises(TypeError):
                sanitize_record(value)

    def test_mission_integer_id_maps_use_json_string_keys_without_losing_values(self):
        written = self.roundtrip({"observed_bearings": {2: {"yaw_rad": np.float32(.5)},
                                                       np.int64(9): {"yaw_rad": np.nan}},
                                  "keys": {None: 1, False: 2, 1.5: 3}})
        self.assertEqual(written["observed_bearings"], {"2": {"yaw_rad": .5}, "9": {"yaw_rad": None}})
        self.assertEqual(written["keys"], {"null": 1, "false": 2, "1.5": 3})
        self.assertEqual(written[SERIALIZATION_FIELD]["nonfinite_paths"], ["/observed_bearings/9/yaw_rad"])
        with self.assertRaisesRegex(ValueError, "key collision"):
            sanitize_record({"bearing": {2: "one", "2": "two"}})

    def test_reserved_metadata_and_circular_references_fail_explicitly(self):
        with self.assertRaisesRegex(ValueError, "sanitize a record only once"):
            sanitize_record({SERIALIZATION_FIELD: {"valid": False}})
        cycle = []
        cycle.append(cycle)
        with self.assertRaisesRegex(ValueError, "circular telemetry"):
            sanitize_record({"cycle": cycle})

    def test_shared_arrays_are_copied_without_being_misclassified_as_cycles(self):
        source = np.array([.2, np.nan])
        written = self.roundtrip({"left": source, "right": source})
        self.assertEqual(written["left"], [.2, None])
        self.assertEqual(written["right"], [.2, None])
        self.assertEqual(written[SERIALIZATION_FIELD]["nonfinite_paths"], ["/left/1", "/right/1"])


if __name__ == "__main__":
    unittest.main()
