"""Contract tests; injected fakes do not exercise the proprietary Linux SDK."""

from __future__ import annotations

import math
import unittest
from unittest.mock import patch

import numpy as np

from task3_sim.interfaces import RobotFeedback, RobotMotionCommand
from task3_sim.official_sdk import (FeedbackStamp, OfficialSDKError, OfficialSDKSession,
                                   SDKLimits, SDKWatchdogError, load_official_sdk)


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.sequence = 0
        self.frozen_stamp = None

    def __call__(self):
        return self.now

    def wait(self, seconds):
        self.now += seconds
        self.sequence += 1

    def stamp(self):
        return self.frozen_stamp or FeedbackStamp(self.sequence, self.now, self.now)


class FakeHighLevel:
    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.connected = True
        self.ctrlmode = 0
        self.result = {}
        self.rpy = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.27]
        self.gyro = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.force_moving = False

    def initRobot(self, *args):
        self.calls.append((self.clock(), "initRobot", args))

    def checkConnect(self):
        return self.connected

    def _action(self, name, *args):
        self.calls.append((self.clock(), name, args))
        return self.result.get(name, 0)

    def standUp(self):
        result = self._action("standUp")
        if not result:
            self.ctrlmode = 1
        return result

    def move(self, vx, vy, yaw):
        result = self._action("move", vx, vy, yaw)
        if not result:
            self.ctrlmode = 18
            self.velocity = [vx, vy, 0.0]
            self.gyro = [0.0, 0.0, yaw]
        return result

    def attitudeControl(self, roll, pitch, yaw, height):
        result = self._action("attitudeControl", roll, pitch, yaw, height)
        if not result:
            self.ctrlmode = 21
            self.velocity = [0.0, 0.0, height]
            self.gyro = [roll, pitch, yaw]
        return result

    def getRPY(self):
        return self.rpy

    def getPosition(self):
        return self.position

    def getBodyGyro(self):
        return [1.0, 0.0, 0.0] if self.force_moving else self.gyro

    def getWorldVelocity(self):
        return self.velocity

    def getCurrentCtrlmode(self):
        return self.ctrlmode


class OfficialSDKTests(unittest.TestCase):
    def session(self, *, stand=True, **kwargs):
        clock = FakeClock()
        sdk = FakeHighLevel(clock)
        opts = dict(sdk=sdk, clock=clock, wait=clock.wait,
                    freshness_provider=clock.stamp, background_watchdog=False)
        opts.update(kwargs)
        session = OfficialSDKSession(**opts)
        session.Connect()
        if stand:
            session.StandUp()
        return session, sdk, clock

    def test_loopback_and_connection_standup_before_first_move(self):
        session, sdk, clock = self.session(stand=False)
        self.assertEqual(sdk.calls, [(0.0, "initRobot", ("127.0.0.1", 43988, "127.0.0.1"))])
        session.StandUp()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
        self.assertEqual([c[1] for c in sdk.calls], ["initRobot", "standUp", "standUp", "move"])
        self.assertGreaterEqual(sdk.calls[3][0] - sdk.calls[2][0], 4.0)
        self.assertTrue(session.ReadFeedback().valid)

    def test_move_before_completed_standup_is_rejected(self):
        session, sdk, _ = self.session(stand=False)
        with self.assertRaisesRegex(OfficialSDKError, "StandUp"):
            session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
        self.assertFalse(any(c[1] == "move" for c in sdk.calls))

    def test_small_move_requests_zero_without_increasing_them(self):
        session, sdk, _ = self.session()
        mapped = session.ExecuteMove(RobotMotionCommand(0.049, -0.099, 0.019))
        self.assertEqual(mapped, RobotMotionCommand())
        self.assertEqual([e["axis"] for e in session.events if e["kind"] == "deadband_zero"], ["vx", "vy", "yaw"])
        self.assertFalse(any(c[1] == "move" and any(c[2]) for c in sdk.calls))

    def test_official_deadband_boundaries_both_signs_are_preserved(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                session, sdk, _ = self.session()
                command = RobotMotionCommand(sign * 0.05, sign * 0.1, sign * 0.02)
                self.assertEqual(session.ExecuteMove(command), command)
                self.assertEqual(sdk.calls[-1][2], (command.vx_m_s, command.vy_m_s, command.yaw_rate_rad_s))

    def test_over_limit_and_nonfinite_commands_rejected_with_current_mode_stop(self):
        for bad in (RobotMotionCommand(vx_m_s=0.151), RobotMotionCommand(yaw_rate_rad_s=math.nan),
                    RobotMotionCommand(pitch_rate_rad_s=math.inf)):
            with self.subTest(bad=bad):
                session, sdk, _ = self.session()
                session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
                with self.assertRaises(OfficialSDKError):
                    session.Execute(bad)
                self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))
                self.assertEqual(session.mode, "fault")

    def test_limits_cannot_exceed_official_ranges(self):
        for limits in (SDKLimits(max_vx_m_s=3.01), SDKLimits(max_vy_m_s=1.01),
                       SDKLimits(max_yaw_rate_rad_s=3.01), SDKLimits(max_pitch_rate_rad_s=0.51),
                       SDKLimits(max_attitude_yaw_rate_rad_s=0.51)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                OfficialSDKSession(limits=limits)

    def test_ambiguous_dispatch_and_pitch_through_move_are_rejected(self):
        for method in ("Execute", "ExecuteMove"):
            session, sdk, _ = self.session()
            with self.subTest(method=method), self.assertRaises(OfficialSDKError):
                getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=0.1, pitch_rate_rad_s=0.1))
            self.assertFalse(any(c[1] in ("move", "attitudeControl") and any(c[2]) for c in sdk.calls))

    def test_explicit_attitude_preserves_fine_yaw_and_pitch_without_move_deadband(self):
        session, sdk, _ = self.session()
        command = RobotMotionCommand(yaw_rate_rad_s=0.001, pitch_rate_rad_s=-0.003)
        self.assertEqual(session.ExecuteAttitude(command), command)
        self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0.0, -0.003, 0.001, 0.0)))
        self.assertFalse(any(e["kind"] == "deadband_zero" for e in session.events))

    def test_attitude_rejects_translation_and_its_own_yaw_limit(self):
        for command in (RobotMotionCommand(vx_m_s=0.01, pitch_rate_rad_s=0.1),
                        RobotMotionCommand(yaw_rate_rad_s=0.36)):
            session, sdk, _ = self.session()
            with self.subTest(command=command), self.assertRaises(OfficialSDKError):
                session.ExecuteAttitude(command)
            self.assertFalse(any(c[1] == "attitudeControl" and any(c[2]) for c in sdk.calls))

    def test_move_to_attitude_requires_zero_stationary_standup_and_four_seconds(self):
        session, sdk, clock = self.session()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
        start = clock()
        self.assertEqual(session.ExecuteAttitude(RobotMotionCommand(yaw_rate_rad_s=0.001, pitch_rate_rad_s=0.01)), RobotMotionCommand())
        self.assertFalse(any(c[1] == "attitudeControl" for c in sdk.calls))
        clock.wait(0.025)
        session.ExecuteAttitude(RobotMotionCommand(yaw_rate_rad_s=0.001, pitch_rate_rad_s=0.01))
        actions = [c for c in sdk.calls if c[1] in ("move", "standUp", "attitudeControl")]
        self.assertEqual([c[1] for c in actions], ["standUp", "standUp", "move", "move", "standUp", "standUp", "attitudeControl"])
        self.assertEqual(actions[3][2], (0.0, 0.0, 0.0))
        self.assertGreaterEqual(actions[4][0] - start, session.stationary_duration_s)
        self.assertGreaterEqual(actions[6][0] - actions[5][0], 4.0)

    def test_attitude_to_move_uses_matching_zero_and_standup(self):
        session, sdk, _ = self.session()
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=0.01))
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=-0.2))
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=-0.2))
        actions = [(c[1], c[2]) for c in sdk.calls]
        self.assertEqual(actions[-4:], [("attitudeControl", (0.0, 0.0, 0.0, 0.0)),
                                      ("standUp", ()), ("standUp", ()), ("move", (0.0, 0.0, -0.2))])

    def test_stop_and_close_in_attitude_never_call_move_liedown_or_passive(self):
        session, sdk, _ = self.session()
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=0.01))
        session.Stop()
        session.close()
        session.close()
        self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0.0, 0.0, 0.0, 0.0)))
        self.assertFalse(any(c[1] in ("move", "lieDown", "passive") for c in sdk.calls))

    def test_nonzero_status_is_not_silently_accepted(self):
        session, sdk, _ = self.session()
        sdk.result["move"] = 0x3013
        with self.assertRaisesRegex(OfficialSDKError, "0x3013"):
            session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
        self.assertEqual(session.mode, "fault")
        self.assertTrue(any(e["kind"] == "stop_failed" for e in session.events))

    def test_standup_failure_never_enters_move(self):
        session, sdk, _ = self.session(stand=False)
        sdk.result["standUp"] = 0x3007
        with self.assertRaisesRegex(OfficialSDKError, "0x3007"):
            session.StandUp()
        self.assertFalse(any(c[1] == "move" for c in sdk.calls))

    def test_unowned_motion_or_action_cannot_be_overridden_with_standup(self):
        for mode in (18, 21, 255):
            session, sdk, _ = self.session(stand=False)
            sdk.ctrlmode = mode
            with self.subTest(mode=mode), self.assertRaisesRegex(OfficialSDKError, "unowned"):
                session.StandUp()
            self.assertEqual([c[1] for c in sdk.calls], ["initRobot"])

    def test_void_status_requires_explicit_binding_compatibility(self):
        session, sdk, _ = self.session(stand=False)
        sdk.result["standUp"] = None
        with self.assertRaisesRegex(OfficialSDKError, "uint32"):
            session.StandUp()
        session, sdk, _ = self.session(stand=False, allow_void_status=True)
        sdk.result["standUp"] = None
        session.StandUp()
        self.assertTrue(any(e["kind"] == "sdk_status_unavailable" for e in session.events))

    def test_getter_read_time_is_not_a_fabricated_source_timestamp(self):
        session, sdk, clock = self.session(stand=False, freshness_provider=None)
        feedback = session.ReadFeedback()
        self.assertFalse(feedback.valid)
        self.assertEqual(feedback.timestamp, -math.inf)
        self.assertEqual(session.last_feedback_meta["read_at_s"], clock())
        self.assertIsNone(session.last_feedback_meta["source_timestamp_s"])
        session.freshness_provider = lambda: FeedbackStamp(1, None, clock())
        self.assertFalse(session.ReadFeedback().valid)

    def test_invalid_feedback_vectors_are_not_used(self):
        for field, values in (("gyro", [math.nan, 0, 0]), ("position", [0, 0]), ("rpy", [0, math.inf, 0])):
            session, sdk, _ = self.session()
            setattr(sdk, field, values)
            with self.subTest(field=field):
                self.assertFalse(session.ReadFeedback().valid)
                with self.assertRaisesRegex(OfficialSDKError, "fresh feedback"):
                    session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.2))
                self.assertFalse(any(c[1] == "move" and any(c[2]) for c in sdk.calls))

    def test_first_wait_with_invalid_robotstate_stays_standing_without_fault(self):
        session, sdk, clock = self.session(stand=False, freshness_provider=None)
        def state():
            return RobotFeedback(True, np.zeros(3), np.array([0, 0, .27]),
                                 np.zeros(3), np.zeros(3), clock())
        session.state_feedback_provider = state
        session.StandUp()
        session.state_feedback_provider = lambda: RobotFeedback(False, np.zeros(3), np.zeros(3),
                                                                 np.zeros(3), np.zeros(3), -math.inf)
        self.assertFalse(session.ReadFeedback().valid)
        invalid_metadata = session.last_feedback_meta.copy()
        before = list(sdk.calls)
        self.assertEqual(session.ExecuteAttitude(RobotMotionCommand()), RobotMotionCommand())
        self.assertEqual(sdk.calls, before)
        self.assertIsNone(session.fault)
        self.assertEqual(session.mode, "standing")
        self.assertIsNone(session._command_mode)
        self.assertEqual(session.last_feedback_meta, invalid_metadata)
        self.assertFalse(session.ReadFeedback().valid)
        self.assertEqual(session.ReadFeedback().timestamp, -math.inf)
        # New acquired state allows motion without reconnecting or standing up
        # again; the safety WAIT did not turn a transient loss into a fault.
        session.state_feedback_provider = state
        command = RobotMotionCommand(pitch_rate_rad_s=.01)
        self.assertEqual(session.ExecuteAttitude(command), command)

    def test_stale_feedback_zero_stops_only_the_owned_family_without_switching(self):
        for start, stop, operation in (("ExecuteMove", "ExecuteAttitude", "move"),
                                       ("ExecuteAttitude", "ExecuteMove", "attitudeControl")):
            with self.subTest(start=start):
                session, sdk, clock = self.session()
                getattr(session, start)(RobotMotionCommand(yaw_rate_rad_s=.1))
                mode = session.mode
                clock.frozen_stamp = clock.stamp()
                clock.wait(.2)  # Feedback expires before the command watchdog.
                self.assertFalse(session.ReadFeedback().valid)
                invalid_metadata = session.last_feedback_meta.copy()
                before = len(sdk.calls)
                self.assertEqual(getattr(session, stop)(RobotMotionCommand()), RobotMotionCommand())
                self.assertEqual(len(sdk.calls), before + 1)
                self.assertEqual(sdk.calls[-1][1], operation)
                self.assertFalse(any(sdk.calls[-1][2]))
                self.assertEqual(session.mode, mode)
                self.assertEqual(session._command_mode, mode)
                self.assertIsNone(session.fault)
                self.assertEqual(session.last_feedback_meta, invalid_metadata)
                self.assertFalse(session.ReadFeedback().valid)

    def test_stale_feedback_nonzero_still_rejected_and_current_family_stopped(self):
        for method, operation in (("ExecuteMove", "move"), ("ExecuteAttitude", "attitudeControl")):
            with self.subTest(method=method):
                session, sdk, clock = self.session()
                getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=.1))
                clock.frozen_stamp = clock.stamp()
                clock.wait(.2)
                before = len(sdk.calls)
                with self.assertRaisesRegex(OfficialSDKError, "fresh feedback"):
                    getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=.1))
                self.assertEqual(len(sdk.calls), before + 1)
                self.assertEqual(sdk.calls[-1][1], operation)
                self.assertFalse(any(sdk.calls[-1][2]))
                self.assertEqual(session.mode, "fault")

    def test_safety_zero_does_not_bypass_connection_standing_or_watchdog(self):
        session, _, _ = self.session(stand=False)
        with self.assertRaisesRegex(OfficialSDKError, "StandUp"):
            session.ExecuteAttitude(RobotMotionCommand())
        session, sdk, _ = self.session()
        sdk.connected = False
        with self.assertRaisesRegex(OfficialSDKError, "disconnected"):
            session.ExecuteAttitude(RobotMotionCommand())
        session, sdk, clock = self.session()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1))
        clock.wait(.3)
        with self.assertRaises(SDKWatchdogError):
            session.ExecuteAttitude(RobotMotionCommand())
        self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))
        with self.assertRaises(SDKWatchdogError):
            session.ExecuteMove(RobotMotionCommand())

    def test_safety_zero_keeps_issued_time_validation_and_deadband_policy(self):
        session, _, clock = self.session()
        with self.assertRaises(SDKWatchdogError):
            session.ExecuteMove(RobotMotionCommand(), issued_at_s=clock() - .3)
        session, sdk, clock = self.session()
        clock.frozen_stamp = clock.stamp()
        clock.wait(.2)
        self.assertFalse(session.ReadFeedback().valid)
        command = RobotMotionCommand(.049, -.099, .019)
        self.assertEqual(session.ExecuteMove(command), RobotMotionCommand())
        self.assertIsNone(session.fault)
        self.assertFalse(any(c[1] == "move" for c in sdk.calls))
        self.assertEqual([e["axis"] for e in session.events if e["kind"] == "deadband_zero"], ["vx", "vy", "yaw"])

    def test_stale_future_and_restamped_heartbeats_are_invalid(self):
        for stamp_kind in ("stale", "future", "restamped", "reverse"):
            session, sdk, clock = self.session()
            session.ReadFeedback()
            with self.subTest(kind=stamp_kind):
                if stamp_kind == "stale":
                    stamp = FeedbackStamp(clock.sequence, clock() - 1, clock())
                elif stamp_kind == "future":
                    stamp = FeedbackStamp(clock.sequence + 1, clock() + 1, clock() + 1)
                elif stamp_kind == "restamped":
                    stamp = FeedbackStamp(clock.sequence, clock() + 0.001, clock() + 0.001)
                    clock.wait(0.001)
                else:
                    stamp = FeedbackStamp(clock.sequence - 1, clock() - 0.001, clock())
                clock.frozen_stamp = stamp
                self.assertFalse(session.ReadFeedback().valid)

    def test_repeated_frozen_sample_cannot_satisfy_stationary_dwell(self):
        session, sdk, clock = self.session(stand=False, transition_timeout_s=4.5)
        clock.frozen_stamp = FeedbackStamp(0, clock(), clock())
        with self.assertRaisesRegex(OfficialSDKError, "stationary"):
            session.StandUp()
        self.assertFalse(session.standing)

    def test_standup_requires_actual_stopped_feedback(self):
        session, sdk, _ = self.session(stand=False, transition_timeout_s=4.5)
        sdk.force_moving = True
        with self.assertRaisesRegex(OfficialSDKError, "stationary"):
            session.StandUp()
        self.assertFalse(session.standing)

    def test_external_robotstate_source_is_not_attributed_to_sdk_getters(self):
        session, sdk, clock = self.session(stand=False, freshness_provider=None)
        session.state_feedback_provider = lambda: RobotFeedback(True, np.array([0.1, 0.2, 0.3]),
            np.array([1.0, 2.0, 0.27]), np.zeros(3), np.zeros(3), clock())
        session.StandUp()
        feedback = session.ReadFeedback()
        np.testing.assert_array_equal(feedback.rpy_rad, [0.1, 0.2, 0.3])
        self.assertEqual(session.last_feedback_meta["sdk_getter_values"][0], [0.0, 0.0, 0.0])
        self.assertIsNone(session.last_feedback_meta["received_at_s"])
        self.assertIn("external official RobotState", session.last_feedback_meta["timestamp_policy"])

    def test_getter_read_end_time_is_used_to_check_actual_sample(self):
        session, sdk, clock = self.session(stand=False)
        def delayed_stamp():
            clock.wait(0.01)
            return clock.stamp()
        session.freshness_provider = delayed_stamp
        feedback = session.ReadFeedback()
        self.assertTrue(feedback.valid)
        self.assertEqual(feedback.timestamp, clock())

    def test_watchdog_stops_in_current_mode_and_latches(self):
        for method, name in (("ExecuteMove", "move"), ("ExecuteAttitude", "attitudeControl")):
            session, sdk, clock = self.session()
            getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=0.1))
            clock.wait(0.3)
            with self.subTest(method=method), self.assertRaises(SDKWatchdogError):
                session.poll_watchdog()
            self.assertEqual(sdk.calls[-1][1], name)
            self.assertFalse(any(sdk.calls[-1][2]))
            with self.assertRaises(SDKWatchdogError):
                getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=0.1))

    def test_expired_command_and_clock_reversal_are_rejected(self):
        session, sdk, clock = self.session()
        with self.assertRaises(SDKWatchdogError):
            session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.1), issued_at_s=clock() - 0.3)
        self.assertFalse(any(c[1] == "move" and any(c[2]) for c in sdk.calls))
        session, sdk, clock = self.session()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.1))
        clock.now -= 0.1
        with self.assertRaises(SDKWatchdogError):
            session.poll_watchdog()

    def test_background_watchdog_stops_without_further_execute_calls(self):
        session, sdk, clock = self.session()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.1))
        clock.wait(0.3)
        session.start_watchdog()
        session._watchdog_thread.join(timeout=1.0)
        self.assertFalse(session._watchdog_thread.is_alive())
        self.assertIsInstance(session.fault, SDKWatchdogError)
        self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))
        session.close()

    def test_disconnect_stops_and_invalidates_feedback(self):
        session, sdk, _ = self.session()
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=0.1))
        sdk.connected = False
        self.assertFalse(session.ReadFeedback().valid)
        with self.assertRaisesRegex(OfficialSDKError, "disconnected"):
            session.poll_watchdog()
        self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))

    def test_native_loader_does_not_attempt_windows_or_wrong_python(self):
        with patch("task3_sim.official_sdk.sys.platform", "win32"), patch("task3_sim.official_sdk.ctypes.CDLL") as loader:
            with self.assertRaisesRegex(OfficialSDKError, "Linux CPython 3.10"):
                load_official_sdk()
            loader.assert_not_called()

    def test_refresh_is_optional_and_obeys_official_maximum_send_frequency(self):
        session, sdk, clock = self.session()
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.02))
        before = list(sdk.calls)
        clock.wait(.05)
        session.poll_refresh()
        self.assertEqual(sdk.calls, before)
        for bad in (0, -.04, .001, .3, math.inf, math.nan, True):
            with self.subTest(interval=bad), self.assertRaises(ValueError):
                OfficialSDKSession(refresh_interval_s=bad)

    def test_refresh_repeats_only_the_exact_original_family_and_payload(self):
        for method, operation, command, arguments in (
                ("ExecuteMove", "move", RobotMotionCommand(.05, -.1, .1), (.05, -.1, .1)),
                ("ExecuteAttitude", "attitudeControl", RobotMotionCommand(yaw_rate_rad_s=.003, pitch_rate_rad_s=-.02),
                 (0.0, -.02, .003, 0.0))):
            with self.subTest(method=method):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                issued = clock()-.01
                until = clock()+.15
                getattr(session, method)(command, issued_at_s=issued, valid_until_s=until)
                lease = session._command_lease
                before = len(sdk.calls)
                clock.wait(.039)
                session.poll_refresh()
                self.assertEqual(len(sdk.calls), before)
                for duration in (.002, .041):
                    clock.wait(duration)
                    session.poll_refresh()
                    self.assertIs(session._command_lease, lease)
                    self.assertEqual(session.last_command_at_s, issued)
                    self.assertEqual(sdk.calls[-1][1:], (operation, arguments))
                repeats = [e for e in session.events if e.get("refresh") is True]
                self.assertEqual(len(repeats), 2)
                for event in repeats:
                    self.assertEqual(event["original_issued_at_s"], issued)
                    self.assertEqual(event["valid_until_s"], until)
                    self.assertEqual(event["arguments"], list(arguments))
                    self.assertEqual(event["operation"], operation)
                refresh_events = [e for e in session.events if e["kind"] == "command_refresh"]
                self.assertEqual(len(refresh_events), 2)
                self.assertEqual(refresh_events[-1]["feedback_source_timestamp_s"], clock())

    def test_refresh_never_renews_original_watchdog_deadline(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        issued = clock()-.02
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1), issued_at_s=issued)
        for _ in range(5):
            clock.wait(.041)
            session.poll_refresh()
            self.assertEqual(session.last_command_at_s, issued)
        clock.wait(.026)
        with self.assertRaises(SDKWatchdogError):
            session.poll_refresh()
        self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))
        self.assertIsNone(session._command_lease)
        with self.assertRaises(SDKWatchdogError):
            session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1))

    def test_lease_expiry_stops_owned_family_without_latching_and_new_image_can_resume(self):
        for method, operation in (("ExecuteMove", "move"), ("ExecuteAttitude", "attitudeControl")):
            with self.subTest(method=method):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                original = clock()
                command = RobotMotionCommand(yaw_rate_rad_s=.1)
                getattr(session, method)(command, issued_at_s=original, valid_until_s=original+.08)
                clock.wait(.041)
                session.poll_refresh()
                clock.wait(.04)
                session.poll_refresh()
                self.assertEqual(sdk.calls[-1][1], operation)
                self.assertFalse(any(sdk.calls[-1][2]))
                self.assertIsNone(session.fault)
                self.assertIsNone(session._command_lease)
                expiration = [e for e in session.events if e["kind"] == "command_lease_expired"]
                self.assertEqual(len(expiration), 1)
                self.assertEqual(expiration[0]["original_issued_at_s"], original)
                zero = [e for e in session.events if e.get("safety_zero")][-1]
                self.assertFalse(any(zero["arguments"]))
                self.assertEqual(zero["original_issued_at_s"], original)
                before = list(sdk.calls)
                clock.wait(.041)
                session.poll_refresh()
                self.assertEqual(sdk.calls, before)
                getattr(session, method)(command, issued_at_s=clock(), valid_until_s=clock()+.1)
                self.assertEqual(session.last_command, command)
                self.assertIsNotNone(session._command_lease)

    def test_already_expired_image_lease_is_zero_without_entering_a_family(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        before = list(sdk.calls)
        self.assertEqual(session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.02),
                         issued_at_s=clock(), valid_until_s=clock()-.01), RobotMotionCommand())
        self.assertEqual(sdk.calls, before)
        self.assertEqual(session.mode, "standing")
        self.assertIsNone(session._command_lease)
        self.assertIsNone(session.fault)
        self.assertTrue(any(e["kind"] == "command_lease_expired" for e in session.events))
        # Even a WAIT created after its image expired is still a safety zero.
        session.ExecuteMove(RobotMotionCommand(), valid_until_s=clock()-.1)
        self.assertEqual(sdk.calls, before)
        self.assertIsNone(session.fault)

    def test_new_image_cannot_replace_an_expired_old_lease_without_zero_evidence(self):
        for method, operation, zero in (("ExecuteMove", "move", (0., 0., 0.)),
                                       ("ExecuteAttitude", "attitudeControl", (0., 0., 0., 0.))):
            with self.subTest(method=method):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                original = clock()
                getattr(session, method)(RobotMotionCommand(yaw_rate_rad_s=.1),
                                         valid_until_s=original+.08)
                clock.wait(.081)  # A new image arrives before any monitor poll.
                replacement = RobotMotionCommand(yaw_rate_rad_s=-.1)
                issued = clock()
                self.assertEqual(getattr(session, method)(replacement, issued_at_s=issued,
                                 valid_until_s=issued+.15), replacement)
                self.assertEqual(sdk.calls[-2][1:], (operation, zero))
                self.assertEqual(session.last_command, replacement)
                expiry = next(row for row in session.events if row["kind"] == "command_lease_expired")
                self.assertEqual(expiry["original_issued_at_s"], original)
                starts = [row for row in session.events if row["kind"] == "command_lease_started"]
                self.assertLess(session.events.index(expiry), session.events.index(starts[-1]))
                self.assertEqual(starts[-1]["original_issued_at_s"], issued)
                self.assertIsNone(session.fault)

    def test_slow_new_image_getter_cannot_erase_old_image_or_original_watchdog_deadline(self):
        for age, duration, watchdog in ((.041, .05, False), (.2, .06, True)):
            with self.subTest(watchdog=watchdog):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                original = clock()
                session.ExecuteAttitude(RobotMotionCommand(yaw_rate_rad_s=.1),
                                        valid_until_s=original+(1. if watchdog else .08))
                clock.wait(age)
                issued = clock()
                def slow_native_getter():
                    clock.wait(duration)
                    return sdk.rpy
                sdk.getRPY = slow_native_getter
                replacement = RobotMotionCommand(yaw_rate_rad_s=-.1)
                if watchdog:
                    with self.assertRaises(SDKWatchdogError):
                        session.ExecuteAttitude(replacement, issued_at_s=issued, valid_until_s=issued+.15)
                    self.assertIsNone(session._command_lease)
                    self.assertFalse(any(call[1] == "attitudeControl" and call[2][2] < 0 for call in sdk.calls))
                else:
                    self.assertEqual(session.ExecuteAttitude(replacement, issued_at_s=issued,
                                     valid_until_s=issued+.15), replacement)
                    self.assertEqual(sdk.calls[-2][1:], ("attitudeControl", (0., 0., 0., 0.)))
                    expiry = next(row for row in session.events if row["kind"] == "command_lease_expired")
                    self.assertEqual(expiry["original_issued_at_s"], original)
                    self.assertIsNone(session.fault)

    def test_final_lease_check_preserves_payload_lifetime_and_never_refreshes(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        self.assertFalse(session.check_current_lease())
        original = clock()
        until = original+.08
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.02), valid_until_s=until)
        lease = session._command_lease
        before = list(sdk.calls)
        clock.wait(.041)
        with session._lock:
            self.assertTrue(session.check_current_lease())
        self.assertEqual(sdk.calls, before)
        self.assertIs(session._command_lease, lease)
        self.assertEqual(session.last_command_at_s, original)
        clock.wait(until-clock())
        with session._lock:
            self.assertFalse(session.check_current_lease())
        self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0., 0., 0., 0.)))
        self.assertIsNone(session.fault)

    def test_final_locked_lease_check_catches_slow_sdk_getter_crossing_either_deadline(self):
        for duration, watchdog in ((.081, False), (.251, True)):
            with self.subTest(watchdog=watchdog):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                original = clock()
                session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.02),
                                        valid_until_s=original+(1. if watchdog else .08))
                def slow_native_getter():
                    clock.wait(duration)
                    return sdk.rpy
                sdk.getRPY = slow_native_getter
                with session._lock:  # Background monitor is excluded by the fire transaction.
                    self.assertTrue(session.ReadFeedback().valid)
                    if watchdog:
                        with self.assertRaises(SDKWatchdogError):
                            session.check_current_lease()
                    else:
                        self.assertFalse(session.check_current_lease())
                        self.assertIsNone(session.fault)
                self.assertIsNone(session._command_lease)
                self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0., 0., 0., 0.)))

    def test_final_lease_check_connection_read_cannot_hide_an_expired_deadline(self):
        for duration, watchdog in ((.081, False), (.251, True)):
            with self.subTest(watchdog=watchdog):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                original = clock()
                session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1),
                                    valid_until_s=original+(1. if watchdog else .08))
                def slow_connect_check():
                    clock.wait(duration)
                    return True
                sdk.checkConnect = slow_connect_check
                if watchdog:
                    with self.assertRaises(SDKWatchdogError):
                        session.check_current_lease()
                else:
                    self.assertFalse(session.check_current_lease())
                self.assertIsNone(session._command_lease)
                self.assertEqual(sdk.calls[-1][1:], ("move", (0., 0., 0.)))

    def test_refresh_expires_at_the_exact_image_deadline(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        until = clock()+.04
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.02), valid_until_s=until)
        clock.wait(until-clock())
        session.poll_refresh()
        self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0.0, 0.0, 0.0, 0.0)))
        self.assertFalse(any(e["kind"] == "command_refresh" for e in session.events))
        self.assertIsNone(session.fault)

    def test_refresh_stale_feedback_zeros_cancels_and_does_not_restamp_or_latch(self):
        for method, operation in (("ExecuteMove", "move"), ("ExecuteAttitude", "attitudeControl")):
            with self.subTest(method=method):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                command = RobotMotionCommand(yaw_rate_rad_s=.1)
                getattr(session, method)(command)
                acquired = clock.stamp()
                clock.frozen_stamp = acquired
                clock.wait(.16)
                session.poll_refresh()
                self.assertEqual(sdk.calls[-1][1], operation)
                self.assertFalse(any(sdk.calls[-1][2]))
                self.assertIsNone(session._command_lease)
                self.assertIsNone(session.fault)
                self.assertEqual(session.last_feedback_meta["source_timestamp_s"], acquired.source_timestamp_s)
                self.assertFalse(session.ReadFeedback().valid)
                before = list(sdk.calls)
                clock.wait(.041)
                session.poll_refresh()
                self.assertEqual(sdk.calls, before)
                self.assertEqual(len([e for e in session.events if e["kind"] == "command_refresh_cancelled"]), 1)
                clock.frozen_stamp = None
                getattr(session, method)(command, issued_at_s=clock(), valid_until_s=clock()+.1)
                self.assertIsNotNone(session._command_lease)

    def test_refresh_sdk_status_failure_getter_failure_and_disconnect_latch(self):
        for kind in ("status", "getter", "disconnect"):
            with self.subTest(failure=kind):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.01))
                clock.wait(.041)
                if kind == "status":
                    sdk.result["attitudeControl"] = 0x3013
                elif kind == "getter":
                    sdk.getRPY = lambda: (_ for _ in ()).throw(RuntimeError("native getter fixture failure"))
                else:
                    sdk.connected = False
                with self.assertRaises(OfficialSDKError):
                    session.poll_refresh()
                self.assertEqual(session.mode, "fault")
                self.assertIsNone(session._command_lease)
                self.assertEqual(sdk.calls[-1][1:], ("attitudeControl", (0.0, 0.0, 0.0, 0.0)))

    def test_refresh_rechecks_both_deadlines_after_a_slow_feedback_read(self):
        for duration, expected_fault in ((.05, False), (.26, True)):
            with self.subTest(duration=duration):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                start = clock()
                session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1), valid_until_s=start+.08)
                def slow_stamp():
                    clock.wait(duration)
                    return clock.stamp()
                session.freshness_provider = slow_stamp
                clock.wait(.041)
                if expected_fault:
                    with self.assertRaises(SDKWatchdogError):
                        session.poll_refresh()
                else:
                    session.poll_refresh()
                    self.assertIsNone(session.fault)
                self.assertIsNone(session._command_lease)
                self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, 0.0)))
                self.assertFalse(any(e.get("refresh") for e in session.events))

    def test_native_refresh_exception_is_recorded_as_a_real_sdk_call(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        original = clock()
        until = original+.15
        session.ExecuteMove(RobotMotionCommand(yaw_rate_rad_s=.1), valid_until_s=until)
        def failed_native(*args):
            sdk.calls.append((clock(), "move", args))
            raise RuntimeError("native SDK call fixture failure")
        sdk.move = failed_native
        clock.wait(.041)
        with self.assertRaisesRegex(OfficialSDKError, "native SDK call fixture failure"):
            session.poll_refresh()
        failures = [e for e in session.events if e["kind"] == "sdk_call_failed"]
        self.assertEqual(len(failures), 2)  # Failed repeat and failed matching zero.
        self.assertTrue(failures[0]["refresh"])
        self.assertTrue(failures[1]["safety_zero"])
        self.assertEqual(failures[0]["sdk_called_at_s"], clock())
        self.assertEqual(failures[0]["original_issued_at_s"], original)
        self.assertEqual(failures[0]["valid_until_s"], until)
        self.assertEqual(failures[1]["arguments"], [0, 0, 0])
        self.assertIsNone(session._command_lease)

    def test_stop_safety_zero_and_standup_all_cancel_refresh(self):
        for action in ("Stop", "zero", "StandUp"):
            with self.subTest(action=action):
                session, sdk, clock = self.session(refresh_interval_s=.04)
                session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.01), valid_until_s=clock()+.15)
                if action == "zero":
                    session.ExecuteMove(RobotMotionCommand())
                else:
                    getattr(session, action)()
                self.assertIsNone(session._command_lease)
                before = list(sdk.calls)
                clock.wait(.041)
                session.poll_refresh()
                self.assertEqual(sdk.calls, before)
                self.assertFalse(any(c[1] == "move" for c in sdk.calls))

    def test_background_watchdog_also_services_optional_refresh(self):
        session, sdk, clock = self.session(refresh_interval_s=.04)
        command = RobotMotionCommand(yaw_rate_rad_s=.1)
        session.ExecuteMove(command, valid_until_s=clock()+.15)
        clock.wait(.041)
        original = session.poll_refresh
        serviced = []
        def one_poll():
            original()
            serviced.append(True)
            session._watchdog_stop.set()
        session.poll_refresh = one_poll
        session.start_watchdog()
        session._watchdog_thread.join(timeout=1)
        self.assertFalse(session._watchdog_thread.is_alive())
        self.assertEqual(serviced, [True])
        self.assertEqual(sdk.calls[-1][1:], ("move", (0.0, 0.0, .1)))
        self.assertTrue(any(e["kind"] == "command_refresh" for e in session.events))
        session.close()

    def test_standup_delay_cannot_be_relaxed_below_official_demo(self):
        with self.assertRaises(ValueError):
            OfficialSDKSession(stand_up_min_s=3.9)


if __name__ == "__main__":
    unittest.main()
