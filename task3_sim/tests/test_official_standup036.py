"""Checked stand-up request/ack regression using injected SDK contracts only."""

from collections import deque
import io
import json
import math
from types import SimpleNamespace
import threading
import unittest

from task3_sim.interfaces import RobotMotionCommand
from task3_sim.official_backend import OfficialDynamicsRobot
from task3_sim.official_sdk import OfficialSDKError, OfficialSDKSession
from task3_sim.tests.test_official_sdk import FakeClock, FakeHighLevel


class CounterAwareSDK(FakeHighLevel):
    """Model async ctrlmode feedback and the SDK's same-state counter reset.

    A successful request is not arrival. Only a later standUp call made while
    actual ctrlmode is 1 clears the stand-up attempt counter. This injected
    contract is deliberately stricter than the old immediate-success fake;
    it cannot establish native or physical-robot acceptance.
    """

    def __init__(self, clock, *, arrival_delay_s=.05):
        super().__init__(clock)
        self.arrival_delay_s = arrival_delay_s
        self.arrival_at_s = None
        self.never_arrive = False
        self.request_status = self.ack_status = 0
        self.request_count = self.ack_count = self.transition_count = 0
        self.largest_transition_count = 0
        self.disconnect_at_s = None
        self.after_request = None

    def _arrival(self):
        if (not self.never_arrive and self.arrival_at_s is not None
                and self.clock() >= self.arrival_at_s):
            self.ctrlmode = 1
            self.arrival_at_s = None

    def checkConnect(self):
        return self.connected and (self.disconnect_at_s is None or self.clock() < self.disconnect_at_s)

    def getCurrentCtrlmode(self):
        self._arrival()
        return self.ctrlmode

    def standUp(self):
        self._arrival()
        self.calls.append((self.clock(), "standUp", ()))
        if self.ctrlmode == 1:
            self.ack_count += 1
            if not self.ack_status:
                self.transition_count = 0
            return self.ack_status
        self.request_count += 1
        self.transition_count += 1
        self.largest_transition_count = max(self.largest_transition_count, self.transition_count)
        result = self.request_status or (0x3007 if self.transition_count > 10 else 0)
        if result == 0:
            self.arrival_at_s = self.clock()+self.arrival_delay_s
            if self.after_request is not None:
                self.after_request()
        return result


class OfficialStandUp036Tests(unittest.TestCase):
    def session(self, *, stand=False, arrival_delay_s=.05, **kwargs):
        clock = FakeClock()
        sdk = CounterAwareSDK(clock, arrival_delay_s=arrival_delay_s)
        options = dict(sdk=sdk, clock=clock, wait=clock.wait, freshness_provider=clock.stamp,
                       command_timeout_s=.5, background_watchdog=False)
        options.update(kwargs)
        session = OfficialSDKSession(**options)
        session.Connect()
        if stand:
            session.StandUp()
        return session, sdk, clock

    def test_fifteen_alternating_modes_ack_each_arrival_and_require_new_image_command(self):
        session, sdk, clock = self.session(stand=True)
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.01),
                                issued_at_s=clock(), valid_until_s=clock()+.15)
        for index in range(15):
            method = session.ExecuteMove if index % 2 == 0 else session.ExecuteAttitude
            command = (RobotMotionCommand(yaw_rate_rad_s=-.2) if index % 2 == 0
                       else RobotMotionCommand(yaw_rate_rad_s=.01, pitch_rate_rad_s=-.01))
            old_image_at = clock()
            old_image_deadline = old_image_at+.15
            before = len(sdk.calls)
            applied = method(command, issued_at_s=old_image_at, valid_until_s=old_image_deadline)
            self.assertEqual(applied, RobotMotionCommand())
            self.assertGreater(clock(), old_image_deadline)
            self.assertEqual(session.mode, "standing")
            self.assertTrue(session.standing)
            actions = sdk.calls[before:]
            self.assertEqual(sum(name == "standUp" for _, name, _ in actions), 2)
            self.assertFalse(any(name in ("move", "attitudeControl") and any(arguments)
                                 for _, name, arguments in actions))
            # Only a newly acquired observation authorizes the next input.
            clock.wait(.025)
            issued_at = clock()
            self.assertEqual(method(command, issued_at_s=issued_at, valid_until_s=issued_at+.15), command)
        self.assertEqual(sdk.request_count, 16)
        self.assertEqual(sdk.ack_count, 16)
        self.assertEqual(sdk.transition_count, 0)
        self.assertEqual(sdk.largest_transition_count, 1)
        self.assertFalse(any(row.get("kind") == "fault" for row in session.events))
        self.assertFalse(any(row.get("kind") == "sdk_result" and row.get("code") != 0 for row in session.events))

    def test_standup_ack_is_after_real_arrival_and_full_four_second_wait_follows_ack(self):
        session, sdk, clock = self.session(arrival_delay_s=.15)
        session.StandUp()
        calls = [row for row in sdk.calls if row[1] == "standUp"]
        self.assertEqual(len(calls), 2)
        requested_at, acknowledged_at = calls[0][0], calls[1][0]
        self.assertGreaterEqual(acknowledged_at-requested_at, sdk.arrival_delay_s+session.stationary_duration_s-1e-8)
        self.assertGreaterEqual(clock()-acknowledged_at, 4.0-1e-8)
        requested = [row for row in session.events if row.get("kind") == "standup_requested"]
        acknowledged = [row for row in session.events if row.get("kind") == "standup_acknowledged"]
        self.assertEqual(len(requested), 1)
        self.assertEqual(len(acknowledged), 1)
        self.assertEqual(acknowledged[0].get("control_mode"), 1)
        # The request records the operation; acknowledgement additionally
        # records the acquired feedback that earned the in-mode SDK call.
        row = acknowledged[0]
        stamp = row.get("source_timestamp_s", row.get("source_timestamp"))
        self.assertIsInstance(stamp, (int, float))
        self.assertTrue(math.isfinite(stamp))
        self.assertLessEqual(stamp, row["read_at_s"])

    def test_expired_pretransition_command_cannot_be_replayed_after_standing(self):
        session, sdk, clock = self.session(stand=True)
        session.ExecuteAttitude(RobotMotionCommand(pitch_rate_rad_s=.01))
        old_image = clock()
        command = RobotMotionCommand(yaw_rate_rad_s=.2)
        self.assertEqual(session.ExecuteMove(command, issued_at_s=old_image,
                                             valid_until_s=old_image+.15), RobotMotionCommand())
        before = len(sdk.calls)
        with self.assertRaises(OfficialSDKError):
            session.ExecuteMove(command, issued_at_s=old_image, valid_until_s=old_image+.15)
        self.assertEqual(session.mode, "fault")
        self.assertFalse(any(name == "move" and any(args) for _, name, args in sdk.calls[before:]))

    def test_request_and_ack_errors_are_never_ignored_or_retried_as_success(self):
        for phase in ("request", "ack"):
            for code in (0x3007, 0x3012):
                with self.subTest(phase=phase, code=code):
                    session, sdk, _ = self.session()
                    setattr(sdk, phase+"_status", code)
                    with self.assertRaisesRegex(OfficialSDKError, f"0x{code:04x}"):
                        session.StandUp()
                    self.assertEqual(session.mode, "fault")
                    self.assertFalse(session.standing)
                    self.assertEqual(sdk.request_count, 1)
                    self.assertEqual(sdk.ack_count, int(phase == "ack"))
                    self.assertEqual(sum(row.get("kind") == "sdk_result" and row.get("operation") == "standUp"
                                         and row.get("code") == code for row in session.events), 1)
                    self.assertFalse(any(name in ("move", "attitudeControl") and any(args) for _, name, args in sdk.calls))

    def test_missing_actual_stand_mode_cannot_be_acknowledged(self):
        session, sdk, clock = self.session(transition_timeout_s=4.5)
        sdk.never_arrive = True
        started = clock()
        with self.assertRaises(OfficialSDKError):
            session.StandUp()
        self.assertEqual(sdk.request_count, 1)
        self.assertEqual(sdk.ack_count, 0)
        self.assertLessEqual(clock()-started, session.transition_timeout_s+session.poll_s+1e-8)
        self.assertFalse(session.standing)

    def test_frozen_or_stale_source_after_request_cannot_earn_ack_or_standup(self):
        for kind in ("frozen", "stale"):
            with self.subTest(kind=kind):
                session, sdk, clock = self.session(transition_timeout_s=4.5)
                if kind == "frozen":
                    sdk.after_request = lambda: setattr(clock, "frozen_stamp", clock.stamp())
                else:
                    previous = session.freshness_provider
                    def source():
                        stamp = previous()
                        return type(stamp)(stamp.sequence, stamp.source_timestamp_s-1., stamp.received_at_s) if sdk.request_count else stamp
                    session.freshness_provider = source
                with self.assertRaises(OfficialSDKError):
                    session.StandUp()
                self.assertEqual(sdk.request_count, 1)
                self.assertEqual(sdk.ack_count, 0)
                self.assertFalse(session.standing)
                self.assertEqual(session.mode, "fault")

    def test_disconnect_between_request_and_arrival_dwell_never_sends_ack(self):
        session, sdk, _ = self.session()
        sdk.disconnect_at_s = .35
        with self.assertRaisesRegex(OfficialSDKError, "disconnect|connection"):
            session.StandUp()
        self.assertEqual(sdk.request_count, 1)
        self.assertEqual(sdk.ack_count, 0)
        self.assertFalse(session.standing)

    def test_slow_control_mode_getter_cannot_qualify_expired_stationary_feedback(self):
        session, sdk, clock = self.session(transition_timeout_s=4.5)
        original = sdk.getCurrentCtrlmode
        def slow_control_mode():
            self.assertFalse(session.standing)
            clock.wait(.2)
            return original()
        sdk.getCurrentCtrlmode = slow_control_mode
        with self.assertRaisesRegex(OfficialSDKError, "Timed out waiting for fresh stationary"):
            session.StandUp()
        self.assertFalse(any(row.get("kind") == "stationary_confirmed" for row in session.events))
        self.assertEqual(sdk.request_count, 0)
        self.assertEqual(sdk.ack_count, 0)
        self.assertFalse(session.standing)
        self.assertEqual(session.mode, "fault")

    def test_slow_final_mode_getter_invalidates_feedback_before_acknowledgement(self):
        session, sdk, clock = self.session()
        original = sdk.getCurrentCtrlmode
        delays = []
        def slow_ack_mode():
            self.assertFalse(session.standing)
            confirmed = sum(row.get("kind") == "stationary_confirmed" for row in session.events)
            # One confirmation precedes request; the second earns arrival.
            # Only the additional getter immediately before ack is slow.
            if confirmed == 2:
                delays.append(clock())
                clock.wait(.2)
            return original()
        sdk.getCurrentCtrlmode = slow_ack_mode
        with self.assertRaisesRegex(OfficialSDKError, "Standing feedback expired"):
            session.StandUp()
        self.assertEqual(len(delays), 1)
        self.assertEqual(sum(row.get("kind") == "stationary_confirmed" for row in session.events), 2)
        self.assertEqual(sdk.request_count, 1)
        self.assertEqual(sdk.ack_count, 0)
        self.assertFalse(any(row.get("kind") == "standup_acknowledged" for row in session.events))
        self.assertFalse(session.standing)
        self.assertEqual(session.mode, "fault")

    def test_all_standup_stages_share_one_deadline(self):
        # Each stage separately fits 4.5 s. Their sum does not: the ack's full
        # four-second confirmation must not silently start a second timeout.
        session, sdk, clock = self.session(transition_timeout_s=4.5, arrival_delay_s=.35)
        started = clock()
        with self.assertRaises(OfficialSDKError):
            session.StandUp()
        self.assertEqual(sdk.request_count, 1)
        self.assertEqual(sdk.ack_count, 1)
        self.assertLessEqual(clock()-started, session.transition_timeout_s+session.poll_s+1e-8)
        self.assertFalse(session.standing)
        self.assertEqual(session.mode, "fault")


class OfficialTraceClock036Tests(unittest.TestCase):
    def test_recorded_task_time_and_age_share_exactly_one_clock_sample(self):
        robot = OfficialDynamicsRobot.__new__(OfficialDynamicsRobot)
        calls = []
        def now():
            value = 10.0+len(calls)*.000013
            calls.append(value)
            return value
        robot.receiver = SimpleNamespace(now=now)
        robot._trace_lock = threading.RLock()
        robot._trace = io.StringIO()
        robot._last_trace_seq = None
        robot.physics_samples = 0
        robot._stationary_samples = deque()
        packet = {"fixture_only": True}
        sample = SimpleNamespace(sequence=1, published_at_s=9.99, packet=packet)
        robot._record_sample(sample)
        record = json.loads(robot._trace.getvalue())
        self.assertEqual(calls, [10.0])
        self.assertEqual(record["publication_age_s"], record["task_time_s"]-record["published_at_s"])
        self.assertEqual(sample.published_at_s, 9.99)
        self.assertIs(sample.packet, packet)
        self.assertEqual(robot.physics_samples, 1)
        robot._record_sample(sample)
        self.assertEqual(calls, [10.0])
        self.assertEqual(robot.physics_samples, 1)


if __name__ == "__main__":
    unittest.main()
