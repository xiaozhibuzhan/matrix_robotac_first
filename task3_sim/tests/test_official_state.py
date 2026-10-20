"""Packet/clock contract tests; these do not run the official Linux gait."""
import copy
import json
import socket
import unittest

import numpy as np

from task3_sim.official_state import (decode_sample, sample_feedback,
                                      OfficialStateReceiver, StalePhysicsSample)


def packet(seq=1, mono=100.0, age=0.01):
    return {"schema": "task3.official_state.v1", "bridge_seq": seq,
            "source_ecal_clock": seq*5, "source_ecal_id": -1, "source_time_stamp_raw": 0,
            "source_ecal_time_us": 5_000_000+seq*10_000,
            "received_ecal_time_us": 5_000_000+seq*10_000+round(age*1e6),
            "received_monotonic_ns": round(mono*1e9), "q": [0, .9, -1.8]*4,
            "qd": [0.]*12, "tau": [0.]*12, "quat_wxyz": [1., 0., 0., 0.],
            "gyro_body": [0.]*3, "acc_body": [0., 0., 9.81],
            "imu_position_world": [0., 0., .27], "velocity_world": [0.]*3}


def timed_packet(seq=1, mono=100.0, age=0.01):
    """Keep the bridge's eCAL and monotonic receive clocks progressing together."""
    item = packet(seq, mono, age)
    received = 5_000_000 + round((mono-100.0)*1e6)
    item["received_ecal_time_us"] = received
    item["source_ecal_time_us"] = received-round(age*1e6)
    return item


class OfficialStateTests(unittest.TestCase):
    def test_source_publication_time_uses_ecal_delta_not_wall_epoch_or_raw(self):
        item = packet()
        sample = decode_sample(item, origin_s=90, now_s=10.02)
        self.assertAlmostEqual(sample.published_at_s, 9.99)
        self.assertEqual(sample.packet["source_time_stamp_raw"], 0)
        self.assertTrue(sample_feedback(sample, 10.02).valid)
        self.assertFalse(sample_feedback(sample, 10.2).valid)

    def test_repeated_or_reversed_packets_cannot_refresh_feedback(self):
        first = decode_sample(packet(), origin_s=90, now_s=10.02)
        with self.assertRaises(ValueError):
            decode_sample(packet(), origin_s=90, now_s=10.03, previous=first)
        later = decode_sample(packet(2, 100.02), origin_s=90, now_s=10.04, previous=first)
        self.assertGreater(later.published_at_s, first.published_at_s)

    def test_clock_reversal_future_or_stale_rejected(self):
        for item in (packet(age=-0.001), packet(age=.2), packet(mono=101), packet(mono=99)):
            with self.subTest(item=item):
                with self.assertRaises(ValueError):
                    decode_sample(item, origin_s=90, now_s=10.02)

    def test_bad_joint_or_quaternion_packet_rejected(self):
        for key, value in (("q", [0]*11), ("qd", [float("nan")]*12),
                           ("quat_wxyz", [2, 0, 0, 0]), ("gyro_body", [0, 0])):
            item = packet()
            item[key] = value
            with self.subTest(key=key):
                with self.assertRaises(ValueError):
                    decode_sample(item, origin_s=90, now_s=10.02)

    def test_quaternion_feedback_and_raw_joint_order(self):
        item = packet()
        item["quat_wxyz"] = [np.cos(.4), 0, 0, np.sin(.4)]
        sample = decode_sample(item, origin_s=90, now_s=10.02)
        feedback = sample_feedback(sample, 10.02)
        self.assertAlmostEqual(feedback.rpy_rad[2], .8)
        np.testing.assert_equal(feedback.position_m, [0, 0, .27])
        self.assertEqual(sample.packet["q"], [0, .9, -1.8]*4)

    def test_loopback_receiver_drops_old_queue_and_latches_bad_fresh_packet(self):
        clock = [100.0]
        receiver = OfficialStateReceiver(origin_s=90, clock=lambda: clock[0])
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(json.dumps(packet(mono=99)).encode(), ("127.0.0.1", receiver.port))
            sender.sendto(json.dumps(packet(2, 100)).encode(), ("127.0.0.1", receiver.port))
            self.assertEqual(receiver.latest().sequence, 2)
            self.assertEqual(receiver.stale_dropped, 1)
            clock[0] += .2
            self.assertFalse(receiver.feedback().valid)
            malformed = packet(3, 100.2)
            malformed["q"] = []
            sender.sendto(json.dumps(malformed).encode(), ("127.0.0.1", receiver.port))
            with self.assertRaises(RuntimeError):
                receiver.latest()
            with self.assertRaises(RuntimeError):
                receiver.latest()
        finally:
            sender.close()
            receiver.close()

    def test_source_transit_backlog_is_dropped_then_fresh_packet_recovers(self):
        clock = [100.02]
        receiver = OfficialStateReceiver(origin_s=90, clock=lambda: clock[0])
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            stale = timed_packet(1, 100.0, age=.2)
            fresh = timed_packet(2, 100.01, age=.01)
            for item in (stale, fresh):
                sender.sendto(json.dumps(item).encode(), ("127.0.0.1", receiver.port))
            sample = receiver.latest()
            self.assertEqual(sample.sequence, 2)
            self.assertIsNone(receiver.error)
            self.assertEqual(receiver.accepted, 1)
            self.assertEqual(receiver.stale_drop_reasons, {"source_transit": 1})
            self.assertAlmostEqual(receiver.last_stale_drop["published_at_s"], 9.8)
            self.assertAlmostEqual(sample.published_at_s, 10.0)
            self.assertEqual(sample.packet, fresh)
            self.assertTrue(receiver.feedback().valid)
        finally:
            sender.close()
            receiver.close()

    def test_stale_drop_reasons_account_for_both_transport_stages(self):
        cases = ((.01, .20, "receiver_queue"),
                 (.10, .06, "combined_transit_and_queue"),
                 (.20, .01, "source_transit"))
        for transit, queue, reason in cases:
            with self.subTest(reason=reason):
                receiver = OfficialStateReceiver(origin_s=90, clock=lambda: 100+queue)
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sender.sendto(json.dumps(timed_packet(age=transit)).encode(),
                                  ("127.0.0.1", receiver.port))
                    self.assertIsNone(receiver.latest())
                    self.assertEqual(receiver.stale_drop_reasons, {reason: 1})
                    self.assertIsNone(receiver.error)
                    self.assertFalse(receiver.feedback().valid)
                    self.assertEqual(receiver.accepted, 0)
                finally:
                    sender.close()
                    receiver.close()

    def test_stale_drop_cannot_refresh_previously_accepted_feedback(self):
        clock = [100.0]
        receiver = OfficialStateReceiver(origin_s=90, clock=lambda: clock[0])
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(json.dumps(timed_packet()).encode(), ("127.0.0.1", receiver.port))
            first = receiver.latest()
            clock[0] = 100.3
            sender.sendto(json.dumps(timed_packet(2, 100.25, age=.2)).encode(),
                          ("127.0.0.1", receiver.port))
            self.assertIs(receiver.latest(), first)
            self.assertFalse(receiver.feedback().valid)
            self.assertEqual(receiver.accepted, 1)
            self.assertEqual(receiver.stale_dropped, 1)
            self.assertEqual(first.packet["received_monotonic_ns"], 100_000_000_000)
        finally:
            sender.close()
            receiver.close()

    def test_age_boundary_uses_total_source_age_without_relaxing_limit(self):
        for queue, stale in ((.049999, False), (.050001, True)):
            with self.subTest(queue=queue):
                if stale:
                    with self.assertRaises(StalePhysicsSample):
                        decode_sample(timed_packet(age=.1), origin_s=90,
                                      now_s=10+queue, max_age_s=.15)
                else:
                    sample = decode_sample(timed_packet(age=.1), origin_s=90,
                                           now_s=10+queue, max_age_s=.15)
                    self.assertTrue(sample_feedback(sample, 10+queue, .15).valid)

    def test_malformed_stale_packets_latch_fault_instead_of_hiding_in_backlog(self):
        for key, value in (("q", []), ("qd", [float("nan")]*12),
                           ("quat_wxyz", [2., 0., 0., 0.]),
                           ("bridge_seq", False), ("source_ecal_id", "-1")):
            with self.subTest(key=key):
                receiver = OfficialStateReceiver(origin_s=90, clock=lambda: 100.3)
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    item = timed_packet(age=.2)
                    item[key] = value
                    sender.sendto(json.dumps(item).encode(), ("127.0.0.1", receiver.port))
                    with self.assertRaises(RuntimeError):
                        receiver.latest()
                    self.assertEqual(receiver.stale_dropped, 0)
                    with self.assertRaises(RuntimeError):
                        receiver.latest()
                finally:
                    sender.close()
                    receiver.close()

    def test_source_reversal_is_fatal_even_when_packet_is_stale(self):
        receiver = OfficialStateReceiver(origin_s=90, clock=lambda: 100.1)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(json.dumps(timed_packet(2, 100.1)).encode(),
                          ("127.0.0.1", receiver.port))
            self.assertEqual(receiver.latest().sequence, 2)
            sender.sendto(json.dumps(timed_packet(1, 99.8)).encode(),
                          ("127.0.0.1", receiver.port))
            with self.assertRaisesRegex(RuntimeError, "repeated or regressed"):
                receiver.latest()
            self.assertEqual(receiver.stale_dropped, 0)
        finally:
            sender.close()
            receiver.close()

    def test_stale_samples_preserve_ordering_before_recovery(self):
        receiver = OfficialStateReceiver(origin_s=90, clock=lambda: 100.1)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(json.dumps(timed_packet(2, 100.0, age=.2)).encode(),
                          ("127.0.0.1", receiver.port))
            self.assertIsNone(receiver.latest())
            sender.sendto(json.dumps(timed_packet(1, 100.05)).encode(),
                          ("127.0.0.1", receiver.port))
            with self.assertRaisesRegex(RuntimeError, "repeated or regressed"):
                receiver.latest()
            self.assertEqual(receiver.stale_dropped, 1)
        finally:
            sender.close()
            receiver.close()

    def test_future_and_discontinuous_clocks_latch_fault(self):
        for item in (timed_packet(age=-.001), timed_packet(mono=100.01)):
            with self.subTest(item=item):
                receiver = OfficialStateReceiver(origin_s=90, clock=lambda: 100.0)
                sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    sender.sendto(json.dumps(item).encode(), ("127.0.0.1", receiver.port))
                    with self.assertRaisesRegex(RuntimeError, "future or uses a discontinuous clock"):
                        receiver.latest()
                    self.assertEqual(receiver.stale_dropped, 0)
                finally:
                    sender.close()
                    receiver.close()

    def test_clock_reversal_cannot_hide_behind_increasing_source_sequence(self):
        first = decode_sample(timed_packet(), origin_s=90, now_s=10.02)
        for key in ("received_monotonic_ns", "source_ecal_time_us", "received_ecal_time_us"):
            with self.subTest(key=key):
                item = timed_packet(2, 100.02)
                item[key] = first.packet[key]-1
                with self.assertRaises(ValueError):
                    decode_sample(item, origin_s=90, now_s=10.03, previous=first)


if __name__ == "__main__":
    unittest.main()
