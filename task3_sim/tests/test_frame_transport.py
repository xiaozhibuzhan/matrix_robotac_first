"""Delivery effects must never manufacture fresh camera observations."""

from __future__ import annotations

import math
import unittest

import numpy as np

from task3_sim.interfaces import Frame
from task3_sim.transport import FrameTransport


def frame(timestamp):
    return Frame(np.full((3, 4, 3), int(timestamp*10) % 256, dtype=np.uint8), timestamp)


class FrameTransportTests(unittest.TestCase):
    def test_passthrough_preserves_object_and_does_not_repeat(self):
        transport = FrameTransport()
        original = frame(0.5)
        transport.push(original, 0.5)
        self.assertIs(transport.poll(0.5), original)
        self.assertIsNone(transport.poll(0.6))
        self.assertEqual(original.timestamp, 0.5)
        self.assertEqual(transport.stats["delivered"], 1)

    def test_delayed_observation_keeps_actual_acquisition_timestamp(self):
        transport = FrameTransport({"delay_s": 0.25})
        original = frame(1.0)
        transport.push(original, 1.0)
        self.assertIsNone(transport.poll(1.24))
        delivered = transport.poll(1.25)
        self.assertIs(delivered, original)
        self.assertGreater(1.25-delivered.timestamp, 0.15)

    def test_freshest_eligible_wins_and_backlog_is_not_replayed(self):
        transport = FrameTransport({"delay_s": 0.1})
        for timestamp in (0.0, 0.05, 0.1):
            transport.push(frame(timestamp), timestamp)
        self.assertEqual(transport.poll(0.2).timestamp, 0.1)
        self.assertIsNone(transport.poll(0.3))
        self.assertEqual(transport.pending_count, 0)
        self.assertEqual(transport.stats["dropped_obsolete"], 2)

    def test_jitter_can_reorder_arrivals_without_reversing_acquisition(self):
        transport = FrameTransport({"delay_s": 0.2, "jitter_s": 0.2, "seed": 9})
        # Fixed seed makes the 0.05 acquisition arrive before the older 0.0 one.
        transport.push(frame(0.0), 0.0)
        transport.push(frame(0.05), 0.05)
        fresh = transport.poll(0.25)
        self.assertIsNotNone(fresh)
        self.assertEqual(fresh.timestamp, 0.05)
        self.assertIsNone(transport.poll(1.0))
        self.assertEqual(transport.stats["dropped_obsolete"], 1)

    def test_seed_reproduces_loss_and_delivery(self):
        def exercise():
            transport = FrameTransport({"delay_s": 0.04, "jitter_s": 0.03,
                                        "drop_probability": 0.3, "seed": 20261019})
            delivered = []
            for index in range(100):
                now = index*0.05
                transport.push(frame(now), now)
                result = transport.poll(now)
                if result is not None:
                    delivered.append(result.timestamp)
            return delivered, transport.stats
        first = exercise()
        self.assertEqual(first, exercise())
        self.assertGreater(first[1]["dropped_probability"], 0)
        self.assertGreater(first[1]["delivered"], 0)
        self.assertEqual(first[0], sorted(set(first[0])))

    def test_blackout_drops_new_acquisitions_at_half_open_boundaries(self):
        transport = FrameTransport({"blackout_intervals_s": [[0.1, 0.3]]})
        for timestamp in (0.05, 0.1, 0.2, 0.3):
            transport.push(frame(timestamp), timestamp)
            result = transport.poll(timestamp)
            self.assertEqual(result is not None, timestamp in (0.05, 0.3))
        self.assertEqual(transport.stats["dropped_blackout"], 2)

    def test_periodic_and_total_drops_do_not_supply_placeholder_frames(self):
        periodic = FrameTransport({"drop_every_n": 2})
        for index in range(4):
            periodic.push(frame(index), index)
            self.assertEqual(periodic.poll(index) is None, index % 2 == 1)
        self.assertEqual(periodic.stats["dropped_periodic"], 2)
        total = FrameTransport({"drop_probability": 1.0})
        total.push(frame(0.0), 0.0)
        self.assertIsNone(total.poll(100.0))

    def test_bounded_queue_and_accounting(self):
        transport = FrameTransport({"delay_s": 10, "max_queue": 3})
        for index in range(10):
            transport.push(frame(index*0.1), index*0.1)
        self.assertEqual(transport.pending_count, 3)
        self.assertEqual(transport.stats["max_queue_observed"], 3)
        self.assertEqual(transport.stats["dropped_overflow"], 7)
        self.assertEqual(transport.poll(20.0).timestamp, 0.9)
        stats = transport.stats
        self.assertEqual(stats["pushed"], stats["delivered"] + sum(value for key, value in stats.items()
                         if key.startswith("dropped_")))

    def test_old_duplicate_acquisitions_cannot_become_fresh(self):
        transport = FrameTransport()
        original = frame(0.5)
        transport.push(original, 0.5)
        transport.push(original, 0.5)
        self.assertIs(transport.poll(0.5), original)
        transport.push(original, 1.0)
        self.assertIsNone(transport.poll(1.0))
        self.assertEqual(transport.stats["delivered"], 1)
        self.assertEqual(transport.stats["dropped_obsolete"], 2)

    def test_invalid_config_and_nonfinite_clock_rejected(self):
        invalid = ({"delay_s": -1}, {"delay_s": math.nan}, {"jitter_s": math.inf},
                   {"drop_probability": 1.1}, {"drop_probability": True},
                   {"drop_every_n": 1.1}, {"drop_every_n": -1}, {"max_queue": 0},
                   {"seed": True}, {"seed": 2**63}, {"unknown": 1},
                   {"blackout_intervals_s": [[1, 0]]},
                   {"blackout_intervals_s": [[0, math.inf]]},
                   {"blackout_intervals_s": "0,1"})
        for settings in invalid:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                FrameTransport(settings)
        transport = FrameTransport()
        with self.assertRaises(ValueError):
            transport.poll(math.nan)
        with self.assertRaises(ValueError):
            transport.push(frame(1.0), 0.0)
        with self.assertRaises(TypeError):
            transport.push(object(), 0.0)
        transport.poll(2.0)
        with self.assertRaises(ValueError):
            transport.poll(1.0)


if __name__ == "__main__":
    unittest.main()
