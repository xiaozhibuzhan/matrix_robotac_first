"""Check collection scheduling and process GC state without wall-clock waits."""

import gc
from pathlib import Path
import sys
import unittest
from unittest import mock
import weakref

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_gc import ScheduledGC


class ScheduledGCTests(unittest.TestCase):
    def setUp(self):
        self.original_enabled = gc.isenabled()
        self.addCleanup(gc.enable if self.original_enabled else gc.disable)

    def test_due_generations_have_distinct_periods_and_collect_lower_generations(self):
        with mock.patch("runtime_gc.time.monotonic", return_value=100.0) as clock, \
                mock.patch("runtime_gc.gc.collect") as collect:
            scheduler = ScheduledGC()
            self.addCleanup(scheduler.close)
            for elapsed in (0.0, 0.999):
                clock.return_value = 100.0 + elapsed
                scheduler.tick()
            collect.assert_not_called()
            clock.return_value = 101.0
            scheduler.tick()
            collect.assert_called_once_with(0)
            clock.return_value = 115.0
            scheduler.tick()
            self.assertEqual(collect.call_args_list, [mock.call(0), mock.call(1)])
            self.assertEqual(scheduler.last, [115.0, 115.0, 100.0])
            clock.return_value = 115.999
            scheduler.tick()
            self.assertEqual(collect.call_count, 2)
            clock.return_value = 220.0
            scheduler.tick()
            self.assertEqual(collect.call_args_list, [mock.call(0), mock.call(1), mock.call(2)])
            self.assertEqual(scheduler.last, [220.0, 220.0, 220.0])
            self.assertEqual(scheduler.collections, [1, 1, 1])
            self.assertEqual(scheduler.last_generation, 2)

    def test_real_cycle_is_reclaimed_by_due_collection(self):
        class CyclicObject:
            pass

        with mock.patch("runtime_gc.time.monotonic", return_value=0.0) as clock:
            scheduler = ScheduledGC()
            self.addCleanup(scheduler.close)
            value = CyclicObject()
            value.cycle = value
            reference = weakref.ref(value)
            del value
            self.assertIsNotNone(reference())
            clock.return_value = 120.0
            scheduler.tick()
            self.assertIsNone(reference())

    def test_close_restores_original_enabled_state_and_is_idempotent(self):
        for initially_enabled in (True, False):
            with self.subTest(initially_enabled=initially_enabled):
                (gc.enable if initially_enabled else gc.disable)()
                scheduler = ScheduledGC()
                self.assertFalse(gc.isenabled())
                scheduler.close()
                self.assertEqual(gc.isenabled(), initially_enabled)
                with mock.patch("runtime_gc.gc.enable") as enable, \
                        mock.patch("runtime_gc.gc.disable") as disable, \
                        mock.patch("runtime_gc.gc.collect") as collect:
                    scheduler.close()
                    scheduler.tick()
                    enable.assert_not_called()
                    disable.assert_not_called()
                    collect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
