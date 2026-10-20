"""Observe result holds and native teardown without opening desktop windows."""

import unittest
import weakref

from task3_sim.ui import InteractiveSession, WINDOW_NAME, close_viewer


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeOpenCV:
    WND_PROP_VISIBLE = 4

    def __init__(self, keys=(), close_after=None):
        self.keys = list(keys)
        self.close_after = close_after
        self.frames = []
        self.wait_count = 0
        self.destroyed = False

    def imshow(self, name, frame):
        assert name == WINDOW_NAME
        self.frames.append(frame)

    def waitKey(self, delay):
        self.wait_count += 1
        return self.keys.pop(0) if self.keys else -1

    def getWindowProperty(self, name, property_id):
        return 0 if self.close_after is not None and self.wait_count >= self.close_after else 1

    def destroyWindow(self, name):
        assert name == WINDOW_NAME
        self.destroyed = True


class FakeViewer:
    def __init__(self, close_after=None):
        self.sync_count = 0
        self.close_after = close_after
        self.closed = False

    def is_running(self):
        return not self.closed and (self.close_after is None or self.sync_count < self.close_after)

    def sync(self):
        self.sync_count += 1

    def close(self):
        self.closed = True


class UILifecycleTests(unittest.TestCase):
    def session(self, **kwargs):
        clock = FakeClock()
        return InteractiveSession(clock=clock, sleep=clock.sleep, **kwargs), clock

    def test_default_hold_waits_for_user_key_and_keeps_same_frame(self):
        cv2 = FakeOpenCV(keys=[-1] * 150 + [ord("q")])
        ui, clock = self.session(show=True, cv2_module=cv2)
        frozen_frame = object()
        self.assertEqual(ui.hold_result(frozen_frame), "key")
        self.assertGreaterEqual(clock.now, 2.9)
        self.assertEqual(len(cv2.frames), 151)
        self.assertTrue(all(frame is frozen_frame for frame in cv2.frames))

    def test_explicit_duration_expires_and_zero_skips_window_updates(self):
        viewer = FakeViewer()
        ui, clock = self.session(viewer=viewer)
        self.assertEqual(ui.hold_result(hold_seconds=0), "skipped")
        self.assertEqual(viewer.sync_count, 0)
        self.assertEqual(ui.hold_result(hold_seconds=0.1), "elapsed")
        self.assertAlmostEqual(clock.now, 0.1)
        self.assertGreater(viewer.sync_count, 0)

    def test_closing_opencv_window_does_not_recreate_it(self):
        cv2 = FakeOpenCV(close_after=2)
        ui, clock = self.session(show=True, cv2_module=cv2)
        self.assertEqual(ui.hold_result(object()), "window_closed")
        frame_count = len(cv2.frames)
        self.assertFalse(ui.present(object()))
        self.assertEqual(len(cv2.frames), frame_count)
        ui.close_windows()
        self.assertTrue(cv2.destroyed)

    def test_viewer_only_hold_ends_on_viewer_close(self):
        viewer = FakeViewer(close_after=4)
        ui, clock = self.session(viewer=viewer)
        self.assertEqual(ui.hold_result(), "viewer_closed")
        self.assertEqual(viewer.sync_count, 4)

    def test_close_of_either_window_ends_both_observers(self):
        cv2 = FakeOpenCV(keys=[27])
        viewer = FakeViewer()
        ui, clock = self.session(show=True, viewer=viewer, cv2_module=cv2)
        self.assertEqual(ui.hold_result(object()), "key")
        self.assertEqual(viewer.sync_count, 1)

    def test_viewer_key_callback_requests_exit_without_syncing(self):
        viewer = FakeViewer()
        ui, clock = self.session(viewer=viewer)
        ui.on_viewer_key(ord("Q"))
        self.assertFalse(ui.present())
        self.assertEqual(ui.close_reason, "key")
        self.assertEqual(viewer.sync_count, 0)

    def test_headless_has_no_hold_or_import_of_gui(self):
        ui, clock = self.session()
        self.assertEqual(ui.hold_result(), "disabled")
        self.assertIsNone(ui._cv2)
        self.assertEqual(clock.now, 0)

    def test_invalid_hold_duration_rejected(self):
        ui, clock = self.session()
        for duration in (-1, float("nan"), float("inf")):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                ui.hold_result(hold_seconds=duration)

    def test_native_close_waits_past_exit_request_for_resource_release(self):
        class Simulator:
            pass

        simulator = Simulator()
        viewer = FakeViewer()
        viewer._sim = weakref.ref(simulator)
        owner = [simulator]
        del simulator
        clock = FakeClock()

        def sleep(duration):
            # Even though close sets is_running false immediately, resources
            # are retained for three render-loop ticks after the exit request.
            self.assertTrue(viewer.closed)
            clock.sleep(duration)
            if clock.now >= 0.03:
                owner.clear()

        self.assertTrue(close_viewer(viewer, clock=clock, sleep=sleep))
        self.assertGreaterEqual(clock.now, 0.03)
        self.assertIsNone(viewer._sim())

    def test_unknown_native_lifetime_is_not_reported_as_joined(self):
        viewer = FakeViewer()
        self.assertFalse(close_viewer(viewer))
        self.assertTrue(viewer.closed)

    def test_native_shutdown_timeout_is_bounded(self):
        class Simulator:
            pass

        simulator = Simulator()
        viewer = FakeViewer()
        viewer._sim = weakref.ref(simulator)
        clock = FakeClock()
        self.assertFalse(close_viewer(viewer, timeout_s=0.03, clock=clock, sleep=clock.sleep))
        self.assertAlmostEqual(clock.now, 0.03)

    def test_session_cleanup_closes_viewer_before_its_opencv_window(self):
        order = []

        class OrderedViewer(FakeViewer):
            def close(self):
                order.append("viewer")
                super().close()

        class OrderedCV(FakeOpenCV):
            def destroyWindow(self, name):
                order.append("opencv")
                super().destroyWindow(name)

        cv2 = OrderedCV()
        viewer = OrderedViewer()
        ui, clock = self.session(show=True, viewer=viewer, cv2_module=cv2)
        ui.present(object())
        warnings = ui.close()
        self.assertEqual(order, ["viewer", "opencv"])
        self.assertEqual(len(warnings), 1)  # Fake has no observable native owner.
        self.assertEqual(ui.close(), [])


if __name__ == "__main__":
    unittest.main()
