"""Observation-window lifetime, independent of task motion and firing.

All ``present``/``hold_result`` calls belong to the main thread. The MuJoCo
key callback only sets an event, so it is also safe on its render thread.
"""

from __future__ import annotations

import math
import threading
import time
import weakref


WINDOW_NAME = "Task 3 - autonomous observation"
QUIT_KEYS = (27, ord("q"), ord("Q"))


class InteractiveSession:
    """Pump observers while running, then keep the saved result visible.

    This helper deliberately has no reference to the controller, shooter, or
    robot. Holding a result cannot advance simulation time or fire again.
    """

    def __init__(self, show=False, viewer=None, *, cv2_module=None,
                 clock=time.monotonic, sleep=time.sleep):
        self.show = bool(show)
        self.viewer = viewer
        self._cv2 = cv2_module
        self._clock = clock
        self._sleep = sleep
        self._quit = threading.Event()
        self._window_created = False
        self._last_frame = None
        self.close_reason = None

    @property
    def enabled(self):
        return self.show or self.viewer is not None

    def on_viewer_key(self, keycode):
        """Suitable for ``launch_passive(..., key_callback=...)``."""
        if keycode in QUIT_KEYS:
            self._quit.set()

    def _stop(self, reason):
        self.close_reason = reason
        return False

    def _window_is_visible(self):
        cv2 = self._cv2
        # Some minimal headless/test backends do not implement this API.
        if not hasattr(cv2, "getWindowProperty"):
            return True
        try:
            return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) >= 1
        except NotImplementedError:
            return True
        except Exception:
            # OpenCV raises a native error after the window manager deletes a
            # window on some X11/Qt versions instead of returning zero.
            return False

    def present(self, frame_bgr=None):
        """Update observation windows; false means Esc/Q or a window closed."""
        if self.close_reason is not None:
            return False
        if self._quit.is_set():
            return self._stop("key")
        if self.viewer is not None:
            if not self.viewer.is_running():
                return self._stop("viewer_closed")
            self.viewer.sync()
        if self.show:
            if self._cv2 is None:
                import cv2
                self._cv2 = cv2
            if self._window_created and not self._window_is_visible():
                return self._stop("window_closed")
            if frame_bgr is not None:
                self._last_frame = frame_bgr
            if self._last_frame is not None:
                self._cv2.imshow(WINDOW_NAME, self._last_frame)
                self._window_created = True
            if self._cv2.waitKey(1) & 0xff in QUIT_KEYS:
                return self._stop("key")
            if self._window_created and not self._window_is_visible():
                return self._stop("window_closed")
        return True

    def hold_result(self, frame_bgr=None, hold_seconds=None, poll_interval=0.02):
        """Hold until close by default, or for an explicitly bounded duration.

        A duration of zero skips the hold. The caller should stop motion and
        save its artifacts before invoking this method. No simulation work is
        performed here, including when only the MuJoCo observer is active.
        """
        if hold_seconds is not None:
            if not math.isfinite(hold_seconds) or hold_seconds < 0:
                raise ValueError("hold_seconds must be finite and nonnegative")
            if hold_seconds == 0:
                return "skipped"
        if not math.isfinite(poll_interval) or poll_interval <= 0:
            raise ValueError("poll_interval must be finite and positive")
        if not self.enabled:
            return "disabled"
        deadline = None if hold_seconds is None else self._clock() + hold_seconds
        while deadline is None or self._clock() < deadline:
            if not self.present(frame_bgr):
                return self.close_reason
            delay = poll_interval if deadline is None else min(poll_interval, max(0, deadline-self._clock()))
            self._sleep(delay)
        return "elapsed"

    def close_windows(self):
        """Destroy the OpenCV observer on its owning main thread."""
        if self._window_created and self._cv2 is not None:
            try:
                self._cv2.destroyWindow(WINDOW_NAME)
            finally:
                self._window_created = False

    def close(self):
        """Close the native viewer before the camera renderer is destroyed.

        Return diagnostic warnings so the caller can save them with its run.
        Only the observation window owned by this session is destroyed.
        """
        warnings = []
        try:
            if not close_viewer(self.viewer):
                warnings.append("MuJoCo viewer exit requested, but native cleanup was not confirmed within 5 seconds")
        except Exception as exc:
            warnings.append(f"MuJoCo viewer shutdown failed: {exc}")
        finally:
            self.viewer = None
        try:
            self.close_windows()
        except Exception as exc:
            warnings.append(f"OpenCV observation window shutdown failed: {exc}")
        return warnings


def close_viewer(viewer, timeout_s=5.0, *, clock=time.monotonic, sleep=time.sleep):
    """Request viewer exit and wait for native cleanup when observable.

    MuJoCo 3.3's ``Handle.close`` only requests exit: ``is_running`` becomes
    false before the render thread has freed its OpenGL context. That release
    exposes no public join operation. Its ``_sim`` weak reference expires once
    native destruction has completed, so use it as a bounded compatibility
    check. Return false if completion cannot be observed; never equate an exit
    request with a completed shutdown. No upstream MuJoCo file is modified.
    """
    if viewer is None:
        return True
    if not math.isfinite(timeout_s) or timeout_s < 0:
        raise ValueError("timeout_s must be finite and nonnegative")
    simulator_ref = getattr(viewer, "_sim", None)
    viewer.close()
    if not isinstance(simulator_ref, weakref.ReferenceType):
        return False
    deadline = clock() + timeout_s
    while simulator_ref() is not None:
        remaining = deadline-clock()
        if remaining <= 0:
            return False
        sleep(min(0.01, remaining))
    return True
