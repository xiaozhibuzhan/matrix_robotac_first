"""Deterministic camera delivery effects, preserving image acquisition times.

Only rendered ``Frame`` objects enter this transport. Delays and losses do not
invent observations: a delivered frame is the original object and is delivered
at most once. Consumers must still apply their own frame-age safety gate.
"""

from __future__ import annotations

import math

import numpy as np

from .interfaces import Frame


class FrameTransport:
    """A bounded, latest-observation queue in simulation seconds.

    Blackouts affect acquisition times in half-open ``[start, end)`` intervals.
    Jitter is uniform in ``[-jitter_s, +jitter_s]`` and total delay is clipped at
    zero. If jitter reorders arrivals, the freshest eligible acquisition wins;
    every older queued acquisition is discarded, including late arrivals.
    """

    def __init__(self, settings=None):
        settings = {} if settings is None else settings
        fields = {"delay_s", "jitter_s", "drop_probability", "drop_every_n",
                  "blackout_intervals_s", "seed", "max_queue"}
        if not isinstance(settings, dict) or set(settings) - fields:
            raise ValueError("unknown observation setting")
        self.delay_s = self._number(settings.get("delay_s", 0.0), "delay_s", 0.0, 600.0)
        self.jitter_s = self._number(settings.get("jitter_s", 0.0), "jitter_s", 0.0, 600.0)
        self.drop_probability = self._number(settings.get("drop_probability", 0.0),
                                             "drop_probability", 0.0, 1.0)
        self.drop_every_n = self._integer(settings.get("drop_every_n", 0), "drop_every_n", 0, 2**31)
        self.max_queue = self._integer(settings.get("max_queue", 128), "max_queue", 1, 100000)
        seed = self._integer(settings.get("seed", 0), "seed", 0, 2**63-1)
        intervals = settings.get("blackout_intervals_s", [])
        if not isinstance(intervals, list):
            raise ValueError("observation.blackout_intervals_s must be a list")
        self.blackout_intervals_s = []
        for interval in intervals:
            if not isinstance(interval, (list, tuple)) or len(interval) != 2:
                raise ValueError("each observation blackout must be [start_s, end_s]")
            start = self._number(interval[0], "blackout start", 0.0, 600.0)
            end = self._number(interval[1], "blackout end", 0.0, 600.0)
            if end <= start:
                raise ValueError("observation blackout end must exceed start")
            self.blackout_intervals_s.append((start, end))
        self.rng = np.random.default_rng(seed)
        self._queue = []
        self._last_delivered_timestamp = -math.inf
        self._last_now = -math.inf
        self._stats = dict.fromkeys(("pushed", "dropped_probability", "dropped_periodic",
                                    "dropped_blackout", "dropped_overflow", "dropped_obsolete",
                                    "delivered", "max_queue_observed"), 0)

    @staticmethod
    def _number(value, name, low, high):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"observation.{name} must be a finite number")
        value = float(value)
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"observation.{name} must be finite in [{low}, {high}]")
        return value

    @staticmethod
    def _integer(value, name, low, high):
        if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
            raise ValueError(f"observation.{name} must be an integer in [{low}, {high}]")
        return value

    def _clock(self, now):
        now = self._number(now, "clock", 0.0, 1e12)
        if now < self._last_now:
            raise ValueError("observation clock cannot move backwards")
        self._last_now = now
        return now

    @property
    def stats(self):
        return dict(self._stats)

    @property
    def pending_count(self):
        return len(self._queue)

    def push(self, frame: Frame, now: float) -> None:
        now = self._clock(now)
        if not isinstance(frame, Frame):
            raise TypeError("observation transport requires a rendered Frame")
        timestamp = self._number(frame.timestamp, "acquisition timestamp", 0.0, 1e12)
        if timestamp > now + 1e-9:
            raise ValueError("frame acquisition cannot be in the future")
        self._stats["pushed"] += 1
        if timestamp <= self._last_delivered_timestamp:
            self._stats["dropped_obsolete"] += 1
            return
        if any(start <= timestamp < end for start, end in self.blackout_intervals_s):
            self._stats["dropped_blackout"] += 1
            return
        if self.drop_every_n and self._stats["pushed"] % self.drop_every_n == 0:
            self._stats["dropped_periodic"] += 1
            return
        if self.drop_probability and self.rng.random() < self.drop_probability:
            self._stats["dropped_probability"] += 1
            return
        jitter = self.rng.uniform(-self.jitter_s, self.jitter_s) if self.jitter_s else 0.0
        self._queue.append((now + max(0.0, self.delay_s+jitter), frame))
        if len(self._queue) > self.max_queue:
            # Retain the freshest acquisitions, regardless of delivery order.
            oldest = min(range(len(self._queue)), key=lambda index: self._queue[index][1].timestamp)
            self._queue.pop(oldest)
            self._stats["dropped_overflow"] += 1
        self._stats["max_queue_observed"] = max(self._stats["max_queue_observed"], len(self._queue))

    def poll(self, now: float) -> Frame | None:
        now = self._clock(now)
        ready = [item for item in self._queue if item[0] <= now+1e-9]
        if not ready:
            return None
        frame = max(ready, key=lambda item: item[1].timestamp)[1]
        obsolete = [item for item in self._queue if item[1].timestamp <= frame.timestamp]
        self._stats["dropped_obsolete"] += len(obsolete)-1
        self._queue = [item for item in self._queue if item[1].timestamp > frame.timestamp]
        self._last_delivered_timestamp = frame.timestamp
        self._stats["delivered"] += 1
        return frame
