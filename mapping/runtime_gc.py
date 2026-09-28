"""Schedule cyclic GC away from unpredictable allocations in scan callbacks.

Reference counting remains active. Cycles are still collected periodically;
only the process-wide automatic cyclic collector is temporarily suspended.
"""

from __future__ import annotations

import gc
import time


class ScheduledGC:
    def __init__(self) -> None:
        self.was_enabled = gc.isenabled()
        self.started = time.monotonic()
        self.last = [self.started] * 3
        self.last_generation = -1
        self.last_ms = 0.0
        self.max_ms = 0.0
        self.collections = [0, 0, 0]
        self.closed = False
        gc.disable()

    def tick(self) -> None:
        if self.closed:
            return
        now = time.monotonic()
        # Large old-generation scans now happen at most once per two minutes,
        # instead of being triggered repeatedly inside dictionary updates.
        generation = next((g for g, period in ((2, 120.0), (1, 15.0), (0, 1.0))
                           if now - self.last[g] >= period), None)
        if generation is None:
            return
        started = time.perf_counter()
        gc.collect(generation)
        self.last_ms = (time.perf_counter() - started) * 1000
        self.max_ms = max(self.max_ms, self.last_ms)
        self.last_generation = generation
        self.collections[generation] += 1
        for g in range(generation + 1):
            self.last[g] = now

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            if self.was_enabled:
                gc.enable()


__all__ = ["ScheduledGC"]
