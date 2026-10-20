"""Read-only packets from the unchanged official physics publisher.

The protobuf's raw time_stamp has no documented unit. Freshness instead uses
the publisher's eCAL send time and the bridge's eCAL/monotonic receive times.
It is a publication timestamp, not a claimed hardware camera acquisition time.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import threading
import time

import numpy as np

from .interfaces import RobotFeedback


@dataclass(frozen=True)
class PhysicsSample:
    sequence: int
    source_clock: int
    published_at_s: float
    bridge_received_at_s: float
    packet: dict


class StalePhysicsSample(ValueError):
    """A structurally valid source sample that must not become feedback."""

    def __init__(self, sample, *, reason, transit_age_s, queued_age_s):
        super().__init__(f"official physics publication is stale ({reason})")
        self.sample = sample
        self.reason = reason
        self.transit_age_s = transit_age_s
        self.queued_age_s = queued_age_s


def _integer(packet, key, minimum=0):
    value = packet.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"official state {key} must be an integer >= {minimum}")
    return value


def decode_sample(packet, *, origin_s, now_s, max_age_s=0.15, previous=None):
    if not isinstance(packet, dict) or packet.get("schema") != "task3.official_state.v1":
        raise ValueError("unknown official physics packet schema")
    seq = _integer(packet, "bridge_seq", 1)
    clock = _integer(packet, "source_ecal_clock", 1)
    _integer(packet, "source_time_stamp_raw")
    if isinstance(packet.get("source_ecal_id"), bool) or not isinstance(packet.get("source_ecal_id"), int):
        raise ValueError("official topic filter ID must be a signed integer")
    sent = _integer(packet, "source_ecal_time_us", 1)
    received_ecal = _integer(packet, "received_ecal_time_us", 1)
    received_mono = _integer(packet, "received_monotonic_ns", 1) * 1e-9
    transit_age = (received_ecal - sent) * 1e-6
    queued_age = now_s + origin_s - received_mono
    if (not all(math.isfinite(x) for x in (origin_s, now_s, max_age_s))
            or max_age_s <= 0 or transit_age < 0 or queued_age < -0.002):
        raise ValueError("official physics publication is future or uses a discontinuous clock")
    publication = received_mono - origin_s - transit_age
    if previous is not None and (seq <= previous.sequence or clock <= previous.source_clock
                                or publication <= previous.published_at_s
                                or received_mono-origin_s < previous.bridge_received_at_s
                                or sent < previous.packet["source_ecal_time_us"]
                                or received_ecal < previous.packet["received_ecal_time_us"]):
        raise ValueError("official physics source sequence/time repeated or regressed")
    # Same-host std::chrono::steady_clock and Python monotonic use Linux
    # CLOCK_MONOTONIC. eCAL plugin time may have another epoch, hence the delta.
    for key, count in (("q", 12), ("qd", 12), ("tau", 12), ("quat_wxyz", 4),
                       ("gyro_body", 3), ("acc_body", 3),
                       ("imu_position_world", 3), ("velocity_world", 3)):
        values = np.asarray(packet.get(key), dtype=float)
        if values.shape != (count,) or not np.all(np.isfinite(values)):
            raise ValueError(f"official physics {key} requires {count} finite values")
    if abs(float(np.linalg.norm(packet["quat_wxyz"])) - 1) > 0.01:
        raise ValueError("official physics quaternion is not unit length")
    sample = PhysicsSample(seq, clock, publication, received_mono-origin_s, packet)
    # Validate the whole packet and its source ordering before classifying age.
    # A delayed publication is recoverable; malformed or reversed packets are
    # not allowed to hide in a stale UDP/eCAL backlog.
    if transit_age + max(0.0, queued_age) > max_age_s:
        reason = ("source_transit" if transit_age > max_age_s else
                  "receiver_queue" if queued_age > max_age_s else
                  "combined_transit_and_queue")
        raise StalePhysicsSample(sample, reason=reason, transit_age_s=transit_age,
                                 queued_age_s=queued_age)
    return sample


def sample_feedback(sample, now_s, max_age_s=0.15):
    if sample is None:
        return RobotFeedback(False, *(np.zeros(3) for _ in range(4)), -1e30)
    packet = sample.packet
    w, x, y, z = np.asarray(packet["quat_wxyz"], float) / np.linalg.norm(packet["quat_wxyz"])
    rpy = np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                    math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
                    math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))])
    return RobotFeedback(0 <= now_s-sample.published_at_s <= max_age_s, rpy,
                         np.array(packet["imu_position_world"], float),
                         np.array(packet["gyro_body"], float),
                         np.array(packet["velocity_world"], float), sample.published_at_s)


class OfficialStateReceiver:
    def __init__(self, *, origin_s=None, clock=time.monotonic, max_age_s=0.15):
        self.origin_s = float(clock()) if origin_s is None else float(origin_s)
        self.clock, self.max_age_s = clock, max_age_s
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.setblocking(False)
        self.port = self.socket.getsockname()[1]
        self.sample = None
        self._last_seen = None
        self.error = None
        self.accepted = self.stale_dropped = 0
        self.stale_drop_reasons = {}
        self.last_stale_drop = None
        self._lock = threading.RLock()

    def now(self):
        return float(self.clock())-self.origin_s

    def latest(self):
        with self._lock:
            if self.error:
                raise RuntimeError(self.error)
            # A delayed consumer drains old packets instead of giving them new
            # timestamps. Only a genuinely fresh, increasing packet is accepted.
            for _ in range(2048):
                try:
                    raw, peer = self.socket.recvfrom(16384)
                except BlockingIOError:
                    break
                try:
                    if peer[0] != "127.0.0.1":
                        raise ValueError("official bridge must use loopback")
                    packet = json.loads(raw)
                    self.sample = decode_sample(packet, origin_s=self.origin_s, now_s=self.now(),
                                                max_age_s=self.max_age_s, previous=self._last_seen)
                    self._last_seen = self.sample
                    self.accepted += 1
                except StalePhysicsSample as exc:
                    # Never refresh a delayed source's publication/receipt time.
                    # Retain source ordering even across rejected old samples.
                    self._last_seen = exc.sample
                    self.stale_dropped += 1
                    self.stale_drop_reasons[exc.reason] = self.stale_drop_reasons.get(exc.reason, 0)+1
                    self.last_stale_drop = {
                        "reason": exc.reason, "sequence": exc.sample.sequence,
                        "source_clock": exc.sample.source_clock,
                        "published_at_s": exc.sample.published_at_s,
                        "bridge_received_at_s": exc.sample.bridge_received_at_s,
                        "transit_age_s": exc.transit_age_s,
                        "queued_age_s": exc.queued_age_s,
                    }
                except Exception as exc:
                    self.error = f"official physics receiver rejected packet: {exc}"
                    raise RuntimeError(self.error) from exc
            return self.sample

    def feedback(self):
        return sample_feedback(self.latest(), self.now(), self.max_age_s)

    def close(self):
        self.socket.close()
