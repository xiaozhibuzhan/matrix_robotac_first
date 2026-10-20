"""Image-driven single-target aiming, with stop/settle/freshness fire gates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from .geometry import AimGeometry, estimate_aim
from .interfaces import Frame, RobotFeedback, RobotMotionCommand
from .perception import TagDetection


@dataclass(frozen=True)
class ControlDecision:
    command: RobotMotionCommand
    state: str
    should_fire: bool
    geometry: AimGeometry | None
    reason: str


class AutoAimController:
    def __init__(self, config: Mapping[str, Any]):
        self.config = config
        self.control = config.get("control", {})
        self.stable_since: float | None = None
        self.fired = False
        self.last_time: float | None = None
        self.last_frame_timestamp: float | None = None

    def mark_fired(self) -> None:
        self.fired = True
        self.stable_since = None

    def _stop(self, state: str, reason: str, geometry: AimGeometry | None = None) -> ControlDecision:
        self.stable_since = None
        return ControlDecision(RobotMotionCommand(), state, False, geometry, reason)

    def update(
        self, frame: Frame, detections: Sequence[TagDetection],
        feedback: RobotFeedback, now: float,
    ) -> ControlDecision:
        if self.fired:
            return self._stop("DONE", "single-target shot completed")
        if not np.isfinite(now) or (self.last_time is not None and now < self.last_time):
            return self._stop("WAIT", "invalid or non-monotonic timestamp")
        self.last_time = now
        max_age = float(self.control.get("max_frame_age_s", 0.15))
        if not feedback.valid:
            return self._stop("WAIT", "robot feedback invalid")
        if not all(np.all(np.isfinite(value)) for value in (
            feedback.rpy_rad, feedback.position_m, feedback.gyro_rad_s, feedback.velocity_m_s,
        )):
            return self._stop("WAIT", "robot feedback contains non-finite values")
        if not 0 <= now - frame.timestamp <= max_age:
            return self._stop("WAIT", "image stale or from the future")
        if not 0 <= now - feedback.timestamp <= float(self.control.get("max_feedback_age_s", max_age)):
            return self._stop("WAIT", "robot feedback stale or from the future")
        # Replaying an old capture with a fresh polling time must not count as
        # another stable visual observation. Timestamp zero is a valid first
        # image in simulation; only subsequent timestamps must increase.
        if self.last_frame_timestamp is not None and frame.timestamp <= self.last_frame_timestamp:
            return self._stop("WAIT", "image timestamp repeated or out of order")
        self.last_frame_timestamp = float(frame.timestamp)
        target_config = self.config["tag"]
        target_id = int(target_config.get("id", 0))
        candidates = [d for d in detections if d.tag_id == target_id
                      and d.hamming == 0
                      and d.decision_margin >= float(target_config.get("min_decision_margin", 30.0))
                      and d.reprojection_error_px <= float(target_config.get("max_reprojection_error_px", 2.0))
                      and 0 <= now - d.timestamp <= max_age
                      and abs(d.timestamp - frame.timestamp) <= 1e-6]
        if not candidates:
            return self._stop("LOST", "target absent or detection quality below threshold")
        detection = min(candidates, key=lambda d: d.reprojection_error_px)
        try:
            geometry = estimate_aim(detection, self.config["camera"], self.config["laser"], target_config)
        except (ValueError, FloatingPointError):
            return self._stop("WAIT", "invalid target geometry")
        if not geometry.reachable:
            return self._stop("UNREACHABLE", "target plane is behind the beam or parallel to it", geometry)
        pitch = float(feedback.rpy_rad[1])
        max_pitch = float(self.control.get("max_pitch_rad", np.deg2rad(20)))
        if abs(pitch) > max_pitch + 1e-5:
            return self._stop("UNREACHABLE", "body pitch exceeds configured limit", geometry)
        tolerance = float(self.control.get("aim_tolerance_m", 0.008))
        if geometry.error_norm_m <= tolerance:
            gyro_ok = np.linalg.norm(feedback.gyro_rad_s) <= float(self.control.get("max_gyro_rad_s", 0.02))
            speed_ok = np.linalg.norm(feedback.velocity_m_s) <= float(self.control.get("max_speed_m_s", 0.01))
            if not gyro_ok or not speed_ok:
                return self._stop("SETTLING", "waiting for angular and linear motion to stop", geometry)
            if self.stable_since is None:
                self.stable_since = now
            stable = now - self.stable_since >= float(self.control.get("stable_duration_s", 0.3))
            return ControlDecision(RobotMotionCommand(), "READY" if stable else "SETTLING", stable,
                                   geometry, "stable visual alignment" if stable else "collecting stable observations")
        self.stable_since = None
        yaw_rate = float(np.clip(
            float(self.control.get("yaw_kp", 1.5)) * geometry.yaw_error_rad,
            -float(self.control.get("max_yaw_rate_rad_s", 0.35)),
            float(self.control.get("max_yaw_rate_rad_s", 0.35)),
        ))
        pitch_rate = float(np.clip(
            float(self.control.get("pitch_kp", 1.5)) * geometry.pitch_error_rad,
            -float(self.control.get("max_pitch_rate_rad_s", 0.18)),
            float(self.control.get("max_pitch_rate_rad_s", 0.18)),
        ))
        if (pitch >= max_pitch and pitch_rate > 0) or (pitch <= -max_pitch and pitch_rate < 0):
            return self._stop("UNREACHABLE", "required pitch exceeds configured limit", geometry)
        return ControlDecision(
            RobotMotionCommand(yaw_rate_rad_s=yaw_rate, pitch_rate_rad_s=pitch_rate),
            "AIMING", False, geometry, "correcting image-derived laser/target error",
        )
