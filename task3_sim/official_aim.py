"""Task-owned PI image servo for the official balance-policy input path.

The public SDK fields retain their official names. In the supplied controller,
the balance policy consumes a filtered bounded attitude input rather than
integrating that input as angular velocity. This outer loop supplies the bias
needed to remove a proportional servo's residual, then preserves the entire
input while visual alignment settles. It changes neither SDK nor policy code.

This module has no simulator, SDK, scene-truth or firing access. The caller
must validate image/feedback freshness before producing a decision and reset
the servo after an SDK mode transition. A unit-test plant is not native gait
validation.
"""
from __future__ import annotations

import math
from typing import Mapping

import numpy as np

from .interfaces import RobotMotionCommand


class OfficialAimServo:
    profile = "official_rl_balance_policy_pi_v1"

    def __init__(self, control: Mapping, settings: Mapping | None = None):
        self.control = dict(control)
        self.settings = dict(settings or {})
        self.bounds = np.array([float(control.get("max_yaw_rate_rad_s", .35)),
                                float(control.get("max_pitch_rate_rad_s", .18))])
        self.ki = np.array([float(self.settings.get("yaw_ki_per_s", .8)),
                           float(self.settings.get("pitch_ki_per_s", .8))])
        self.max_dt = float(control.get("max_frame_age_s", .15))
        self.stall_duration = float(self.settings.get("stall_duration_s", 12.0))
        self.stall_improvement = float(self.settings.get("stall_improvement_m", .002))
        self.stall_no_response = float(self.settings.get("stall_no_response_rad", .001))
        if (not np.all(np.isfinite(self.bounds)) or np.any(self.bounds <= 0)
                or np.any(self.bounds > .5)):
            raise ValueError("Official attitude input bounds must be positive and at most .5")
        if not np.all(np.isfinite(self.ki)) or np.any(self.ki < 0):
            raise ValueError("PI integral gains must be finite and nonnegative")
        if not all(math.isfinite(x) and x > 0 for x in (
                self.max_dt, self.stall_duration, self.stall_improvement, self.stall_no_response)):
            raise ValueError("PI sample and stagnation thresholds must be positive and finite")
        self.reset_count = 0
        self.reset("initialization")

    def reset(self, reason: str):
        """Revoke the previous target's integral and hold after lost authority."""
        self.reset_count += 1
        self.last_reset_reason = str(reason)
        self.phase = "reset"
        self.target_id = None
        self.failure_reason = None
        self._integral = np.zeros(2)
        self._last_time = None
        self._last_state = None
        self._held = RobotMotionCommand()
        self._requested = RobotMotionCommand()
        self._proportional = np.zeros(2)
        self._last_error = None
        self._integrated_dt = 0.0
        self._saturated = np.zeros(2, dtype=bool)
        self.hold_started_at_s = None
        self.last_input_change_at_s = None
        self._clear_stall()

    def _clear_stall(self):
        self._stall_since = None
        self._stall_anchor_error = None
        self._stall_min_angle = None
        self._stall_max_angle = None
        self._stall_no_response_observed = False

    def restart_hold(self, now, reason):
        """Revoke dwell after an actual SDK interruption, retaining the input.

        The caller has just reissued this same bounded input from a new valid
        observation. Neither the old hold time nor the old firing authority
        survives an intervening zero command.
        """
        if self.phase != "holding" or not math.isfinite(now):
            raise ValueError("Only a live held input can restart its dwell")
        self.hold_started_at_s = float(now)
        self.last_input_change_at_s = float(now)
        self.last_reset_reason = str(reason)

    def _apply(self, command, now):
        if command != self._held or self.last_input_change_at_s is None:
            self.last_input_change_at_s = now
        self._held = command
        return command

    def _invalid(self, reason, now=None):
        self.reset(reason)
        self.phase = "waiting"
        if now is not None and math.isfinite(now):
            return self._apply(RobotMotionCommand(), now)
        return RobotMotionCommand()

    @staticmethod
    def _geometry(decision):
        geometry = getattr(decision, "geometry", None)
        if geometry is None or not geometry.reachable:
            raise ValueError("No reachable image geometry")
        angles = np.array([geometry.yaw_error_rad, geometry.pitch_error_rad], dtype=float)
        error = float(geometry.error_norm_m)
        distance = float(geometry.range_m)
        if (not np.all(np.isfinite(angles)) or not math.isfinite(error) or error < 0
                or not math.isfinite(distance) or distance <= 0):
            raise ValueError("Invalid image-derived angular error/range")
        for name, shape in (("bullseye_base_m", (3,)), ("plane_normal_base", (3,)),
                            ("laser_intersection_base_m", (3,)), ("error_tag_xy_m", (2,))):
            vector = np.asarray(getattr(geometry, name), dtype=float)
            if vector.shape != shape or not np.all(np.isfinite(vector)):
                raise ValueError("Invalid image-derived geometry vector")
        return angles, error

    def _supervise(self, now, angles, error):
        if (self._stall_since is None
                or error <= self._stall_anchor_error - self.stall_improvement):
            self._stall_since = now
            self._stall_anchor_error = error
            self._stall_min_angle = angles.copy()
            self._stall_max_angle = angles.copy()
        else:
            self._stall_min_angle = np.minimum(self._stall_min_angle, angles)
            self._stall_max_angle = np.maximum(self._stall_max_angle, angles)
        self._stall_no_response_observed = bool(
            np.linalg.norm(self._stall_max_angle - self._stall_min_angle) <= self.stall_no_response)
        if (now - self._stall_since >= self.stall_duration
                and (bool(np.any(self._saturated)) or self._stall_no_response_observed)):
            self.failure_reason = "aim_response_stalled"
            self.phase = "failed"

    def update(self, decision, now) -> RobotMotionCommand:
        """Use one newly acquired valid decision; return a bounded SDK input.

        P is already present in decision.command. Only new AIMING decisions
        earn integral time, capped to one max_frame_age_s interval. SETTLING
        and READY freeze the last complete input, including both P and I.
        WAIT/LOST/SEARCHING and other phases revoke that hold immediately.
        """
        try:
            now = float(now)
        except (TypeError, ValueError):
            return self._invalid("invalid_time")
        if not math.isfinite(now) or (self._last_time is not None and now <= self._last_time):
            return self._invalid("non_monotonic_time", now)
        state = getattr(decision, "state", None)
        target = getattr(decision, "target_id", None)
        if state not in ("AIMING", "SETTLING", "READY"):
            return self._invalid(f"decision_{state}", now)
        if target is None:
            return self._invalid("missing_target_identity", now)
        if target != self.target_id:
            self.reset("target_changed")
            self.target_id = target
        if self.failure_reason is not None:
            return self._apply(RobotMotionCommand(), now)
        try:
            angles, error = self._geometry(decision)
            requested = decision.command
            values = np.array([requested.vx_m_s, requested.vy_m_s,
                               requested.yaw_rate_rad_s, requested.pitch_rate_rad_s], dtype=float)
            if not np.all(np.isfinite(values)) or np.any(values[:2] != 0):
                raise ValueError("Fine attitude servo requires finite commands without translation")
        except (AttributeError, TypeError, ValueError, OverflowError):
            return self._invalid("invalid_aim_observation", now)
        self._requested = RobotMotionCommand(*values.tolist())
        self._last_error = {"yaw_rad": float(angles[0]), "pitch_rad": float(angles[1]),
                            "norm_m": error, "observed_at_s": now}
        self._integrated_dt = 0.0
        if state in ("SETTLING", "READY"):
            if self.phase != "holding":
                self.hold_started_at_s = now
            self.phase = "holding"
            self._clear_stall()
            self._last_time, self._last_state = now, state
            return self._apply(self._held, now)

        dt = (min(now - self._last_time, self.max_dt)
              if self._last_time is not None and self._last_state == "AIMING" else 0.0)
        if self._last_time is not None and now - self._last_time > self.max_dt + 1e-9:
            # Sparse observations cannot prove twelve seconds of continuous
            # response failure, even though their integral step is bounded.
            self._clear_stall()
        self._proportional = values[2:].copy()
        increments = self.ki * angles * dt
        candidate = np.clip(self._integral + increments, -self.bounds, self.bounds)
        raw_candidate = self._proportional + candidate
        # Reject only increments that drive farther into saturation. Opposite
        # error can unwind an existing bias immediately; there is no unbounded
        # hidden accumulator behind the bounded policy input.
        blocked = ((raw_candidate > self.bounds) & (increments > 0)
                   | (raw_candidate < -self.bounds) & (increments < 0))
        self._integral = np.where(blocked, self._integral, candidate)
        raw = self._proportional + self._integral
        applied = np.clip(raw, -self.bounds, self.bounds)
        self._saturated = np.abs(raw) >= self.bounds - 1e-12
        self._integrated_dt = dt
        self.phase = "aiming"
        self.hold_started_at_s = None
        self._last_time, self._last_state = now, state
        self._supervise(now, angles, error)
        command = (RobotMotionCommand() if self.failure_reason else RobotMotionCommand(
            yaw_rate_rad_s=float(applied[0]), pitch_rate_rad_s=float(applied[1])))
        return self._apply(command, now)

    def report(self) -> dict:
        """Strict-JSON diagnostics; field names preserve the SDK API contract."""
        command = vars(self._held).copy()
        bounded = (abs(self._held.yaw_rate_rad_s) <= self.bounds[0]
                   and abs(self._held.pitch_rate_rad_s) <= self.bounds[1]
                   and self._held.vx_m_s == 0 and self._held.vy_m_s == 0)
        return {"profile": self.profile, "phase": self.phase, "target_id": self.target_id,
                "failure_reason": self.failure_reason, "hold_started_at_s": self.hold_started_at_s,
                "last_input_change_at_s": self.last_input_change_at_s,
                "held_command": command, "input_in_bounds": bool(bounded),
                "requested_rate_command": vars(self._requested).copy(),
                "integral_input": {"yaw": float(self._integral[0]), "pitch": float(self._integral[1])},
                "proportional_input": {"yaw": float(self._proportional[0]), "pitch": float(self._proportional[1])},
                "input_bounds": {"yaw_abs_max": float(self.bounds[0]), "pitch_abs_max": float(self.bounds[1])},
                "integral_bounds": {"yaw_abs_max": float(self.bounds[0]), "pitch_abs_max": float(self.bounds[1])},
                "ki_per_s": {"yaw": float(self.ki[0]), "pitch": float(self.ki[1])},
                "integrated_dt_s": self._integrated_dt, "max_integration_dt_s": self.max_dt,
                "saturated": {"yaw": bool(self._saturated[0]), "pitch": bool(self._saturated[1])},
                "last_error": self._last_error.copy() if self._last_error else None,
                "stall": {"started_at_s": self._stall_since, "anchor_error_m": self._stall_anchor_error,
                          "duration_limit_s": self.stall_duration, "required_improvement_m": self.stall_improvement,
                          "no_angular_response_observed": self._stall_no_response_observed},
                "last_reset_reason": self.last_reset_reason, "reset_count": self.reset_count}
