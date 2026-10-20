"""Public-observation-only sequencing of independent fixed targets.

Only camera/laser calibration, per-ID marker-to-bullseye offsets and control
limits enter this manager. It never reads a scene, simulator or target world
pose. ``single_pass`` follows the declared target order; ``round_robin`` follows
ascending numeric IDs on every round. The default search stays within +/-
``search_span_rad`` of the first valid feedback
yaw. Optional ``search_mode=full_rotation`` searches in positive yaw, measuring
unwrapped feedback travel separately for each selected target. Once that
rotation budget is spent it stops searching. ``fixed_directions`` uses only
public direction priors and image-derived ID bearings: it turns by the shortest
angle to a previously observed identity, or probes the remaining known target
directions. Direction priors never imply a particular AprilTag identity.

``update`` returns a decision; it never fires. ``record_shot`` accepts exactly
one callback for the latest fresh READY decision, with the matching intended ID
and ammo decreased by one. An actual hit on another ID is recorded but never
credited to either target. Legacy single_pass gives each target one attempt.
round_robin revisits numeric IDs in ascending order until the magazine is empty;
each visit requires a new controller/dwell and can spend at most one round.
Empty delivery polls always stop and revoke fire authority. A short gap while
the previous acquisition and stationary feedback remain fresh pauses stable
dwell; it earns no observation time. Stale/repeated frames or long gaps reset.

Overall states: RUNNING, SUCCESS (all completed), PARTIAL (all terminal, some
completed), FAILED (all terminal, none completed), NO_AMMO (unfinished targets
failed because the shared magazine is empty), CANCELLED. Target states:
PENDING, ACTIVE, LOST, COMPLETED, FAILED. ``summary`` and decision ``to_dict``
return JSON-compatible values, including per-shot identities and history.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from .controller import AutoAimController
from .geometry import AimGeometry, estimate_aim
from .interfaces import Frame, RobotFeedback, RobotMotionCommand, ShotResult
from .perception import TagDetection


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    if isinstance(value, np.generic):
        return _plain(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


@dataclass(frozen=True)
class MissionDecision:
    command: RobotMotionCommand
    state: str
    should_fire: bool
    geometry: AimGeometry | None
    reason: str
    target_id: int | None
    mission_state: str

    def to_dict(self) -> dict[str, Any]:
        return _plain(asdict(self))


@dataclass
class _TargetRecord:
    id: int
    state: str = "PENDING"
    reason: str = "awaiting declared-order selection"
    attempts: int = 0
    ring: int = 0
    active_since: float | None = None
    last_seen: float | None = None
    missing_since: float | None = None
    finished_at: float | None = None
    search_origin_yaw_rad: float | None = None
    search_travel_rad: float = 0.0
    search_exhausted: bool = False
    search_probes: list[dict[str, Any]] = field(default_factory=list)
    history: list[dict[str, Any]] = field(default_factory=list)
    successful_hits: int = 0
    round_index: int = 0
    retired: bool = False
    retired_round: int | None = None
    retired_at_s: float | None = None
    visits: list[dict[str, Any]] = field(default_factory=list)


@dataclass(frozen=True)
class _FireToken:
    target_id: int
    now: float
    frame_timestamp: float
    ammo_before: int
    round_index: int
    target_attempt: int


class MultiTargetMission:
    """Keep target identity, attempts and fresh-image fire authority together."""

    def __init__(self, public_config: Mapping[str, Any]):
        # Explicit copying of public fields prevents accidentally retaining a
        # simulation subsection or any target entry's world-pose metadata.
        self._base = {key: deepcopy(public_config[key]) for key in ("camera", "laser")}
        self._base["control"] = deepcopy(public_config.get("control", {}))
        self.control = self._base["control"]
        self.settings = deepcopy(public_config.get("mission", {}))
        self._tag_configs: dict[int, dict[str, Any]] = {}
        common = public_config.get("tag", {})
        for entry in public_config.get("targets", []):
            target_id = int(entry["id"])
            if target_id in self._tag_configs:
                raise ValueError(f"duplicate declared target ID {target_id}")
            tag = {**deepcopy(common), **deepcopy(entry.get("tag", {}))}
            if int(tag.get("id", target_id)) != target_id:
                raise ValueError("target ID and tag calibration ID differ")
            tag["id"] = target_id
            self._tag_configs[target_id] = tag
        if not self._tag_configs:
            raise ValueError("a multi-target mission requires declared target IDs")
        self.shooting_policy = self.settings.get("shooting_policy", "single_pass")
        if self.shooting_policy not in ("single_pass", "round_robin"):
            raise ValueError("mission.shooting_policy must be single_pass or round_robin")
        if self.shooting_policy == "round_robin":
            self._tag_configs = dict(sorted(self._tag_configs.items()))
        self.round_index = 1
        self.completed_rounds = 0
        self.termination_reason = None
        self.target_timeout = self._positive("target_timeout_s", 12.0)
        self.lost_timeout = self._positive("lost_timeout_s", 1.2)
        self.switch_pause = self._positive("switch_pause_s", 0.15, allow_zero=True)
        self.search_rate = self._positive("search_yaw_rate_rad_s", 0.15, allow_zero=True)
        self.search_span = self._positive("search_span_rad", 1.1, allow_zero=True)
        self.search_mode = self.settings.get("search_mode", "bounded")
        if self.search_mode not in ("bounded", "full_rotation", "fixed_directions"):
            raise ValueError("mission.search_mode must be bounded, full_rotation or fixed_directions")
        self.search_max_rotation = self._positive("search_max_rotation_rad", float(2 * np.pi))
        if self.search_max_rotation > 2 * np.pi:
            raise ValueError("mission.search_max_rotation_rad cannot exceed one full rotation")
        directions = self.settings.get("search_directions_rad", [0.0, float(np.pi / 2), float(-np.pi / 2)])
        if (not isinstance(directions, (list, tuple)) or not 1 <= len(directions) <= 32
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not np.isfinite(value) for value in directions)):
            raise ValueError("mission.search_directions_rad must contain 1..32 finite angles")
        self.search_directions = [self._wrap(float(value)) for value in directions]
        if any(abs(self._wrap(a - b)) < 1e-6 for index, a in enumerate(self.search_directions)
               for b in self.search_directions[index + 1:]):
            raise ValueError("mission.search_directions_rad must be unique modulo a full turn")
        self.direction_tolerance = self._positive("direction_tolerance_rad", 0.10)
        if self.direction_tolerance >= np.pi / 4:
            raise ValueError("mission.direction_tolerance_rad must be smaller than pi/4")
        self.direction_probe = self._positive("direction_probe_s", 0.35)
        self.bearing_max_age = self._positive("bearing_max_age_s", 120.0)
        self.bearing_max_translation = self._positive("bearing_max_translation_m", 0.20)
        self.local_search_span = self._positive("local_search_span_rad", 0.25, allow_zero=True)
        if self.local_search_span >= np.pi / 2:
            raise ValueError("mission.local_search_span_rad must be smaller than pi/2")
        if int(self.settings.get("max_target_attempts", 1)) != 1:
            raise ValueError("each fixed-target visit allows one attempt")
        self._records = {target_id: _TargetRecord(target_id) for target_id in self._tag_configs}
        self.active_target_id: int | None = None
        self._controller: AutoAimController | None = None
        self._status = "RUNNING"
        self._armed: _FireToken | None = None
        self._last_now: float | None = None
        self._last_frame_timestamp: float | None = None
        self._switch_until = float("-inf")
        self._search_origin: float | None = None
        self._search_direction = 1.0
        self._active_yaw_last: float | None = None
        self._shots: list[dict[str, Any]] = []
        self._ammo_remaining: int | None = None
        self._bearing_map: dict[int, dict[str, Any]] = {}
        self._feedback_history: list[RobotFeedback] = []
        self._search_goal: dict[str, Any] | None = None
        self._checked_directions: set[int] = set()
        self._cached_probe_center: float | None = None
        self._cached_probe_index = 0
        self._probe_elapsed = 0.0
        self._probe_last_now: float | None = None
        self._probe_last_capture: float | None = None
        self._observation_not_before = float("-inf")
        self._observation_invalidation_reason = "control mode transition requires new image"

    def _positive(self, name: str, default: float, allow_zero: bool = False) -> float:
        value = float(self.settings.get(name, default))
        if not np.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
            raise ValueError(f"mission.{name} must be {'nonnegative' if allow_zero else 'positive'} and finite")
        return value

    @staticmethod
    def _wrap(angle: float) -> float:
        return float(np.arctan2(np.sin(angle), np.cos(angle)))

    @property
    def status(self) -> str:
        return self._status

    @property
    def done(self) -> bool:
        return self._status != "RUNNING"

    @property
    def target_states(self) -> dict[int, str]:
        return {target_id: record.state for target_id, record in self._records.items()}

    def summary(self) -> dict[str, Any]:
        score_sums = {target_id: 0 for target_id in self._records}
        for shot in self._shots:
            score_sums[shot["intended_target_id"]] += shot["credited_ring"]
        total_score = sum(score_sums.values())
        return _plain({
            "status": self.status,
            "active_target_id": self.active_target_id,
            "ammo_remaining": self._ammo_remaining,
            "shooting_policy": self.shooting_policy,
            "firing_order": list(self._records),
            "round_index": self.round_index,
            "completed_rounds": self.completed_rounds,
            "termination_reason": self.termination_reason,
            "coverage_complete": all(record.successful_hits > 0 for record in self._records.values()),
            "completed_count": sum((record.successful_hits > 0 if self.shooting_policy == "round_robin"
                                     else record.state == "COMPLETED") for record in self._records.values()),
            "failed_count": sum(record.state == "FAILED" for record in self._records.values()),
            "total_score": total_score,
            "shot_score_sum": total_score,
            "latest_target_score_total": sum(record.ring for record in self._records.values()),
            "score_policy": "sum_of_committed_credited_shots; shot_score_sum equals total_score; per-target ring is latest attempt",
            "targets": [{**asdict(record), "score_sum": score_sums[record.id]}
                        for record in self._records.values()],
            "shots": deepcopy(self._shots),
            "search": {"mode": self.search_mode, "initial_yaw_rad": self._search_origin,
                       "directions_relative_rad": self.search_directions,
                       "observed_bearings": deepcopy(self._bearing_map)},
        })

    def _transition(self, record: _TargetRecord, state: str, reason: str, now: float) -> None:
        if record.state != state:
            record.history.append({"timestamp": float(now), "from": record.state, "to": state, "reason": reason})
        record.state, record.reason = state, reason
        if state in ("COMPLETED", "FAILED"):
            record.finished_at = float(now)

    def _reset_dwell(self) -> None:
        if self._controller is not None:
            self._controller.stable_since = None
        self._armed = None

    def invalidate_control_observation(self, now: float, reason: str = "control mode transition requires new image") -> None:
        """Revoke a decision after an external control transition.

        Identity, target state, attempts, ammo and image-derived search bearings
        are retained. A new stable dwell requires an image acquired strictly
        after this transition completed; queued pre-transition images cannot
        rearm a previously READY decision even while their age is acceptable.
        The caller owns the control transition and supplies its session time.
        """
        self._reset_dwell()
        self._pause_probe(reset=True)
        if (not np.isfinite(now) or now < self._observation_not_before
                or (self._last_now is not None and now < self._last_now)):
            raise ValueError("control observation invalidation needs a finite nondecreasing session time")
        self._observation_not_before = max(self._observation_not_before, float(now))
        self._observation_invalidation_reason = str(reason)
        self._feedback_history = []  # Do not interpolate across the unobserved control transition.

    def _decision(self, state: str, reason: str, *, command: RobotMotionCommand | None = None,
                  geometry: AimGeometry | None = None, fire: bool = False,
                  target_id: int | None = None) -> MissionDecision:
        return MissionDecision(command or RobotMotionCommand(), state, fire, geometry, reason,
                               self.active_target_id if target_id is None else target_id, self.status)

    def _finish_if_terminal(self, now=None) -> None:
        if self.shooting_policy == "round_robin":
            if self._status != "RUNNING":
                return
            now = float(self._last_now if now is None else now)
            round_done = all(record.retired or (record.round_index == self.round_index
                and record.state in ("COMPLETED", "FAILED")) for record in self._records.values())
            if round_done:
                self.completed_rounds = self.round_index
            if self._ammo_remaining == 0:
                covered = all(record.successful_hits > 0 for record in self._records.values())
                for record in self._records.values():
                    if record.state not in ("COMPLETED", "FAILED"):
                        self._transition(record, "COMPLETED" if record.successful_hits else "FAILED",
                                         "ammo_exhausted_after_prior_hit" if record.successful_hits else "no_ammo", now)
                self._status = "SUCCESS" if covered else "NO_AMMO"
                self.termination_reason = "ammo_exhausted"
            elif round_done:
                eligible = [record for record in self._records.values() if not record.retired]
                if eligible:
                    self.round_index += 1
                    for record in eligible:
                        self._transition(record, "PENDING", "next ascending-ID round; new observation required", now)
                        record.active_since = record.last_seen = record.missing_since = record.finished_at = None
                        record.search_origin_yaw_rad = None
                        record.search_travel_rad = 0.0
                        record.search_exhausted = False
                    return
                self._status = "PARTIAL" if any(record.successful_hits for record in self._records.values()) else "FAILED"
                self.termination_reason = "all_targets_retired"
            if self.done:
                self.active_target_id, self._controller, self._armed = None, None, None
            return
        terminal = all(record.state in ("COMPLETED", "FAILED") for record in self._records.values())
        if terminal and self._status == "RUNNING":
            completed = sum(record.state == "COMPLETED" for record in self._records.values())
            self._status = "SUCCESS" if completed == len(self._records) else "PARTIAL" if completed else "FAILED"
            self.active_target_id, self._controller = None, None
            self._armed = None

    def _end_active(self, state: str, reason: str, now: float, *, retire=False) -> None:
        record = self._records[self.active_target_id]
        if retire:
            record.retired, record.retired_round, record.retired_at_s = True, self.round_index, float(now)
        if record.visits:
            record.visits[-1].update(finished_at_s=float(now), state=state, reason=reason,
                                    shot_index=len(self._shots) if not retire else None)
        self._transition(record, state, reason, now)
        self.active_target_id, self._controller = None, None
        self._armed = None
        self._switch_until = now + self.switch_pause
        self._finish_if_terminal(now)

    def fail_active(self, now: float, reason: str) -> MissionDecision:
        """Retire an unresponsive target without spending ammunition or score."""
        if (not np.isfinite(now) or (self._last_now is not None and now < self._last_now)):
            raise ValueError("target failure needs finite nondecreasing mission time")
        target = self.active_target_id
        if target is None or self.done:
            raise ValueError("no active target can be failed")
        self._last_now = float(now)
        self._reset_dwell()
        self._end_active("FAILED", str(reason), now, retire=True)
        return self._decision(self.status if self.done else "SWITCHING", str(reason), target_id=target)

    def _feedback_error(self, feedback: RobotFeedback, now: float) -> str | None:
        if not feedback.valid:
            return "robot feedback invalid"
        try:
            if not all(np.asarray(value).shape == (3,) and np.all(np.isfinite(value)) for value in (
                feedback.rpy_rad, feedback.position_m, feedback.gyro_rad_s, feedback.velocity_m_s,
            )):
                return "robot feedback contains invalid vectors"
            max_age = float(self.control.get("max_feedback_age_s", self.control.get("max_frame_age_s", 0.15)))
            if not 0 <= now - feedback.timestamp <= max_age:
                return "robot feedback stale or from the future"
        except (TypeError, ValueError):
            return "robot feedback contains invalid values"
        return None

    def _note_missing(self, record: _TargetRecord, now: float, reason: str) -> None:
        self._reset_dwell()
        if record.missing_since is None:
            record.missing_since = now
        if now - record.missing_since >= self.lost_timeout:
            self._transition(record, "LOST", reason, now)

    def _observe_active_yaw(self, yaw: float) -> None:
        """Measure actual rotation using public feedback, including wraparound.

        A feedback update is much smaller than pi during normal operation, so
        its shortest angular increment unwraps both [-pi, pi] feedback and the
        backend's already unbounded yaw. Include aiming corrections and coast
        motion in the budget; reacquisition never grants another full sweep.
        """
        if self.search_mode not in ("full_rotation", "fixed_directions") or self.active_target_id is None:
            return
        record = self._records[self.active_target_id]
        if self._active_yaw_last is None:
            self._active_yaw_last = yaw
            record.search_origin_yaw_rad = yaw
            return
        delta = float(np.arctan2(np.sin(yaw - self._active_yaw_last), np.cos(yaw - self._active_yaw_last)))
        record.search_travel_rad += abs(delta)
        self._active_yaw_last = yaw
        if self.search_mode == "full_rotation":
            record.search_exhausted = record.search_travel_rad >= self.search_max_rotation

    def _remember_feedback(self, feedback: RobotFeedback) -> None:
        """Keep the public feedback time base for delayed camera acquisitions."""
        if self._feedback_history and feedback.timestamp <= self._feedback_history[-1].timestamp:
            return
        self._feedback_history.append(feedback)
        cutoff = feedback.timestamp - max(2.0, float(self.control.get("max_frame_age_s", 0.15)) * 2)
        while len(self._feedback_history) > 2 and self._feedback_history[1].timestamp < cutoff:
            self._feedback_history.pop(0)

    def _feedback_at(self, timestamp: float) -> tuple[np.ndarray, np.ndarray] | None:
        """Interpolate only when feedback covers the image capture timestamp.

        SDK adapters must normalize camera/feedback timestamps to one clock.
        There is deliberately no extrapolation using the newest robot yaw.
        """
        for feedback in self._feedback_history:
            if abs(feedback.timestamp - timestamp) <= 1e-6:
                return np.asarray(feedback.rpy_rad, dtype=float), np.asarray(feedback.position_m, dtype=float)
        for before, after in zip(self._feedback_history, self._feedback_history[1:]):
            if before.timestamp <= timestamp <= after.timestamp:
                if after.timestamp - before.timestamp > float(self.control.get("max_feedback_age_s", 0.15)):
                    return None  # An unobserved interval is not a measured yaw trajectory.
                fraction = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
                delta = np.array([self._wrap(float(b - a)) for a, b in zip(before.rpy_rad, after.rpy_rad)])
                return (np.asarray(before.rpy_rad, dtype=float) + fraction * delta,
                        np.asarray(before.position_m, dtype=float)
                        + fraction * (np.asarray(after.position_m) - np.asarray(before.position_m)))
        return None

    def _remember_bearings(self, frame: Frame, detections: Sequence[TagDetection]) -> None:
        if self.search_mode != "fixed_directions":
            return
        sampled = self._feedback_at(float(frame.timestamp))
        if sampled is None:
            return
        rpy, position = sampled
        roll, pitch, yaw = rpy
        cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
        rotation_world_base = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        for target_id, tag in self._tag_configs.items():
            same_id = [item for item in detections if item.tag_id == target_id
                       and abs(item.timestamp - frame.timestamp) <= 1e-6]
            if len(same_id) != 1:
                continue  # Ambiguous IDs cannot create or move a bearing.
            detection = same_id[0]
            if (detection.hamming != 0 or not np.isfinite(detection.decision_margin)
                    or detection.decision_margin < float(tag.get("min_decision_margin", 30.0))
                    or not np.isfinite(detection.reprojection_error_px)
                    or not 0 <= detection.reprojection_error_px <= float(tag.get("max_reprojection_error_px", 2.0))):
                continue
            try:
                geometry = estimate_aim(detection, self._base["camera"], self._base["laser"], tag)
                vector_world = rotation_world_base @ geometry.bullseye_base_m
                if np.linalg.norm(vector_world[:2]) < 1e-6:
                    continue
            except (ValueError, FloatingPointError, TypeError):
                continue
            self._bearing_map[target_id] = {
                "yaw_rad": float(np.arctan2(vector_world[1], vector_world[0])),
                "timestamp": float(frame.timestamp), "position_m": np.array(position, copy=True),
                "source": "image_pose_with_timestamped_robot_feedback",
            }

    def _usable_bearing(self, target_id: int, feedback: RobotFeedback, now: float) -> dict[str, Any] | None:
        bearing = self._bearing_map.get(target_id)
        if (bearing is None or not 0 <= now - bearing["timestamp"] <= self.bearing_max_age
                or np.linalg.norm(np.asarray(feedback.position_m) - bearing["position_m"])
                > self.bearing_max_translation):
            return None
        return bearing

    def _pause_probe(self, *, reset: bool = False) -> None:
        self._probe_last_now = self._probe_last_capture = None
        if reset:
            self._probe_elapsed = 0.0

    def _start_goal(self, yaw: float, source: str, now: float, index: int | None = None) -> None:
        self._search_goal = {"yaw_rad": self._wrap(yaw), "source": source, "direction_index": index}
        self._pause_probe(reset=True)
        self._records[self.active_target_id].search_probes.append({
            "timestamp": float(now), **self._search_goal,
        })

    def _fixed_direction_search(self, frame: Frame, feedback: RobotFeedback, now: float) -> MissionDecision:
        record = self._records[self.active_target_id]
        yaw = float(feedback.rpy_rad[2])
        if record.search_exhausted:
            return self._decision("SEARCH_EXHAUSTED", "fixed directions probed; stopped until target deadline")
        bearing = self._usable_bearing(record.id, feedback, now)
        if bearing is None and self._cached_probe_center is not None:
            self._cached_probe_center, self._search_goal = None, None
            self._pause_probe(reset=True)
        if bearing is not None and self._cached_probe_center is None:
            # A visual identity acquired while probing another direction
            # immediately replaces the direction-prior hypothesis.
            self._cached_probe_center = float(bearing["yaw_rad"])
            self._cached_probe_index = 0
            self._search_goal = None
        if self._search_goal is None:
            if self._cached_probe_center is not None and bearing is not None:
                offsets = [0.0, self.local_search_span, -self.local_search_span]
                if self.local_search_span == 0:
                    offsets = [0.0]
                if self._cached_probe_index < len(offsets):
                    self._start_goal(self._cached_probe_center + offsets[self._cached_probe_index],
                                     "image_derived_bearing", now)
                else:
                    record.search_exhausted = True
            else:
                occupied = set()
                for other_id in self._tag_configs:
                    if other_id == record.id:
                        continue
                    other = self._usable_bearing(other_id, feedback, now)
                    if other is not None:
                        distances = [abs(self._wrap(other["yaw_rad"] - self._search_origin - direction))
                                     for direction in self.search_directions]
                        nearest = int(np.argmin(distances))
                        if distances[nearest] <= max(0.35, self.direction_tolerance * 2):
                            occupied.add(nearest)
                candidates = [index for index in range(len(self.search_directions))
                              if index not in occupied and index not in self._checked_directions]
                if candidates:
                    index = min(candidates, key=lambda item: abs(self._wrap(
                        self._search_origin + self.search_directions[item] - yaw)))
                    self._start_goal(self._search_origin + self.search_directions[index],
                                     "public_direction_prior_identity_unknown", now, index)
                else:
                    record.search_exhausted = True
            if record.search_exhausted:
                return self._decision("SEARCH_EXHAUSTED", "fixed directions probed; stopped until target deadline")
        error = self._wrap(self._search_goal["yaw_rad"] - yaw)
        if abs(error) > self.direction_tolerance:
            self._pause_probe(reset=True)
            limit = min(self.search_rate, float(self.control.get("max_yaw_rate_rad_s", 0.35)))
            rate = float(np.clip(float(self.control.get("yaw_kp", 1.5)) * error, -limit, limit))
            return self._decision("SEARCHING", "shortest-angle turn to " + self._search_goal["source"],
                                  command=RobotMotionCommand(yaw_rate_rad_s=rate))
        # Stop at the candidate. Only successive fresh camera deliveries with
        # stationary feedback accrue probe time; missing polls earn none.
        stationary = (np.linalg.norm(feedback.gyro_rad_s) <= float(self.control.get("max_gyro_rad_s", 0.02))
                      and np.linalg.norm(feedback.velocity_m_s) <= float(self.control.get("max_speed_m_s", 0.01)))
        max_age = float(self.control.get("max_frame_age_s", 0.15))
        if not stationary:
            self._pause_probe(reset=True)
        else:
            if (self._probe_last_now is not None and self._probe_last_capture is not None
                    and 0 < frame.timestamp - self._probe_last_capture <= max_age):
                self._probe_elapsed += now - self._probe_last_now
            self._probe_last_now, self._probe_last_capture = float(now), float(frame.timestamp)
        if self._probe_elapsed >= self.direction_probe:
            index = self._search_goal["direction_index"]
            if index is not None:
                self._checked_directions.add(index)
            else:
                self._cached_probe_index += 1
            self._search_goal = None
            self._pause_probe(reset=True)
        return self._decision("PROBING", "stationary fresh-image probe; identity must be visually decoded")

    def update(self, frame: Frame | None, detections: Sequence[TagDetection],
               feedback: RobotFeedback, now: float, ammo_remaining: int) -> MissionDecision:
        self._armed = None  # Every polling result revokes earlier fire authority.
        if self.done:
            return self._decision(self.status, "mission already finished")
        if not np.isfinite(now) or (self._last_now is not None and now < self._last_now):
            self._reset_dwell()
            return self._decision("WAIT", "invalid or non-monotonic timestamp")
        previous_poll_time = self._last_now
        self._last_now = float(now)
        if isinstance(ammo_remaining, (bool, np.bool_)) or not isinstance(ammo_remaining, (int, np.integer)) or ammo_remaining < 0:
            self._reset_dwell()
            return self._decision("WAIT", "invalid shared ammunition count")
        if (self.shooting_policy == "round_robin" and self._ammo_remaining is not None
                and ammo_remaining != self._ammo_remaining):
            self._reset_dwell()
            return self._decision("WAIT", "ammunition changed outside the accepted shot transaction")
        self._ammo_remaining = int(ammo_remaining)
        if ammo_remaining == 0:
            if self.shooting_policy == "round_robin":
                self._finish_if_terminal(now)
                return self._decision(self.status, "shared magazine empty; per-shot scores retained")
            for record in self._records.values():
                if record.state not in ("COMPLETED", "FAILED"):
                    self._transition(record, "FAILED", "no_ammo", now)
            self._status = "NO_AMMO"
            self.active_target_id, self._controller = None, None
            return self._decision("NO_AMMO", "shared magazine empty; completed targets retained")
        if self.active_target_id is not None:
            record = self._records[self.active_target_id]
            if now - record.active_since >= self.target_timeout:
                timed_out_id = record.id
                self._end_active("FAILED", "target_timeout", now, retire=True)
                return self._decision(self.status if self.done else "SWITCHING", "target_timeout", target_id=timed_out_id)
        feedback_error = self._feedback_error(feedback, now)
        if feedback_error is None:
            self._remember_feedback(feedback)
            self._observe_active_yaw(float(feedback.rpy_rad[2]))
        max_age = float(self.control.get("max_frame_age_s", 0.15))
        # A delivery queue can legitimately have one empty polling tick while
        # its most recent acquisition remains fresh. Stop on that tick, revoke
        # fire authority, and *pause* the dwell clock rather than interpreting
        # a short transport gap as visual target disappearance. Advancing the
        # start by this polling interval means no silent time earns stability.
        # Actual observations that are stale/repeated, long capture gaps and
        # invalid feedback still use the reset path below.
        if (frame is None and feedback_error is None and self.active_target_id is not None
                and self._last_frame_timestamp is not None
                and 0 <= now - self._last_frame_timestamp <= max_age
                and self._records[self.active_target_id].missing_since is None):
            stationary = (np.linalg.norm(feedback.gyro_rad_s)
                          <= float(self.control.get("max_gyro_rad_s", 0.02))
                          and np.linalg.norm(feedback.velocity_m_s)
                          <= float(self.control.get("max_speed_m_s", 0.01)))
            if stationary and self._controller.stable_since is not None:
                self._controller.stable_since += now - previous_poll_time
            elif not stationary:
                self._reset_dwell()
            self._pause_probe()
            return self._decision("WAIT", "short camera delivery gap; stable dwell paused" if stationary
                                  else "short camera delivery gap; robot motion resets stable dwell")
        image_error = None
        if frame is None:
            image_error = "no delivered camera frame"
        elif not np.isfinite(frame.timestamp) or not 0 <= now - frame.timestamp <= max_age:
            image_error = "image stale or from the future"
        elif self._last_frame_timestamp is not None and frame.timestamp <= self._last_frame_timestamp:
            image_error = "image timestamp repeated or out of order"
        elif frame.timestamp <= self._observation_not_before:
            image_error = "image acquired before latest control transition completed: " + self._observation_invalidation_reason
        if feedback_error or image_error:
            self._reset_dwell()
            self._pause_probe(reset=True)
            if self.active_target_id is not None:
                self._note_missing(self._records[self.active_target_id], now, feedback_error or image_error)
            return self._decision("WAIT", feedback_error or image_error)
        if self._last_frame_timestamp is not None and frame.timestamp - self._last_frame_timestamp > max_age:
            self._reset_dwell()  # A silent gap between capture deliveries is not stable observation time.
        self._last_frame_timestamp = float(frame.timestamp)
        if self._search_origin is None:
            self._search_origin = float(feedback.rpy_rad[2])
        self._remember_bearings(frame, detections)
        if now < self._switch_until:
            return self._decision("SWITCHING", "stopping before selecting the next declared target")
        if self.active_target_id is None:
            target_id = next(target_id for target_id, record in self._records.items() if record.state == "PENDING")
            record = self._records[target_id]
            self.active_target_id = target_id
            self._controller = AutoAimController({**self._base, "tag": self._tag_configs[target_id]})
            self._search_goal, self._cached_probe_center = None, None
            self._checked_directions = set()
            self._cached_probe_index = 0
            self._pause_probe(reset=True)
            self._active_yaw_last = None
            self._observe_active_yaw(float(feedback.rpy_rad[2]))
            record.active_since = float(now)
            record.round_index = self.round_index
            record.visits.append({"round_index": self.round_index, "selected_at_s": float(now),
                                  "finished_at_s": None, "state": "ACTIVE", "reason": "selected", "shot_index": None})
            self._transition(record, "ACTIVE", "selected in ascending ID order" if self.shooting_policy == "round_robin"
                             else "selected in declared target order", now)
        record = self._records[self.active_target_id]
        matching = [detection for detection in detections if detection.tag_id == record.id
                    and abs(detection.timestamp - frame.timestamp) <= 1e-6]
        if len(matching) > 1:
            self._note_missing(record, now, "ambiguous duplicate target ID")
            self._pause_probe(reset=True)
            return self._decision("AMBIGUOUS", "duplicate same-ID detections inhibit aiming and firing")
        tag = self._tag_configs[record.id]
        valid = [detection for detection in matching if detection.hamming == 0
                 and np.isfinite(detection.decision_margin) and np.isfinite(detection.reprojection_error_px)
                 and detection.decision_margin >= float(tag.get("min_decision_margin", 30.0))
                 and 0 <= detection.reprojection_error_px <= float(tag.get("max_reprojection_error_px", 2.0))]
        if not valid:
            self._note_missing(record, now, "declared target absent or below observation quality threshold")
            if self.search_mode == "fixed_directions":
                return self._fixed_direction_search(frame, feedback, now)
            if self.search_mode == "full_rotation":
                if record.search_exhausted:
                    return self._decision("SEARCH_EXHAUSTED", "full visual rotation completed; stopped until target deadline")
                rate = min(self.search_rate, float(self.control.get("max_yaw_rate_rad_s", 0.35)))
                return self._decision("SEARCHING", "positive-yaw visual scan; rotation measured from robot feedback",
                                      command=RobotMotionCommand(yaw_rate_rad_s=rate))
            yaw_offset = float(np.arctan2(np.sin(feedback.rpy_rad[2] - self._search_origin),
                                         np.cos(feedback.rpy_rad[2] - self._search_origin)))
            if yaw_offset >= self.search_span:
                self._search_direction = -1.0
            elif yaw_offset <= -self.search_span:
                self._search_direction = 1.0
            rate = self._search_direction * min(self.search_rate, float(self.control.get("max_yaw_rate_rad_s", 0.35)))
            if self.search_span == 0:
                rate = 0.0
            return self._decision("SEARCHING", "bounded visual search for the declared target",
                                  command=RobotMotionCommand(yaw_rate_rad_s=rate))
        if record.state == "LOST" or record.missing_since is not None:
            self._reset_dwell()
            self._transition(record, "ACTIVE", "target reacquired; stable dwell restarted", now)
        record.last_seen, record.missing_since = float(now), None
        if self.search_mode == "fixed_directions":
            self._search_goal, self._cached_probe_center = None, None
            self._cached_probe_index = 0
            self._pause_probe(reset=True)
            record.search_exhausted = False
        decision = self._controller.update(frame, valid, feedback, now)
        if decision.should_fire:
            self._armed = _FireToken(record.id, float(now), float(frame.timestamp), int(ammo_remaining),
                                     self.round_index, record.attempts + 1)
        return self._decision(decision.state, decision.reason, command=decision.command,
                              geometry=decision.geometry, fire=decision.should_fire)

    def authorize_shot(self, intended_id: int, now: float, ammo_before: int, *, frame_timestamp=None) -> bool:
        """Read-only check before a physical debit; stale decisions spend nothing."""
        token = self._armed
        if (isinstance(intended_id, (bool, np.bool_)) or token is None or self.done
                or intended_id != token.target_id or intended_id != self.active_target_id):
            return False
        max_age = float(self.control.get("max_frame_age_s", 0.15))
        if not np.isfinite(now) or now < token.now or not 0 <= now - token.frame_timestamp <= max_age:
            return False
        if (isinstance(ammo_before, (bool, np.bool_)) or not isinstance(ammo_before, (int, np.integer))
                or ammo_before <= 0 or ammo_before != token.ammo_before
                or token.round_index != self.round_index
                or token.target_attempt != self._records[intended_id].attempts + 1
                or (frame_timestamp is not None and (not np.isfinite(frame_timestamp)
                    or abs(frame_timestamp-token.frame_timestamp) > 1e-9))):
            return False
        return True

    def record_shot(self, intended_id: int, result: ShotResult, now: float, ammo_remaining: int) -> bool:
        if (isinstance(ammo_remaining, (bool, np.bool_)) or not isinstance(ammo_remaining, (int, np.integer))
                or ammo_remaining < 0 or not self.authorize_shot(intended_id, now, ammo_remaining+1)):
            return False
        token = self._armed
        self._armed = None
        record = self._records[intended_id]
        record.attempts += 1
        self._ammo_remaining = int(ammo_remaining)
        correct_hit = bool(result.hit and result.target_id == intended_id)
        record.successful_hits += int(correct_hit)
        credited_ring = int(result.ring) if correct_hit else 0
        record.ring = credited_ring
        reason = "matching_target_hit" if correct_hit else "wrong_target_hit" if result.target_id is not None and result.target_id != intended_id else result.reason or "miss"
        self._shots.append({
            "shot_index": len(self._shots) + 1,
            "round_index": token.round_index,
            "target_attempt": record.attempts,
            "timestamp": float(now),
            "intended_target_id": int(intended_id),
            "actual_target_id": result.target_id,
            "hit": bool(result.hit),
            "correct_target_hit": correct_hit,
            "ring": int(result.ring),
            "credited_ring": credited_ring,
            "point_world_m": _plain(result.point_world_m),
            "reason": reason,
            "referee_reason": result.reason,
            "ammo_before": token.ammo_before,
            "ammo_remaining": int(ammo_remaining),
        })
        self._controller.mark_fired()
        self._end_active("COMPLETED" if correct_hit else "FAILED", reason, now)
        if ammo_remaining == 0 and not self.done:
            for pending in self._records.values():
                if pending.state not in ("COMPLETED", "FAILED"):
                    self._transition(pending, "FAILED", "no_ammo", now)
            self._status = "NO_AMMO"
        return True

    def _close_active_visit(self, now: float, reason: str) -> None:
        """Finish only the open visit when the entire run is terminating."""
        if self.active_target_id is None:
            return
        visits = self._records[self.active_target_id].visits
        if visits and visits[-1]["finished_at_s"] is None:
            visits[-1].update(finished_at_s=float(now), state="FAILED",
                              reason=reason, shot_index=None)

    def finish_timeout(self, now: float) -> None:
        """Bound the entire run without discarding already completed targets."""
        if self.done:
            return
        self._close_active_visit(now, "global_timeout")
        self._reset_dwell()
        for record in self._records.values():
            if record.state not in ("COMPLETED", "FAILED"):
                self._transition(record, "FAILED", "global_timeout", now)
        self.active_target_id, self._controller = None, None
        if self.shooting_policy == "round_robin":
            self._status = "PARTIAL" if any(record.successful_hits for record in self._records.values()) else "FAILED"
            self.termination_reason = "global_timeout"
            return
        self._finish_if_terminal()

    def cancel(self, now: float, reason: str = "cancelled by operator") -> None:
        if self.done:
            return
        self._close_active_visit(now, reason)
        self._reset_dwell()
        for record in self._records.values():
            if record.state not in ("COMPLETED", "FAILED"):
                self._transition(record, "FAILED", reason, now)
        self.active_target_id, self._controller = None, None
        self._status = "CANCELLED"
        self.termination_reason = str(reason)
