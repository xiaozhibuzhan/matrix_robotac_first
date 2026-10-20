"""Nearest physical ray hit, finite ammunition, and an auditable shot ledger."""
from __future__ import annotations

import mujoco
import numpy as np

from .geometry import normalized
from .interfaces import ShotResult
from .multi_config import expand_target
from .scoring import ring_for_point


class MultiRayScorer:
    def __init__(self, model, data, config):
        self.model, self.data, self.config = model, data, config
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if self.base_id < 0:
            raise ValueError("referee model requires base_link")
        self.mission_ids = {int(spec["id"]) for spec in config["targets"]}
        self.boards = {}
        for spec in config["targets"] + config.get("distractors", []):
            if spec.get("tag_only", False):
                continue  # Visual distractors have no scoring surface or bullseye.
            identity = int(spec["id"])
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"task3_target_{identity}")
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"task3_target_collision_{identity}")
            if body < 0 or geom < 0:
                raise ValueError(f"referee cannot find physical target {identity}")
            self.boards[geom] = (identity, body, expand_target(config, spec)["simulation"])
        self.ammo_remaining = int(config["simulation"]["ammo_count"])
        self.last_scores = {}
        self.shots = []
        self.ledger = []
        self.initialized = False

    @property
    def scores(self):
        return self.last_scores

    def shooting_init(self) -> int:
        self.initialized = True
        return 0

    def shooting_reload(self) -> int:
        return 0 if self.initialized and self.ammo_remaining > 0 else -1

    def shooting_fire(self, intended_id: int) -> ShotResult:
        if isinstance(intended_id, bool) or not isinstance(intended_id, (int, np.integer)) or intended_id not in self.mission_ids:
            return ShotResult(None, 0, False, reason="unknown_intended_target")
        if not self.initialized:
            return ShotResult(None, 0, False, reason="not_initialized")
        if self.ammo_remaining <= 0:
            return ShotResult(None, 0, False, reason="no_ammo")
        before = self.ammo_remaining
        self.ammo_remaining -= 1
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        origin = self.data.xpos[self.base_id] + rotation @ np.asarray(self.config["laser"]["origin_base_m"], dtype=float)
        direction = rotation @ normalized(self.config["laser"]["direction_base"])
        geom_id = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(self.model, self.data, origin, direction,
                                 np.array([1, 0, 1, 0, 1, 1], dtype=np.uint8), 1, -1, geom_id)
        point = origin + distance * direction if distance >= 0 else None
        actual_ring = None
        if int(geom_id[0]) not in self.boards:
            result = ShotResult(None, 3, False, point, "occluded" if distance >= 0 else "no_surface")
        else:
            identity, body, simulation = self.boards[int(geom_id[0])]
            local = self.data.xmat[body].reshape(3, 3).T @ (point - self.data.xpos[body])
            actual_ring = ring_for_point(local[:2], simulation)
            if identity not in self.mission_ids:
                result = ShotResult(identity, actual_ring, False, point, "distractor")
            else:
                reason = "wrong_target" if identity != intended_id else ("ring_hit" if actual_ring >= 6 else "outside_rings")
                result = ShotResult(identity, actual_ring, actual_ring >= 6, point, reason)
                # Only the board actually intersected earns/loses score. A miss
                # has no physical target identity; it is kept in the ledger.
                self.last_scores[identity] = actual_ring
        self.shots.append(result)
        self.ledger.append({"sequence": len(self.ledger) + 1, "intended_target_id": int(intended_id),
                            "actual_target_id": result.target_id, "ring": result.ring, "hit": result.hit,
                            "reason": result.reason, "ammo_before": before, "ammo_after": self.ammo_remaining,
                            "point_world_m": None if point is None else point.tolist(),
                            "physical_ring": actual_ring, "timestamp_s": float(self.data.time)})
        return result

    def shooting_shutdown(self) -> None:
        self.initialized = False
