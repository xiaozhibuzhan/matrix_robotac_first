"""Independent ray-based referee. Ground truth is confined to this module."""

from __future__ import annotations

import mujoco
import numpy as np

from .geometry import normalized
from .interfaces import ShotResult


def ring_for_radius(radius: float, radii) -> int:
    """Synthetic configurable rings; exactly on a line receives the inner ring."""
    if not np.isfinite(radius) or radius < 0:
        raise ValueError("radius must be finite and nonnegative")
    for ring, limit in zip(range(10, 5, -1), radii):
        if radius <= float(limit) + 1e-10:
            return ring
    return 3


def validate_ring_ellipses(simulation):
    """Validate optional image-calibrated printed ring conics and their nesting."""
    polygon = simulation.get("scoring_polygon_m")
    if polygon is not None:
        polygon = np.asarray(polygon, dtype=float)
        if polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 64 or not np.all(np.isfinite(polygon)):
            raise ValueError("scoring_polygon_m needs 3..64 finite 2D vertices")
        half_size = np.array([simulation["board_width_m"], simulation["board_height_m"]])/2
        board_center = np.array([0, simulation["board_center_down_m"]])
        if np.any(np.abs(polygon-board_center) > half_size+1e-8):
            raise ValueError("scoring polygon must fit the board")
        area_twice = np.sum(polygon[:, 0]*np.roll(polygon[:, 1], -1)-polygon[:, 1]*np.roll(polygon[:, 0], -1))
        if abs(area_twice) <= 1e-10:
            raise ValueError("scoring polygon must enclose an area")
    rings = simulation.get("ring_ellipses_m")
    if rings is None:
        return
    if not isinstance(rings, list) or len(rings) != 5:
        raise ValueError("ring_ellipses_m requires five ellipses for rings 10 through 6")
    previous_boundary = None
    angles = np.linspace(0, 2*np.pi, 257)
    unit_circle = np.array([np.cos(angles), np.sin(angles)])
    for ring in rings:
        center = np.asarray(ring["center_offset_m"], dtype=float)
        quadratic = np.asarray(ring["quadratic_m_inv2"], dtype=float)
        if center.shape != (2,) or quadratic.shape != (2, 2) or not np.all(np.isfinite(center)) or not np.all(np.isfinite(quadratic)):
            raise ValueError("ring ellipse needs a finite 2D center and 2x2 quadratic")
        if not np.allclose(quadratic, quadratic.T, atol=1e-8, rtol=1e-10):
            raise ValueError("ring ellipse quadratic must be symmetric")
        eigenvalues, eigenvectors = np.linalg.eigh(quadratic)
        if np.any(eigenvalues <= 0):
            raise ValueError("ring ellipse quadratic must be positive definite")
        # Coordinate extremes are the square roots of the inverse diagonal.
        extent = np.sqrt(np.diag(np.linalg.inv(quadratic)))
        half_size = np.array([simulation["board_width_m"], simulation["board_height_m"]])/2
        board_center = np.array([0, simulation["board_center_down_m"]])
        if np.any(np.abs(center-board_center)+extent > half_size+1e-8):
            raise ValueError("printed scoring ellipse must fit the board")
        if previous_boundary is not None:
            delta = previous_boundary-center[:, None]
            if np.max(np.einsum("in,ij,jn->n", delta, quadratic, delta)) > 1+1e-7:
                raise ValueError("printed scoring ellipses must be nested")
        previous_boundary = center[:, None] + eigenvectors @ (unit_circle/np.sqrt(eigenvalues[:, None]))


def point_in_scoring_polygon(point, polygon):
    """Even/odd polygon containment with inclusive printed boundaries."""
    inside = False
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge, delta = end-start, point-start
        length_sq = float(edge @ edge)
        cross = float(edge[0]*delta[1]-edge[1]*delta[0])
        if length_sq > 0 and abs(cross) <= 1e-10 and -1e-10 <= float(delta @ edge) <= length_sq+1e-10:
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            intersection_x = start[0]+(point[1]-start[1])*edge[0]/edge[1]
            if point[0] < intersection_x:
                inside = not inside
    return inside


def ring_for_point(point, simulation):
    """Score a board-local point against calibrated artwork, or circular rings."""
    point = np.asarray(point, dtype=float)
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError("scoring point must be finite and 2D")
    polygon = simulation.get("scoring_polygon_m")
    if polygon is not None and not point_in_scoring_polygon(point, np.asarray(polygon, dtype=float)):
        return 3
    rings = simulation.get("ring_ellipses_m")
    if rings is None:
        return ring_for_radius(float(np.linalg.norm(point)), simulation["ring_radii_m"])
    for score, ellipse in zip(range(10, 5, -1), rings):
        delta = point - np.asarray(ellipse["center_offset_m"])
        quadratic = np.asarray(ellipse["quadratic_m_inv2"])
        if float(delta @ quadratic @ delta) <= 1+1e-10:
            return score
    return 3


class RayScorer:
    def __init__(self, model, data, config):
        self.model, self.data, self.config = model, data, config
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "task3_target")
        self.target_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "task3_target_collision")
        self.ammo_remaining = int(config["simulation"]["ammo_count"])
        self.last_scores = {}
        self.shots = []
        self.initialized = False

    def shooting_init(self) -> int:
        self.initialized = True
        return 0

    def shooting_reload(self) -> int:
        # No unlimited replenishment: reserved hardware interface only.
        return 0 if self.initialized and self.ammo_remaining > 0 else -1

    def shooting_fire(self) -> ShotResult:
        if not self.initialized:
            return ShotResult(None, 0, False, reason="not_initialized")
        if self.ammo_remaining <= 0:
            return ShotResult(None, 0, False, reason="no_ammo")
        self.ammo_remaining -= 1
        rotation = self.data.xmat[self.base_id].reshape(3,3)
        origin = self.data.xpos[self.base_id] + rotation @ np.array(self.config["laser"]["origin_base_m"])
        direction = rotation @ normalized(self.config["laser"]["direction_base"])
        geom_id = np.array([-1], dtype=np.int32)
        # Exclude visual-only geometry (1) and the shooter's full subtree (3).
        distance = mujoco.mj_ray(self.model, self.data, origin, direction,
                                 np.array([1,0,1,0,1,1], dtype=np.uint8), 1, -1, geom_id)
        point = origin + distance*direction if distance >= 0 else None
        if geom_id[0] != self.target_geom_id:
            result = ShotResult(None, 3, False, point, "occluded" if distance >= 0 else "no_surface")
        else:
            local = self.data.xmat[self.target_body_id].reshape(3,3).T @ (point-self.data.xpos[self.target_body_id])
            ring = ring_for_point(local[:2], self.config["simulation"])
            result = ShotResult(int(self.config["tag"]["id"]), ring, ring >= 6, point, "ring_hit" if ring >= 6 else "outside_rings")
            # This is the last shot, never max(previous, new).
            self.last_scores[result.target_id] = ring
        # Single-target policy: every spent shot is assigned to the intended
        # ID, including off-board/occluded misses. Competition ambiguity remains
        # configurable in a later multi-target referee, not silently guessed.
        self.last_scores[int(self.config["tag"]["id"])] = result.ring
        self.shots.append(result)
        return result

    def shooting_shutdown(self) -> None:
        self.initialized = False
