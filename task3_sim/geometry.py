"""Camera/tag/laser geometry used only from measured image observations.

OpenCV and AprilTag camera coordinates are +X right, +Y down, +Z forward.
AprilTag's object coordinates are +X right, +Y down, +Z into the tag. A
bullseye below its marker therefore has a positive object-frame Y offset.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


DEFAULT_ROTATION_BASE_FROM_CAMERA = np.array(
    [[0.0, 0.258819045102521, 0.965925826289068],
     [-1.0, 0.0, 0.0], [0.0, -0.965925826289068, 0.258819045102521]]
)


def vector3(value: Any, name: str = "vector") -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


def rotation3(value: Any, name: str = "rotation") -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(result)) or not np.allclose(
        result.T @ result, np.eye(3), atol=1e-5
    ) or not np.isclose(np.linalg.det(result), 1.0, atol=1e-5):
        raise ValueError(f"{name} must be a right-handed orthonormal 3x3 rotation")
    return result


def normalized(value: Any, name: str = "direction") -> np.ndarray:
    result = vector3(value, name)
    length = np.linalg.norm(result)
    if length < 1e-10:
        raise ValueError(f"{name} cannot be zero")
    return result / length


def camera_extrinsics(config: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    rotation = rotation3(config.get("rotation_base_from_camera", DEFAULT_ROTATION_BASE_FROM_CAMERA))
    position = vector3(config.get("position_base_m", [0.30, 0.0, -0.05185]))
    return rotation, position


def ray_plane_intersection(
    origin: Any, direction: Any, plane_point: Any, plane_normal: Any
) -> tuple[np.ndarray, float] | None:
    """Return the forward ray intersection and distance, or None if unavailable."""
    origin = vector3(origin)
    direction = normalized(direction)
    plane_point = vector3(plane_point)
    normal = normalized(plane_normal)
    denominator = float(np.dot(direction, normal))
    if abs(denominator) < 1e-8:
        return None
    distance = float(np.dot(plane_point - origin, normal) / denominator)
    if distance <= 0:
        return None
    return origin + distance * direction, distance


@dataclass(frozen=True)
class AimGeometry:
    bullseye_base_m: np.ndarray
    plane_normal_base: np.ndarray
    laser_intersection_base_m: np.ndarray | None
    error_tag_xy_m: np.ndarray
    error_norm_m: float
    yaw_error_rad: float
    pitch_error_rad: float
    range_m: float
    reachable: bool


def estimate_aim(
    detection: Any,
    camera_config: Mapping[str, Any],
    laser_config: Mapping[str, Any],
    tag_config: Mapping[str, Any],
) -> AimGeometry:
    """Estimate desired aiming correction, without accessing scene truth.

    Plane intersection gives the physical miss distance. Angular correction
    compares the current and desired beam direction in the robot base frame.
    Positive body yaw turns left; positive body pitch points the beam down.
    """
    rotation_base_camera, camera_position = camera_extrinsics(camera_config)
    rotation_camera_tag = rotation3(detection.R_camera_from_tag)
    tag_position_camera = vector3(detection.t_camera_from_tag)
    rotation_base_tag = rotation_base_camera @ rotation_camera_tag
    tag_position_base = rotation_base_camera @ tag_position_camera + camera_position
    offset_tag = vector3(tag_config.get("bullseye_offset_tag_m", [0.0, 0.20, 0.0]))
    bullseye = rotation_base_tag @ offset_tag + tag_position_base
    normal = rotation_base_tag[:, 2]
    origin = vector3(laser_config.get("origin_base_m", [0.0, 0.0, 0.0601224966]))
    direction = normalized(laser_config.get("direction_base", [0.9995155775, 0.0210760583, 0.0229000022]))
    desired = bullseye - origin
    distance = float(np.linalg.norm(desired))
    if distance < 1e-6:
        raise ValueError("bullseye cannot coincide with the laser origin")
    desired /= distance
    desired_yaw = np.arctan2(desired[1], desired[0])
    beam_yaw = np.arctan2(direction[1], direction[0])
    yaw_error = float(np.arctan2(np.sin(desired_yaw - beam_yaw), np.cos(desired_yaw - beam_yaw)))
    pitch_error = float(
        np.arctan2(-desired[2], np.hypot(desired[0], desired[1]))
        - np.arctan2(-direction[2], np.hypot(direction[0], direction[1]))
    )
    intersection = ray_plane_intersection(origin, direction, bullseye, normal)
    if intersection is None:
        hit, error_xy, error_norm = None, np.full(2, np.inf), float("inf")
    else:
        hit, _ = intersection
        error_xy = (rotation_base_tag.T @ (bullseye - hit))[:2]
        error_norm = float(np.linalg.norm(error_xy))
    return AimGeometry(
        bullseye, normal, hit, error_xy, error_norm, yaw_error, pitch_error,
        distance, bool(desired[0] > 0 and intersection is not None),
    )
