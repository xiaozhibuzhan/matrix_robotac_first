"""Decode the two pose metadata points embedded by the UE lidar driver.

The driver emits two leading points with ``intensity == 111``.  Their XYZ
values are metadata rather than returns: point 0 stores position in
centimetres as ``(x, -y, z)`` and point 1 stores Euler angles in degrees as
``(roll, -pitch, -yaw)``.  This module deliberately has no ROS dependency so
the same decoder can be used by online and bag finalization paths.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class EmbeddedPoseResult:
    """Decoded pose and cloud with metadata records removed."""

    # Pose is (px, py, pz, qx, qy, qz, qw), in metres and ROS XYZW order.
    pose: tuple[float, float, float, float, float, float, float] | None
    points: np.ndarray
    metadata_mask: np.ndarray
    reason: str


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    # Active ROS XYZ (roll, pitch, yaw) rotation, matching rotate_vector().
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def decode_embedded_pose(points, intensity, *, marker: float = 111.0) -> EmbeddedPoseResult:
    """Decode and strip embedded pose records from a local XYZ cloud.

    A result with ``pose is None`` gives a diagnostic ``reason``.  At least two
    finite marker records are required; all records carrying the marker
    intensity are removed from ``points`` even when the pair is incomplete.
    """
    xyz = np.asarray(points, dtype=np.float64)
    values = np.asarray(intensity, dtype=np.float64).reshape(-1)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(values) != len(xyz):
        raise ValueError("points must be an Nx3 array and intensity must have N values")
    metadata = np.isclose(values, float(marker), rtol=0.0, atol=1e-5)
    indices = np.flatnonzero(metadata)
    stripped = xyz[~metadata].copy()
    if len(indices) < 2:
        # Marker records are never lidar returns, even when the pair is
        # incomplete; remove any present marker before the caller's fallback.
        return EmbeddedPoseResult(None, stripped, metadata, "missing_embedded_pose")
    first, second = xyz[indices[0]], xyz[indices[1]]
    if not np.isfinite(first).all() or not np.isfinite(second).all():
        return EmbeddedPoseResult(None, stripped, metadata, "nonfinite_embedded_pose")
    # The stored Y/angle signs are part of the wire format (see module doc).
    position = (float(first[0]) * 0.01, -float(first[1]) * 0.01, float(first[2]) * 0.01)
    roll = math.radians(float(second[0]))
    pitch = math.radians(-float(second[1]))
    yaw = math.radians(-float(second[2]))
    if not all(math.isfinite(value) for value in (*position, roll, pitch, yaw)):
        return EmbeddedPoseResult(None, stripped, metadata, "nonfinite_embedded_pose")
    quaternion = _quaternion_from_rpy(roll, pitch, yaw)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        return EmbeddedPoseResult(None, stripped, metadata, "invalid_embedded_orientation")
    quaternion = tuple(value / norm for value in quaternion)
    pose = (*position, *quaternion)
    return EmbeddedPoseResult(pose, stripped, metadata, "ok")
