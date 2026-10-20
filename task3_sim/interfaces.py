"""Small adapter contracts matching the supplied robot/shooting interfaces.

These types are deliberately independent of MuJoCo, ROS, and the official SDK.
Positions are metres, angles radians, and timestamps simulation seconds. The
base frame is right handed: +X forward, +Y left, +Z up.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray
    timestamp: float


@dataclass(frozen=True)
class RobotFeedback:
    valid: bool
    rpy_rad: np.ndarray
    position_m: np.ndarray
    gyro_rad_s: np.ndarray
    velocity_m_s: np.ndarray
    timestamp: float


@dataclass(frozen=True)
class RobotMotionCommand:
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    yaw_rate_rad_s: float = 0.0
    # Simulation milestone extension. A real adapter must explicitly implement
    # this through a supported body-attitude interface; it is not an SDK move().
    pitch_rate_rad_s: float = 0.0


@dataclass(frozen=True)
class ShotResult:
    target_id: int | None
    ring: int
    hit: bool
    point_world_m: np.ndarray | None = None
    reason: str = ""


@runtime_checkable
class RobotInterface(Protocol):
    def Connect(self) -> None: ...
    def StandUp(self) -> None: ...
    def ReadFeedback(self) -> RobotFeedback: ...
    def Execute(self, command: RobotMotionCommand) -> None: ...
    def Stop(self) -> None: ...


@runtime_checkable
class ShootingInterface(Protocol):
    def shooting_init(self) -> int: ...
    def shooting_reload(self) -> int: ...
    def shooting_fire(self) -> ShotResult: ...
    def shooting_shutdown(self) -> None: ...
