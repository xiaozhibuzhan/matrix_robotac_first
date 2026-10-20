"""MuJoCo renderer and a constrained, explicitly kinematic robot adapter.

Original leg geometry/joints remain visible at their home positions. Base
commands have velocity lag and a watchdog, but are NOT a locomotion policy.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .interfaces import Frame, RobotFeedback, RobotMotionCommand
from .scene import quaternion, rpy_matrix
from .sensor import CameraEffects


class MujocoRobot:
    def __init__(self, model, config):
        self.model = model
        self.data = mujoco.MjData(model)
        self.config = config
        self.sim = config["simulation"]
        self.position = np.array(self.sim["initial_position_m"], dtype=float)
        self.rpy = np.array(self.sim["initial_rpy_rad"], dtype=float)
        self.velocity = np.zeros(4)  # body vx, vy, yaw rate, pitch rate
        self.command = RobotMotionCommand()
        self.command_time = 0.0
        self.connected = False
        self.standing = False
        self.renderer = None
        self.camera_effects = CameraEffects(config.get("appearance", {}).get("camera_effects"))
        self.render_option = mujoco.MjvOption()
        self.render_option.geomgroup[2] = 0
        self.render_option.geomgroup[3] = 1
        self._home = np.array(model.key_qpos[0]) if model.nkey else np.array(model.qpos0)
        self.data.qpos[:] = self._home
        self._sync()

    def Connect(self):
        self.connected = True

    def StandUp(self):
        if not self.connected:
            raise RuntimeError("Connect must precede StandUp")
        self.standing = True
        self._sync()

    def ReadFeedback(self):
        # Body angular velocity from Rz(yaw) Ry(pitch), roll held zero.
        yaw_rate, pitch_rate = self.velocity[2:]
        gyro = np.array([-math.sin(self.rpy[1])*yaw_rate, pitch_rate, math.cos(self.rpy[1])*yaw_rate])
        world_velocity = rpy_matrix([0,0,self.rpy[2]]) @ np.array([*self.velocity[:2],0.0])
        return RobotFeedback(self.connected and self.standing, self.rpy.copy(), self.position.copy(),
                             gyro, world_velocity, float(self.data.time))

    def Execute(self, command):
        if not self.connected or not self.standing:
            raise RuntimeError("robot is not initialized")
        values = np.array([command.vx_m_s, command.vy_m_s, command.yaw_rate_rad_s, command.pitch_rate_rad_s])
        if not np.all(np.isfinite(values)):
            self.Stop()
            raise ValueError("command contains nonfinite values")
        control = self.config["control"]
        self.command = RobotMotionCommand(*np.clip(values,
            [-0.15,-0.15,-control["max_yaw_rate_rad_s"],-control["max_pitch_rate_rad_s"]],
            [0.15,0.15,control["max_yaw_rate_rad_s"],control["max_pitch_rate_rad_s"]]))
        self.command_time = float(self.data.time)

    def Stop(self):
        self.command = RobotMotionCommand()
        self.command_time = float(self.data.time)

    def advance(self, duration):
        remaining = float(duration)
        while remaining > 1e-9:
            dt = min(remaining, self.sim["integration_step_s"])
            command = self.command
            if not self.connected or not self.standing or self.data.time-self.command_time > self.sim["command_timeout_s"]:
                command = RobotMotionCommand()
            desired = np.array([command.vx_m_s,command.vy_m_s,command.yaw_rate_rad_s,command.pitch_rate_rad_s])
            self.velocity += (desired-self.velocity)*(1-math.exp(-dt/self.sim["velocity_time_constant_s"]))
            self.position += rpy_matrix([0,0,self.rpy[2]]) @ np.array([self.velocity[0], self.velocity[1], 0])*dt
            self.rpy[2] += self.velocity[2]*dt
            pitch_limit = self.config["control"]["max_pitch_rad"]
            self.rpy[1] = np.clip(self.rpy[1]+self.velocity[3]*dt,-pitch_limit,pitch_limit)
            if abs(self.rpy[1]) >= pitch_limit and self.rpy[1]*self.velocity[3] > 0:
                self.velocity[3] = 0
            self.data.time += dt
            remaining -= dt
        self._sync()

    def _sync(self):
        self.data.qpos[:] = self._home
        self.data.qpos[:3] = self.position
        self.data.qpos[3:7] = quaternion(rpy_matrix(self.rpy))
        self.data.qvel[:] = 0
        self.data.qvel[:3] = rpy_matrix([0,0,self.rpy[2]]) @ np.array([*self.velocity[:2],0])
        yaw_rate, pitch_rate = self.velocity[2:]
        self.data.qvel[3:6] = [-math.sin(self.rpy[1])*yaw_rate,pitch_rate,math.cos(self.rpy[1])*yaw_rate]
        mujoco.mj_forward(self.model, self.data)

    def capture(self) -> Frame:
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=self.config["camera"]["height"], width=self.config["camera"]["width"])
        self.renderer.update_scene(self.data, camera="task3_camera", scene_option=self.render_option)
        return Frame(self.camera_effects.apply(self.renderer.render().copy()), float(self.data.time))

    def close(self):
        self.Stop()
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
