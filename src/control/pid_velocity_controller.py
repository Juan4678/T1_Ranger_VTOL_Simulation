"""Baseline velocity-and-attitude PID controller."""

from __future__ import annotations

from typing import Dict

import numpy as np

from .base_controller import BaseController


class PIDVelocityController(BaseController):
    """Simple nested PID controller inspired by ArduPilot-like hover control."""

    def __init__(self, model, controller_config: Dict, log_enabled: bool = False):
        super().__init__(model, log_enabled=log_enabled)
        gains = controller_config["controllers"]["pid"]
        self.vel_kp = np.asarray(gains["velocity_gains"]["kp"], dtype=float)
        self.vel_ki = np.asarray(gains["velocity_gains"]["ki"], dtype=float)
        self.vel_kd = np.asarray(gains["velocity_gains"]["kd"], dtype=float)
        self.att_kp = np.asarray(gains["attitude_gains"]["kp"], dtype=float)
        self.att_ki = np.asarray(gains["attitude_gains"]["ki"], dtype=float)
        self.att_kd = np.asarray(gains["attitude_gains"]["kd"], dtype=float)
        self.tilt_limit = float(gains["limits"].get("tilt_rad", 0.35))
        self.collective_margin = float(gains["limits"].get("collective_margin_n", 20.0))
        self.vel_integral = np.zeros(3, dtype=float)
        self.att_integral = np.zeros(3, dtype=float)
        self.prev_vel_error = np.zeros(3, dtype=float)

    def reset(self) -> None:
        super().reset()
        self.vel_integral[:] = 0.0
        self.att_integral[:] = 0.0
        self.prev_vel_error[:] = 0.0

    def compute_controls(self, state: np.ndarray, reference: Dict, t: float = 0.0, dt: float = 0.01) -> np.ndarray:
        pos = state[0:3]
        vel = state[3:6]
        euler = state[6:9]
        rates = state[9:12]

        ref_pos = np.asarray(reference.get("position", pos), dtype=float)
        ref_vel = np.asarray(reference.get("velocity", np.zeros(3)), dtype=float)
        ref_yaw = float(reference.get("yaw", 0.0))

        position_feedback = 0.2 * (ref_pos - pos)
        velocity_error = (ref_vel + position_feedback) - vel
        self.vel_integral += velocity_error * dt
        velocity_derivative = (velocity_error - self.prev_vel_error) / max(dt, 1e-6)
        self.prev_vel_error = velocity_error

        desired_acc = (
            self.vel_kp * velocity_error
            + self.vel_ki * self.vel_integral
            + self.vel_kd * velocity_derivative
        )

        gravity = self.model.params.gravity_mps2
        desired_roll = np.clip(-desired_acc[1] / max(gravity, 1e-6), -self.tilt_limit, self.tilt_limit)
        desired_pitch = np.clip(desired_acc[0] / max(gravity, 1e-6), -self.tilt_limit, self.tilt_limit)
        desired_yaw = ref_yaw
        desired_attitude = np.array([desired_roll, desired_pitch, desired_yaw], dtype=float)

        attitude_error = desired_attitude - euler
        attitude_error[2] = np.arctan2(np.sin(attitude_error[2]), np.cos(attitude_error[2]))
        self.att_integral += attitude_error * dt
        rate_error = -rates

        desired_torques = (
            self.att_kp * attitude_error
            + self.att_ki * self.att_integral
            + self.att_kd * rate_error
        )

        mass = self.model.params.mass_kg
        collective = mass * (gravity + desired_acc[2])
        max_collective = sum(self.model.max_thrusts[idx] for idx in self.model.hover_indices)
        collective = np.clip(collective, 0.0, max_collective + self.collective_margin)

        commands = self.model.allocate_hover_wrench(collective, desired_torques)
        self._log(
            {
                "t": t,
                "position": pos.copy(),
                "reference": ref_pos.copy(),
                "attitude_error": attitude_error.copy(),
                "commands": commands.copy(),
            }
        )
        return commands
