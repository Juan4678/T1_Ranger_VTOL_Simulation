"""Hover-linearized LQR controller for baseline comparisons."""

from __future__ import annotations

from typing import Dict

import numpy as np
from scipy.linalg import solve_continuous_are

from .base_controller import BaseController


class BaselineLQRController(BaseController):
    """Continuous-time LQR around hover for velocity and attitude regulation."""

    def __init__(self, model, controller_config: Dict, log_enabled: bool = False):
        super().__init__(model, log_enabled=log_enabled)
        lqr_config = controller_config["controllers"]["lqr"]
        self.Q = np.diag(np.asarray(lqr_config["Q_diag"], dtype=float))
        self.R = np.diag(np.asarray(lqr_config["R_diag"], dtype=float))
        self.K = self._compute_gain_matrix()

    def _compute_gain_matrix(self) -> np.ndarray:
        g = self.model.params.gravity_mps2
        m = self.model.params.mass_kg
        ix, iy, iz = self.model.params.inertia_kgm2

        a = np.zeros((9, 9), dtype=float)
        a[0, 4] = g
        a[1, 3] = -g
        a[3, 6] = 1.0
        a[4, 7] = 1.0
        a[5, 8] = 1.0

        b = np.zeros((9, 4), dtype=float)
        b[2, 0] = 1.0 / m
        b[6, 1] = 1.0 / ix
        b[7, 2] = 1.0 / iy
        b[8, 3] = 1.0 / iz

        p = solve_continuous_are(a, b, self.Q, self.R)
        return np.linalg.solve(self.R, b.T @ p)

    def compute_controls(self, state: np.ndarray, reference: Dict, t: float = 0.0, dt: float = 0.01) -> np.ndarray:
        _ = dt
        vel = state[3:6]
        euler = state[6:9]
        rates = state[9:12]

        ref_vel = np.asarray(reference.get("velocity", np.zeros(3)), dtype=float)
        ref_yaw = float(reference.get("yaw", 0.0))

        yaw_error = np.arctan2(np.sin(euler[2] - ref_yaw), np.cos(euler[2] - ref_yaw))
        state_error = np.array(
            [
                vel[0] - ref_vel[0],
                vel[1] - ref_vel[1],
                vel[2] - ref_vel[2],
                euler[0],
                euler[1],
                yaw_error,
                rates[0],
                rates[1],
                rates[2],
            ],
            dtype=float,
        )

        control_delta = -self.K @ state_error
        collective = self.model.params.mass_kg * self.model.params.gravity_mps2 + control_delta[0]
        collective = np.clip(collective, 0.0, np.sum(self.model.max_thrusts[self.model.hover_indices]))
        commands = self.model.allocate_hover_wrench(collective, control_delta[1:4])
        self._log({"t": t, "state_error": state_error.copy(), "commands": commands.copy()})
        return commands
