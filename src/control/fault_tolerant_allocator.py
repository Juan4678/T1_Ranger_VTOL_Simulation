"""Fault-tolerant control allocation for rotor-loss research.

This controller does not replace the baseline PID/LQR law. It wraps an existing
controller, asks it for the desired nominal rotor commands, converts those
commands into a hover wrench, and reallocates that wrench across the currently
healthy motors.
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from .base_controller import BaseController


class FaultTolerantAllocatorController(BaseController):
    """Reallocate a nominal hover wrench around degraded or disabled rotors.

    The current draft uses an oracle efficiency estimate supplied by the
    simulation loop. That gives a useful upper-bound benchmark. Later work can
    replace `update_rotor_efficiency_estimate()` with an estimator driven by
    IMU residuals, motor telemetry, or EKF innovation signals.
    """

    def __init__(
        self,
        model,
        nominal_controller: BaseController,
        controller_config: Dict,
        log_enabled: bool = False,
    ):
        super().__init__(model, log_enabled=log_enabled)
        self.nominal_controller = nominal_controller
        self.controller_config = controller_config
        robust_config = controller_config.get("controllers", {}).get("fault_tolerant", {})
        self.min_effective_efficiency = float(robust_config.get("min_effective_efficiency", 0.05))
        self.regularization = float(robust_config.get("regularization", 1.0e-4))
        self.command_smoothing = float(robust_config.get("command_smoothing", 0.35))
        self.rotor_efficiency_estimate = np.ones(model.rotor_count, dtype=float)
        self.previous_commands: Optional[np.ndarray] = None

    def reset(self) -> None:
        super().reset()
        self.nominal_controller.reset()
        self.rotor_efficiency_estimate[:] = 1.0
        self.previous_commands = None

    def update_rotor_efficiency_estimate(self, efficiencies: np.ndarray, time_s: float) -> None:
        """Update the allocator's current motor-health estimate."""
        _ = time_s
        efficiencies = np.asarray(efficiencies, dtype=float)
        if efficiencies.shape != (self.model.rotor_count,):
            raise ValueError("Rotor-efficiency estimate must match rotor count.")
        self.rotor_efficiency_estimate = np.clip(efficiencies, 0.0, 1.0)

    def compute_controls(self, state: np.ndarray, reference: Dict, t: float = 0.0, dt: float = 0.01) -> np.ndarray:
        nominal_commands = self.nominal_controller.compute_controls(state, reference, t=t, dt=dt)
        nominal_commands = np.clip(nominal_commands, 0.0, self.model.max_thrusts)

        allocation = self.model.hover_allocation_matrix()
        desired_wrench = allocation @ nominal_commands
        effective_allocation = allocation * self.rotor_efficiency_estimate[np.newaxis, :]

        healthy = self.rotor_efficiency_estimate > self.min_effective_efficiency
        if not np.any(healthy):
            commands = np.zeros(self.model.rotor_count, dtype=float)
        else:
            a_healthy = effective_allocation[:, healthy]
            rhs = desired_wrench
            if self.regularization > 0.0:
                reg = self.regularization * np.eye(a_healthy.shape[1])
                a_aug = np.vstack((a_healthy, reg))
                rhs = np.concatenate((rhs, np.zeros(a_healthy.shape[1])))
            else:
                a_aug = a_healthy

            solved, *_ = np.linalg.lstsq(a_aug, rhs, rcond=None)
            commands = np.zeros(self.model.rotor_count, dtype=float)
            commands[healthy] = solved
            commands = np.clip(commands, 0.0, self.model.max_thrusts)

        if self.previous_commands is not None and 0.0 < self.command_smoothing < 1.0:
            alpha = self.command_smoothing
            commands = alpha * commands + (1.0 - alpha) * self.previous_commands
        commands = np.clip(commands, 0.0, self.model.max_thrusts)
        self.previous_commands = commands.copy()

        achieved_wrench = effective_allocation @ commands
        wrench_error = desired_wrench - achieved_wrench
        self._log(
            {
                "t": t,
                "mode": "fault_tolerant_allocator",
                "efficiency": self.rotor_efficiency_estimate.copy(),
                "nominal_commands": nominal_commands.copy(),
                "commands": commands.copy(),
                "desired_wrench": desired_wrench.copy(),
                "achieved_wrench": achieved_wrench.copy(),
                "wrench_error": wrench_error.copy(),
            }
        )
        return commands
