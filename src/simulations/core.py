"""Core simulation loop and results container."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np

from src.faults import FaultInjector


@dataclass
class SimulationResults:
    """Stores time histories for one simulation run."""

    t: np.ndarray
    state: np.ndarray
    control: np.ndarray
    reference: np.ndarray
    errors: np.ndarray
    rotor_efficiency: np.ndarray
    controller_name: str
    fault_name: str = "none"
    metadata: Dict[str, str] = field(default_factory=dict)

    def save_npz(self, path: str) -> None:
        """Save all arrays to a NumPy archive."""
        path_obj = Path(path)
        path_obj.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path_obj,
            t=self.t,
            state=self.state,
            control=self.control,
            reference=self.reference,
            errors=self.errors,
            rotor_efficiency=self.rotor_efficiency,
        )

    def save_csv(self, path: str) -> None:
        """Save a flattened CSV view for quick inspection."""
        path_obj = Path(path)
        path_obj.parent.mkdir(parents=True, exist_ok=True)
        headers = [
            "t",
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "roll",
            "pitch",
            "yaw",
            "p",
            "q",
            "r",
        ]
        headers += [f"u_{idx}" for idx in range(self.control.shape[1])]
        headers += ["ref_x", "ref_y", "ref_z", "ref_yaw"]
        headers += ["err_x", "err_y", "err_z", "err_yaw"]
        headers += [f"eff_{idx}" for idx in range(self.rotor_efficiency.shape[1])]

        with path_obj.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(headers)
            for idx in range(len(self.t)):
                row = [self.t[idx]]
                row.extend(self.state[idx].tolist())
                row.extend(self.control[idx].tolist())
                row.extend(self.reference[idx].tolist())
                row.extend(self.errors[idx].tolist())
                row.extend(self.rotor_efficiency[idx].tolist())
                writer.writerow(row)


def _rk4_step(dynamics_fn: Callable, time_s: float, state: np.ndarray, commands: np.ndarray, dt: float) -> np.ndarray:
    k1 = dynamics_fn(time_s, state, commands)
    k2 = dynamics_fn(time_s + 0.5 * dt, state + 0.5 * dt * k1, commands)
    k3 = dynamics_fn(time_s + 0.5 * dt, state + 0.5 * dt * k2, commands)
    k4 = dynamics_fn(time_s + dt, state + dt * k3, commands)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_experiment(model, controller, fault_profile: Optional[FaultInjector], trajectory: Callable[[float], Dict], initial_state: np.ndarray, duration: float, dt: float) -> SimulationResults:
    """Run a closed-loop simulation experiment.

    Args:
        model: VTOL dynamics model.
        controller: Controller implementing compute_controls.
        fault_profile: Fault injector or None.
        trajectory: Callable returning a reference dictionary.
        initial_state: Initial 12-state vector.
        duration: Total simulation duration in seconds.
        dt: Integration and controller step.

    Returns:
        SimulationResults object with time histories.
    """
    fault_profile = fault_profile or FaultInjector()
    times = np.arange(0.0, duration + dt, dt)
    state = np.asarray(initial_state, dtype=float).copy()

    state_log = np.zeros((len(times), len(state)), dtype=float)
    control_log = np.zeros((len(times), model.rotor_count), dtype=float)
    reference_log = np.zeros((len(times), 4), dtype=float)
    error_log = np.zeros((len(times), 4), dtype=float)
    efficiency_log = np.zeros((len(times), model.rotor_count), dtype=float)

    controller.reset()

    for idx, time_s in enumerate(times):
        reference = trajectory(time_s)
        sensed_state = fault_profile.apply_sensor_faults(time_s, state)
        efficiency = fault_profile.rotor_efficiencies(time_s, model.rotor_count)
        if hasattr(controller, "update_rotor_efficiency_estimate"):
            controller.update_rotor_efficiency_estimate(efficiency, time_s)
        commands = controller.compute_controls(sensed_state, reference, t=time_s, dt=dt)
        faulted_commands = np.clip(commands * efficiency, 0.0, model.max_thrusts)

        ref_position = np.asarray(reference.get("position", np.zeros(3)), dtype=float)
        ref_yaw = float(reference.get("yaw", 0.0))
        current_yaw = state[8]
        yaw_error = np.arctan2(np.sin(ref_yaw - current_yaw), np.cos(ref_yaw - current_yaw))

        state_log[idx] = state
        control_log[idx] = faulted_commands
        reference_log[idx] = np.array([ref_position[0], ref_position[1], ref_position[2], ref_yaw], dtype=float)
        error_log[idx] = np.array([
            ref_position[0] - state[0],
            ref_position[1] - state[1],
            ref_position[2] - state[2],
            yaw_error,
        ])
        efficiency_log[idx] = efficiency

        if idx < len(times) - 1:
            state = _rk4_step(model.derivatives, time_s, state, faulted_commands, dt)

    return SimulationResults(
        t=times,
        state=state_log,
        control=control_log,
        reference=reference_log,
        errors=error_log,
        rotor_efficiency=efficiency_log,
        controller_name=controller.__class__.__name__,
        fault_name=fault_profile.name,
    )
