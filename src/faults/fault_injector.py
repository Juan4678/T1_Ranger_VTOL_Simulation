"""Actuator and sensor fault injection framework."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

import numpy as np


@dataclass
class RotorFaultEvent:
    """Defines a rotor-efficiency fault over a time interval."""

    rotor_indices: Sequence[int]
    start_time: float
    duration: Optional[float]
    efficiency: float

    def is_active(self, time_s: float) -> bool:
        if time_s < self.start_time:
            return False
        if self.duration is None:
            return True
        return time_s <= self.start_time + self.duration


@dataclass
class SensorFaultEvent:
    """Defines additive bias and noise on a subset of state indices."""

    state_indices: Sequence[int]
    start_time: float
    duration: Optional[float]
    bias: Sequence[float]
    noise_std: Sequence[float]

    def is_active(self, time_s: float) -> bool:
        if time_s < self.start_time:
            return False
        if self.duration is None:
            return True
        return time_s <= self.start_time + self.duration


class FaultInjector:
    """Applies actuator degradation and sensor corruption during simulations."""

    def __init__(self, actuator_faults: Optional[List[RotorFaultEvent]] = None, sensor_faults: Optional[List[SensorFaultEvent]] = None, name: str = "none", random_seed: int = 7):
        self.actuator_faults = actuator_faults or []
        self.sensor_faults = sensor_faults or []
        self.name = name
        self.rng = np.random.default_rng(random_seed)

    @classmethod
    def from_dict(cls, profile: Optional[dict], rotor_count: int, random_seed: int = 7) -> "FaultInjector":
        if not profile:
            return cls(name="none", random_seed=random_seed)
        actuator_faults = []
        for entry in profile.get("actuator_faults", []):
            rotor_indices = entry.get("rotor_indices", [])
            for rotor_index in rotor_indices:
                if rotor_index < 0 or rotor_index >= rotor_count:
                    raise ValueError(f"Rotor index {rotor_index} is outside 0..{rotor_count - 1}.")
            actuator_faults.append(
                RotorFaultEvent(
                    rotor_indices=rotor_indices,
                    start_time=float(entry["start_time"]),
                    duration=None if entry.get("duration") is None else float(entry["duration"]),
                    efficiency=float(entry.get("efficiency", 0.0)),
                )
            )
        sensor_faults = [
            SensorFaultEvent(
                state_indices=entry.get("state_indices", []),
                start_time=float(entry["start_time"]),
                duration=None if entry.get("duration") is None else float(entry["duration"]),
                bias=entry.get("bias", [0.0] * len(entry.get("state_indices", []))),
                noise_std=entry.get("noise_std", [0.0] * len(entry.get("state_indices", []))),
            )
            for entry in profile.get("sensor_faults", [])
        ]
        return cls(actuator_faults=actuator_faults, sensor_faults=sensor_faults, name=profile.get("name", "custom"), random_seed=random_seed)

    def rotor_efficiencies(self, time_s: float, rotor_count: int) -> np.ndarray:
        efficiencies = np.ones(rotor_count, dtype=float)
        for event in self.actuator_faults:
            if event.is_active(time_s):
                for rotor_index in event.rotor_indices:
                    efficiencies[rotor_index] *= event.efficiency
        return efficiencies

    def apply_actuator_faults(self, time_s: float, commands: Sequence[float]) -> np.ndarray:
        commands = np.asarray(commands, dtype=float)
        return commands * self.rotor_efficiencies(time_s, len(commands))

    def apply_sensor_faults(self, time_s: float, state: Sequence[float]) -> np.ndarray:
        state = np.asarray(state, dtype=float).copy()
        for event in self.sensor_faults:
            if event.is_active(time_s):
                idx = np.asarray(event.state_indices, dtype=int)
                bias = np.asarray(event.bias, dtype=float)
                std = np.asarray(event.noise_std, dtype=float)
                noise = self.rng.normal(0.0, std, size=len(idx))
                state[idx] += bias + noise
        return state
