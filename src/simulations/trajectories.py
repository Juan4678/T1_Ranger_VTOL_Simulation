"""Reference generators for the experiment scripts."""

from __future__ import annotations

from typing import Callable, Dict, Sequence

import numpy as np


def hover_trajectory(position: Sequence[float] = (0.0, 0.0, 0.0), yaw: float = 0.0) -> Callable[[float], Dict]:
    ref_position = np.asarray(position, dtype=float)

    def _trajectory(time_s: float) -> Dict:
        _ = time_s
        return {"position": ref_position, "velocity": np.zeros(3), "yaw": yaw}

    return _trajectory


def line_trajectory(start: Sequence[float] = (0.0, 0.0, -2.0), end: Sequence[float] = (10.0, 0.0, -2.0), duration: float = 20.0, yaw: float = 0.0) -> Callable[[float], Dict]:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start

    def _trajectory(time_s: float) -> Dict:
        alpha = np.clip(time_s / max(duration, 1e-6), 0.0, 1.0)
        position = start + alpha * delta
        velocity = delta / max(duration, 1e-6)
        return {"position": position, "velocity": velocity, "yaw": yaw}

    return _trajectory


def circle_trajectory(radius: float = 6.0, altitude: float = -2.0, period: float = 20.0) -> Callable[[float], Dict]:
    omega = 2.0 * np.pi / max(period, 1e-6)

    def _trajectory(time_s: float) -> Dict:
        position = np.array([radius * np.cos(omega * time_s), radius * np.sin(omega * time_s), altitude], dtype=float)
        velocity = np.array([-radius * omega * np.sin(omega * time_s), radius * omega * np.cos(omega * time_s), 0.0], dtype=float)
        yaw = np.arctan2(velocity[1], max(np.linalg.norm(velocity[0:2]), 1e-6))
        return {"position": position, "velocity": velocity, "yaw": yaw}

    return _trajectory


def figure8_trajectory(amplitude: float = 5.0, altitude: float = -2.0, period: float = 24.0) -> Callable[[float], Dict]:
    omega = 2.0 * np.pi / max(period, 1e-6)

    def _trajectory(time_s: float) -> Dict:
        x = amplitude * np.sin(omega * time_s)
        y = amplitude * np.sin(omega * time_s) * np.cos(omega * time_s)
        vx = amplitude * omega * np.cos(omega * time_s)
        vy = amplitude * omega * (np.cos(2.0 * omega * time_s))
        position = np.array([x, y, altitude], dtype=float)
        velocity = np.array([vx, vy, 0.0], dtype=float)
        yaw = np.arctan2(vy, max(abs(vx), 1e-6))
        return {"position": position, "velocity": velocity, "yaw": yaw}

    return _trajectory
