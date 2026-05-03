"""Lightweight gain tuning wrapper around scipy.optimize."""

from __future__ import annotations

from typing import Callable, Sequence

import numpy as np
from scipy.optimize import minimize, OptimizeResult


def tune_gains(dynamics_fn: Callable, cost_fn: Callable, initial_guess: Sequence[float], bounds: Sequence[tuple], method: str = "L-BFGS-B") -> OptimizeResult:
    """Tune controller gains by minimizing a user-defined cost.

    Args:
        dynamics_fn: Callable that runs a simulation or returns experiment data.
        cost_fn: Callable with signature cost_fn(dynamics_fn, gains) -> float.
        initial_guess: Initial gain vector.
        bounds: Bounds per gain element.
        method: Optimization method supported by scipy.optimize.minimize.

    Returns:
        SciPy optimization result.
    """

    initial_guess = np.asarray(initial_guess, dtype=float)

    def objective(gains: np.ndarray) -> float:
        return float(cost_fn(dynamics_fn, np.asarray(gains, dtype=float)))

    return minimize(objective, initial_guess, bounds=bounds, method=method)
