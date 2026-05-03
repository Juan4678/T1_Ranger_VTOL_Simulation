"""Abstract controller interface shared by all controllers."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List

import numpy as np


class BaseController(ABC):
    """Base class for controllers that output per-rotor thrust commands."""

    def __init__(self, model, log_enabled: bool = False):
        self.model = model
        self.log_enabled = log_enabled
        self.history: List[Dict] = []

    @abstractmethod
    def compute_controls(self, state: np.ndarray, reference: Dict, t: float = 0.0, dt: float = 0.01) -> np.ndarray:
        """Compute rotor thrust commands.

        Args:
            state: Current vehicle state.
            reference: Reference dictionary with desired position, velocity, and yaw.
            t: Current time.
            dt: Controller sample time.

        Returns:
            Rotor thrusts in Newtons.
        """

    def reset(self) -> None:
        """Reset controller internal state."""
        self.history.clear()

    def _log(self, payload: Dict) -> None:
        if self.log_enabled:
            self.history.append(payload)
