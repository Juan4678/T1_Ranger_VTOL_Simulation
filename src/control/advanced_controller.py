"""Placeholder for robust or fault-tolerant controllers."""

from __future__ import annotations

from typing import Dict

import numpy as np

from .base_controller import BaseController


class PlaceholderAdvancedController(BaseController):
    """Template for future MPC, FTC, or robust controllers.

    Extend this class when the research baseline is ready. The method
    signature already matches the rest of the simulation stack.
    """

    def __init__(self, model, controller_config: Dict, log_enabled: bool = False):
        super().__init__(model, log_enabled=log_enabled)
        self.controller_config = controller_config

    def compute_controls(self, state: np.ndarray, reference: Dict, t: float = 0.0, dt: float = 0.01) -> np.ndarray:
        """Return nominal hover commands until custom logic is inserted."""
        _ = (state, reference, t, dt)
        commands = self.model.trim_hover_commands()
        self._log({"t": t, "mode": "placeholder", "commands": commands.copy()})
        return commands
