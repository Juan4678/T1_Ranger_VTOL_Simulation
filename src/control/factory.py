"""Factory helper for controller selection."""

from .advanced_controller import PlaceholderAdvancedController
from .baseline_lqr_controller import BaselineLQRController
from .pid_velocity_controller import PIDVelocityController


def build_controller(name: str, model, config: dict, log_enabled: bool = False):
    normalized = name.lower()
    if normalized == "pid":
        return PIDVelocityController(model, config, log_enabled=log_enabled)
    if normalized == "lqr":
        return BaselineLQRController(model, config, log_enabled=log_enabled)
    if normalized in {"advanced", "placeholder", "ftc", "mpc"}:
        return PlaceholderAdvancedController(model, config, log_enabled=log_enabled)
    raise ValueError(f"Unknown controller '{name}'.")
