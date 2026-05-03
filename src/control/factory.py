"""Factory helper for controller selection."""


def build_controller(name: str, model, config: dict, log_enabled: bool = False):
    normalized = name.lower()
    if normalized == "pid":
        from .pid_velocity_controller import PIDVelocityController

        return PIDVelocityController(model, config, log_enabled=log_enabled)
    if normalized == "lqr":
        from .baseline_lqr_controller import BaselineLQRController

        return BaselineLQRController(model, config, log_enabled=log_enabled)
    if normalized in {"advanced", "placeholder", "ftc", "mpc"}:
        from .advanced_controller import PlaceholderAdvancedController

        return PlaceholderAdvancedController(model, config, log_enabled=log_enabled)
    raise ValueError(f"Unknown controller '{name}'.")
