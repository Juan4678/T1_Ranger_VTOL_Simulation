"""Factory helper for controller selection."""


def build_controller(name: str, model, config: dict, log_enabled: bool = False):
    normalized = name.lower()
    if normalized == "pid":
        from .pid_velocity_controller import PIDVelocityController

        return PIDVelocityController(model, config, log_enabled=log_enabled)
    if normalized == "lqr":
        from .baseline_lqr_controller import BaselineLQRController

        return BaselineLQRController(model, config, log_enabled=log_enabled)
    if normalized in {"ftc", "robust", "fault_tolerant"}:
        from .fault_tolerant_allocator import FaultTolerantAllocatorController

        nominal_name = config.get("controllers", {}).get("fault_tolerant", {}).get("nominal_controller", "pid")
        if nominal_name.lower() in {"ftc", "robust", "fault_tolerant"}:
            raise ValueError("Fault-tolerant controller cannot wrap itself as the nominal controller.")
        nominal = build_controller(nominal_name, model, config, log_enabled=log_enabled)
        return FaultTolerantAllocatorController(model, nominal, config, log_enabled=log_enabled)
    if normalized in {"advanced", "placeholder", "mpc"}:
        from .advanced_controller import PlaceholderAdvancedController

        return PlaceholderAdvancedController(model, config, log_enabled=log_enabled)
    raise ValueError(f"Unknown controller '{name}'.")
