"""Run a lightweight transition-aero simulation and emit HIL-ready packets."""

from __future__ import annotations

from pathlib import Path
import importlib.util
import json
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "src" / "dynamics" / "transition_aero.py"
spec = importlib.util.spec_from_file_location("transition_aero", MODULE_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Could not load {MODULE_PATH}")
transition_aero = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = transition_aero
spec.loader.exec_module(transition_aero)

TransitionAeroParameters = transition_aero.TransitionAeroParameters
simulate_transition_aero = transition_aero.simulate_transition_aero
to_hil_packet = transition_aero.to_hil_packet
transition_blend_weight = transition_aero.transition_blend_weight


def main() -> None:
    params = TransitionAeroParameters()
    samples = simulate_transition_aero(params=params, dt_s=0.25)

    print("t  airspeed  tilt  Fa/W  Lift/W  FW")
    for sample in samples[::4]:
        fixed_wing_weight = transition_blend_weight(
            sample.aero.support_ratio,
            sample.airspeed_mps,
            sample.tilt_angle_rad,
        )
        print(
            f"{sample.time_s:4.1f} {sample.airspeed_mps:8.2f} "
            f"{sample.tilt_angle_rad * 180.0 / 3.141592653589793:5.1f} "
            f"{sample.aero.fa_ratio:5.2f} {sample.aero.support_ratio:6.2f} "
            f"{fixed_wing_weight:4.2f}"
        )

    print("\nLast HIL packet:")
    print(json.dumps(to_hil_packet(samples[-1], params), indent=2))


if __name__ == "__main__":
    main()
