"""Tests for transition-aerodynamic force scheduling."""

from pathlib import Path
import importlib.util
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

MODULE_PATH = REPO_ROOT / "src" / "dynamics" / "transition_aero.py"
spec = importlib.util.spec_from_file_location("transition_aero", MODULE_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Could not load {MODULE_PATH}")
transition_aero = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = transition_aero
spec.loader.exec_module(transition_aero)

AeroCoefficients = transition_aero.AeroCoefficients
TransitionAeroParameters = transition_aero.TransitionAeroParameters
aerodynamic_forces = transition_aero.aerodynamic_forces
simulate_transition_aero = transition_aero.simulate_transition_aero
to_hil_packet = transition_aero.to_hil_packet
transition_blend_weight = transition_aero.transition_blend_weight


class TransitionAeroTests(unittest.TestCase):
    def test_fa_ratio_increases_with_airspeed(self):
        params = TransitionAeroParameters()
        coeffs = AeroCoefficients()
        slow = aerodynamic_forces(params, coeffs, airspeed_mps=6.0, alpha_rad=0.08)
        fast = aerodynamic_forces(params, coeffs, airspeed_mps=18.0, alpha_rad=0.08)
        self.assertGreater(fast.fa_ratio, slow.fa_ratio)
        self.assertGreater(fast.support_ratio, slow.support_ratio)

    def test_transition_blend_requires_lift_speed_and_tilt(self):
        no_lift_weight = transition_blend_weight(0.05, airspeed_mps=18.0, tilt_angle_rad=1.55)
        hover_tilt_weight = transition_blend_weight(1.0, airspeed_mps=18.0, tilt_angle_rad=0.0)
        ready_weight = transition_blend_weight(1.0, airspeed_mps=18.0, tilt_angle_rad=1.55)
        self.assertEqual(no_lift_weight, 0.0)
        self.assertEqual(hover_tilt_weight, 0.0)
        self.assertGreater(ready_weight, 0.95)

    def test_simulation_hil_packet_shape(self):
        params = TransitionAeroParameters()
        sample = simulate_transition_aero(params=params, duration_s=1.0, dt_s=1.0)[-1]
        packet = to_hil_packet(sample, params)
        self.assertEqual(
            set(packet),
            {"mass_kg", "airspeed_mps", "tilt_angle_rad", "aero_force_ned_n"},
        )
        self.assertEqual(len(packet["aero_force_ned_n"]), 3)


if __name__ == "__main__":
    unittest.main()
