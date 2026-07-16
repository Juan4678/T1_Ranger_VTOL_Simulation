"""Transition aerodynamics helpers for tilt-gondola VTOL studies."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, sqrt
from typing import Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class TransitionAeroParameters:
    """Approximate aerodynamic parameters for transition scheduling."""

    mass_kg: float = 1.85
    wing_area_m2: float = 0.34
    air_density_kg_m3: float = 1.225
    gravity_mps2: float = 9.80665

    @property
    def weight_n(self) -> float:
        return self.mass_kg * self.gravity_mps2


@dataclass(frozen=True)
class AeroCoefficients:
    cl0: float = 0.18
    cl_alpha_per_rad: float = 4.8
    cd0: float = 0.045
    induced_drag_factor: float = 0.075
    stall_angle_deg: float = 14.0

    def lift_coefficient(self, alpha_rad: float) -> float:
        stall = radians(self.stall_angle_deg)
        alpha = clamp(alpha_rad, -stall, stall)
        return self.cl0 + self.cl_alpha_per_rad * alpha

    def drag_coefficient(self, cl: float) -> float:
        return self.cd0 + self.induced_drag_factor * cl * cl


@dataclass(frozen=True)
class AeroResult:
    lift_n: float
    drag_n: float
    force_ned_n: tuple[float, float, float]
    fa_ratio: float
    support_ratio: float


@dataclass(frozen=True)
class TransitionSample:
    time_s: float
    airspeed_mps: float
    alpha_rad: float
    tilt_angle_rad: float
    aero: AeroResult


def aerodynamic_forces(
    params: TransitionAeroParameters,
    coeffs: AeroCoefficients,
    airspeed_mps: float,
    alpha_rad: float,
) -> AeroResult:
    """Estimate aerodynamic force and Fa/(m*g) in NED coordinates."""
    q = 0.5 * params.air_density_kg_m3 * airspeed_mps * airspeed_mps
    cl = coeffs.lift_coefficient(alpha_rad)
    cd = coeffs.drag_coefficient(cl)
    lift_n = q * params.wing_area_m2 * cl
    drag_n = q * params.wing_area_m2 * cd

    fx = -drag_n * cos(alpha_rad) + lift_n * sin(alpha_rad)
    fz = drag_n * sin(alpha_rad) - lift_n * cos(alpha_rad)
    force_ned_n = (fx, 0.0, fz)
    magnitude = sqrt(fx * fx + fz * fz)
    support = max(-fz, 0.0)
    return AeroResult(
        lift_n=lift_n,
        drag_n=drag_n,
        force_ned_n=force_ned_n,
        fa_ratio=magnitude / params.weight_n,
        support_ratio=support / params.weight_n,
    )


def transition_blend_weight(
    support_ratio: float,
    airspeed_mps: float,
    tilt_angle_rad: float,
    support_bounds: tuple[float, float] = (0.18, 0.92),
    airspeed_bounds_mps: tuple[float, float] = (7.0, 16.0),
    tilt_bounds_deg: tuple[float, float] = (15.0, 82.0),
) -> float:
    """Return fixed-wing authority weight gated by lift, speed, and tilt."""

    def smoothstep(edge0: float, edge1: float, value: float) -> float:
        if edge0 == edge1:
            return 1.0 if value >= edge1 else 0.0
        x = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)

    support_progress = smoothstep(support_bounds[0], support_bounds[1], support_ratio)
    airspeed_progress = smoothstep(airspeed_bounds_mps[0], airspeed_bounds_mps[1], airspeed_mps)
    tilt_progress = smoothstep(radians(tilt_bounds_deg[0]), radians(tilt_bounds_deg[1]), tilt_angle_rad)
    return support_progress * airspeed_progress * tilt_progress


def simulate_transition_aero(
    params: TransitionAeroParameters | None = None,
    coeffs: AeroCoefficients | None = None,
    duration_s: float = 8.0,
    dt_s: float = 0.05,
    start_airspeed_mps: float = 2.0,
    end_airspeed_mps: float = 20.0,
    alpha_deg: float = 5.0,
) -> list[TransitionSample]:
    params = params or TransitionAeroParameters()
    coeffs = coeffs or AeroCoefficients()
    steps = max(1, int(duration_s / dt_s))
    samples: list[TransitionSample] = []
    alpha = radians(alpha_deg)

    for index in range(steps + 1):
        frac = index / steps
        ramp = frac * frac * (3.0 - 2.0 * frac)
        airspeed = start_airspeed_mps + (end_airspeed_mps - start_airspeed_mps) * ramp
        tilt = radians(90.0 * ramp)
        aero = aerodynamic_forces(params, coeffs, airspeed, alpha)
        samples.append(
            TransitionSample(
                time_s=index * dt_s,
                airspeed_mps=airspeed,
                alpha_rad=alpha,
                tilt_angle_rad=tilt,
                aero=aero,
            )
        )

    return samples


def to_hil_packet(sample: TransitionSample, params: TransitionAeroParameters) -> dict[str, object]:
    """Serialize a transition sample for the FX405 reference HIL bridge."""
    return {
        "mass_kg": params.mass_kg,
        "airspeed_mps": sample.airspeed_mps,
        "tilt_angle_rad": sample.tilt_angle_rad,
        "aero_force_ned_n": list(sample.aero.force_ned_n),
    }


def fa_ratio_from_force(force_ned_n: Sequence[float], mass_kg: float, gravity_mps2: float = 9.80665) -> float:
    """Compute |Fa|/(m*g) for logs or imported HIL data."""
    weight_n = max(mass_kg * gravity_mps2, 1.0e-6)
    return sqrt(sum(float(value) * float(value) for value in force_ned_n)) / weight_n
