"""Dynamics utilities for the VTOL research model."""

from .vtol_model import VTOLDynamicsModel, RotorSpec, VehicleParameters, build_vehicle_parameters
from .lagrangian_quaternion import (
    LagrangianQuaternionStateSpaceModel,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_derivative,
    quaternion_to_rotation_matrix,
    rotation_vector_to_quaternion,
)
from .transition_aero import (
    AeroCoefficients,
    TransitionAeroParameters,
    aerodynamic_forces,
    fa_ratio_from_force,
    simulate_transition_aero,
    to_hil_packet,
    transition_blend_weight,
)

__all__ = [
    "VTOLDynamicsModel",
    "RotorSpec",
    "VehicleParameters",
    "build_vehicle_parameters",
    "LagrangianQuaternionStateSpaceModel",
    "normalize_quaternion",
    "quaternion_conjugate",
    "quaternion_derivative",
    "quaternion_to_rotation_matrix",
    "rotation_vector_to_quaternion",
    "AeroCoefficients",
    "TransitionAeroParameters",
    "aerodynamic_forces",
    "fa_ratio_from_force",
    "simulate_transition_aero",
    "to_hil_packet",
    "transition_blend_weight",
]
