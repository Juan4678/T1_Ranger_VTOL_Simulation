"""Quaternion state-space model from a Lagrange-d'Alembert derivation.

The implementation evaluates the state-space equations documented in
``docs/lagrangian_quaternion_state_space.md``. It keeps the same force and
torque model as the didactic Newton-Euler scaffold, but exposes attitude with a
unit quaternion instead of Euler angles.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np

from .vtol_model import VTOLDynamicsModel, VehicleParameters, build_vehicle_parameters


def normalize_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    """Return a scalar-first unit quaternion [qw, qx, qy, qz]."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,):
        raise ValueError("Quaternion must have shape (4,).")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Cannot normalize a near-zero quaternion.")
    return q / norm


def quaternion_conjugate(quaternion: Sequence[float]) -> np.ndarray:
    """Return the conjugate of a scalar-first unit quaternion."""
    qw, qx, qy, qz = normalize_quaternion(quaternion)
    return np.array([qw, -qx, -qy, -qz], dtype=float)


def rotation_vector_to_quaternion(rotation_vector: Sequence[float]) -> np.ndarray:
    """Convert a local 3D rotation vector to a scalar-first quaternion."""
    delta = np.asarray(rotation_vector, dtype=float)
    if delta.shape != (3,):
        raise ValueError("Rotation vector must have shape (3,).")

    angle = np.linalg.norm(delta)
    if angle < 1e-12:
        return normalize_quaternion(np.concatenate(([1.0], 0.5 * delta)))

    axis = delta / angle
    half_angle = 0.5 * angle
    return np.concatenate(([np.cos(half_angle)], np.sin(half_angle) * axis))


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    """Hamilton product for scalar-first quaternions."""
    left_array = np.asarray(left, dtype=float)
    right_array = np.asarray(right, dtype=float)
    if left_array.shape != (4,) or right_array.shape != (4,):
        raise ValueError("Quaternion operands must both have shape (4,).")
    lw, lx, ly, lz = left_array
    rw, rx, ry, rz = right_array
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def quaternion_to_rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Return the body-to-inertial rotation matrix for [qw, qx, qy, qz]."""
    qw, qx, qy, qz = normalize_quaternion(quaternion)
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def quaternion_derivative(quaternion: Sequence[float], omega_body: Sequence[float]) -> np.ndarray:
    """Return q_dot = 0.5 * q (*) [0, omega_body]."""
    q = normalize_quaternion(quaternion)
    omega_quaternion = np.concatenate(([0.0], np.asarray(omega_body, dtype=float)))
    return 0.5 * quaternion_multiply(q, omega_quaternion)


class LagrangianQuaternionStateSpaceModel:
    """Continuous-time VTOL state-space model with quaternion attitude.

    State order:
        [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]

    Input order:
        rotor thrusts in Newtons, matching the configured rotor order.
    """

    state_size = 13
    error_state_size = 12

    def __init__(self, params: VehicleParameters):
        self.params = params
        self._rigid_body = VTOLDynamicsModel(params)
        self.inertia = self._rigid_body.inertia
        self.inertia_inv = self._rigid_body.inertia_inv
        self.rotor_count = self._rigid_body.rotor_count
        self.max_thrusts = self._rigid_body.max_thrusts

    @classmethod
    def from_config(cls, config: Dict) -> "LagrangianQuaternionStateSpaceModel":
        """Build a model from the repository JSON configuration."""
        return cls(build_vehicle_parameters(config["vehicle"]))

    def trim_hover_commands(self) -> np.ndarray:
        """Return the same nominal hover trim used by the study model."""
        return self._rigid_body.trim_hover_commands()

    def body_force_and_torque(self, commands: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Compute net body-frame force and torque from rotor thrusts."""
        return self._rigid_body.body_force_and_torque(commands)

    def normalize_state(self, state: Sequence[float]) -> np.ndarray:
        """Return a copy of the state with the quaternion projected to unit norm."""
        state_array = np.asarray(state, dtype=float).copy()
        if state_array.shape != (self.state_size,):
            raise ValueError(f"State must have shape ({self.state_size},).")
        state_array[6:10] = normalize_quaternion(state_array[6:10])
        return state_array

    def apply_error_state(
        self,
        equilibrium_state: Sequence[float],
        error_state: Sequence[float],
    ) -> np.ndarray:
        """Map a 12-state local error vector into the full quaternion state.

        Error order:
            [dx, dy, dz, dvx, dvy, dvz, dtheta_x, dtheta_y, dtheta_z, dp, dq, dr]

        The small attitude perturbation is composed on the right side of the
        nominal quaternion, so `dtheta` is a local body-frame attitude error.
        """
        x0 = self.normalize_state(equilibrium_state)
        error = np.asarray(error_state, dtype=float)
        if error.shape != (self.error_state_size,):
            raise ValueError(f"Error state must have shape ({self.error_state_size},).")

        state = x0.copy()
        state[0:3] += error[0:3]
        state[3:6] += error[3:6]
        delta_q = rotation_vector_to_quaternion(error[6:9])
        state[6:10] = normalize_quaternion(quaternion_multiply(x0[6:10], delta_q))
        state[10:13] += error[9:12]
        return state

    def derivative_to_error_derivative(
        self,
        equilibrium_state: Sequence[float],
        state_derivative: Sequence[float],
    ) -> np.ndarray:
        """Project a 13-state derivative into the 12-state local error basis."""
        x0 = self.normalize_state(equilibrium_state)
        derivative = np.asarray(state_derivative, dtype=float)
        if derivative.shape != (self.state_size,):
            raise ValueError(f"State derivative must have shape ({self.state_size},).")

        local_q_dot = quaternion_multiply(quaternion_conjugate(x0[6:10]), derivative[6:10])

        error_derivative = np.zeros(self.error_state_size, dtype=float)
        error_derivative[0:3] = derivative[0:3]
        error_derivative[3:6] = derivative[3:6]
        error_derivative[6:9] = 2.0 * local_q_dot[1:4]
        error_derivative[9:12] = derivative[10:13]
        return error_derivative

    def error_state_derivatives(
        self,
        time_s: float,
        equilibrium_state: Sequence[float],
        error_state: Sequence[float],
        commands: Sequence[float],
    ) -> np.ndarray:
        """Evaluate local 12-state error derivatives at an offset from `x0`."""
        state = self.apply_error_state(equilibrium_state, error_state)
        state_dot = self.derivatives(time_s, state, commands)
        return self.derivative_to_error_derivative(equilibrium_state, state_dot)

    def derivatives(self, time_s: float, state: Sequence[float], commands: Sequence[float]) -> np.ndarray:
        """Evaluate x_dot = f(x, u) for ODE solvers or linearization."""
        _ = time_s
        state_array = self.normalize_state(state)
        commands_array = np.clip(np.asarray(commands, dtype=float), 0.0, self.max_thrusts)
        if commands_array.shape != (self.rotor_count,):
            raise ValueError(f"Commands must have shape ({self.rotor_count},).")

        velocity = state_array[3:6]
        quaternion = state_array[6:10]
        omega = state_array[10:13]

        force_body, torque_body = self.body_force_and_torque(commands_array)
        rotation = quaternion_to_rotation_matrix(quaternion)
        gravity = np.array([0.0, 0.0, -self.params.gravity_mps2], dtype=float)
        linear_drag = -self.params.linear_drag * velocity
        acceleration = gravity + (rotation @ force_body + linear_drag) / self.params.mass_kg

        q_dot = quaternion_derivative(quaternion, omega)
        coriolis = np.cross(omega, self.inertia @ omega)
        angular_drag = self.params.angular_drag * omega
        omega_dot = self.inertia_inv @ (torque_body - coriolis - angular_drag)

        derivative = np.zeros(self.state_size, dtype=float)
        derivative[0:3] = velocity
        derivative[3:6] = acceleration
        derivative[6:10] = q_dot
        derivative[10:13] = omega_dot
        return derivative

    def linearize(
        self,
        equilibrium_state: Sequence[float],
        equilibrium_commands: Sequence[float],
        epsilon: float = 1e-5,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Numerically linearize in the 12-state local quaternion error basis."""
        return self.linearize_error_state(equilibrium_state, equilibrium_commands, epsilon)

    def linearize_error_state(
        self,
        equilibrium_state: Sequence[float],
        equilibrium_commands: Sequence[float],
        epsilon: float = 1e-5,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return A and B for local error dynamics around an operating point.

        The local error state is:
            [delta p_I, delta v_I, delta theta_B, delta omega_B]

        This is the controller-facing linearization. It avoids treating the four
        quaternion components as independent states.
        """
        x0 = self.normalize_state(equilibrium_state)
        u0 = np.asarray(equilibrium_commands, dtype=float)
        if u0.shape != (self.rotor_count,):
            raise ValueError(f"Equilibrium commands must have shape ({self.rotor_count},).")

        state_count = self.error_state_size
        input_count = self.rotor_count
        a_matrix = np.zeros((state_count, state_count), dtype=float)
        b_matrix = np.zeros((state_count, input_count), dtype=float)
        zero_error = np.zeros(state_count, dtype=float)

        for column in range(state_count):
            perturb = np.zeros(state_count, dtype=float)
            perturb[column] = epsilon
            f_plus = self.error_state_derivatives(0.0, x0, perturb, u0)
            f_minus = self.error_state_derivatives(0.0, x0, -perturb, u0)
            a_matrix[:, column] = (f_plus - f_minus) / (2.0 * epsilon)

        for column in range(input_count):
            perturb = np.zeros(input_count, dtype=float)
            perturb[column] = epsilon
            f_plus = self.error_state_derivatives(0.0, x0, zero_error, u0 + perturb)
            f_minus = self.error_state_derivatives(0.0, x0, zero_error, u0 - perturb)
            b_matrix[:, column] = (f_plus - f_minus) / (2.0 * epsilon)

        return a_matrix, b_matrix
