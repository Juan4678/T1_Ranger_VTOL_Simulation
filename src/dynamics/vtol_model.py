        """Simplified VTOL dynamics model suitable for research simulations."""

        from __future__ import annotations

        from dataclasses import dataclass
        from typing import Dict, Iterable, List, Sequence

        import numpy as np


        def _as_array(values: Sequence[float]) -> np.ndarray:
            return np.asarray(values, dtype=float)


        @dataclass
        class RotorSpec:
            """Stores geometry and limits for one rotor."""

            name: str
            position_m: np.ndarray
            axis_body: np.ndarray
            direction: int
            max_thrust_n: float
            torque_coefficient: float = 0.0

            def normalized_axis(self) -> np.ndarray:
                axis = _as_array(self.axis_body)
                norm = np.linalg.norm(axis)
                if norm == 0.0:
                    raise ValueError(f"Rotor '{self.name}' has a zero axis vector.")
                return axis / norm


        @dataclass
        class VehicleParameters:
            """Container for the main VTOL physical parameters."""

            name: str
            mass_kg: float
            gravity_mps2: float
            inertia_kgm2: np.ndarray
            linear_drag: np.ndarray
            angular_drag: np.ndarray
            rotors: List[RotorSpec]


        def build_vehicle_parameters(vehicle_config: Dict) -> VehicleParameters:
            """Build parameter dataclasses from the JSON configuration dictionary.

            Args:
                vehicle_config: Vehicle section of the JSON config.

            Returns:
                Parsed vehicle parameters.
            """
            rotors = [
                RotorSpec(
                    name=entry["name"],
                    position_m=_as_array(entry["position_m"]),
                    axis_body=_as_array(entry["axis_body"]),
                    direction=int(entry["direction"]),
                    max_thrust_n=float(entry["max_thrust_n"]),
                    torque_coefficient=float(entry.get("torque_coefficient", 0.0)),
                )
                for entry in vehicle_config["rotors"]
            ]
            return VehicleParameters(
                name=vehicle_config["name"],
                mass_kg=float(vehicle_config["mass_kg"]),
                gravity_mps2=float(vehicle_config.get("gravity_mps2", 9.81)),
                inertia_kgm2=_as_array(vehicle_config["inertia_kgm2"]),
                linear_drag=_as_array(vehicle_config.get("linear_drag", [0.0, 0.0, 0.0])),
                angular_drag=_as_array(vehicle_config.get("angular_drag", [0.0, 0.0, 0.0])),
                rotors=rotors,
            )


        def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
            """Return the ZYX rotation matrix from body to inertial frame."""
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            return np.array(
                [
                    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                    [-sp, cp * sr, cp * cr],
                ],
                dtype=float,
            )


        def body_rates_to_euler_rates(roll: float, pitch: float, omega: np.ndarray) -> np.ndarray:
            """Convert body angular rates to Euler angle rates."""
            sr, cr = np.sin(roll), np.cos(roll)
            cp = np.cos(pitch)
            cp = np.sign(cp) * max(abs(cp), 1e-5)
            tp = np.tan(pitch)
            transform = np.array(
                [
                    [1.0, sr * tp, cr * tp],
                    [0.0, cr, -sr],
                    [0.0, sr / cp, cr / cp],
                ],
                dtype=float,
            )
            return transform @ omega


        class VTOLDynamicsModel:
            """Simplified 12-state VTOL rigid-body model.

            State order:
                [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
            """

            def __init__(self, params: VehicleParameters):
                self.params = params
                self.inertia = np.diag(params.inertia_kgm2)
                self.inertia_inv = np.diag(1.0 / params.inertia_kgm2)
                self.rotor_count = len(params.rotors)
                self.max_thrusts = np.array([rotor.max_thrust_n for rotor in params.rotors], dtype=float)
                self.hover_indices = [
                    idx
                    for idx, rotor in enumerate(params.rotors)
                    if rotor.normalized_axis()[2] > 0.5
                ]

            @classmethod
            def from_config(cls, config: Dict) -> "VTOLDynamicsModel":
                return cls(build_vehicle_parameters(config["vehicle"]))

            def describe(self) -> str:
                """Return a readable description of the model parameters."""
                lines = [
                    f"Vehicle: {self.params.name}",
                    f"Mass: {self.params.mass_kg:.2f} kg",
                    f"Inertia diag: {self.params.inertia_kgm2.tolist()}",
                    f"Rotors: {self.rotor_count}",
                ]
                for rotor in self.params.rotors:
                    lines.append(
                        f"  - {rotor.name}: pos={rotor.position_m.tolist()}, axis={rotor.normalized_axis().tolist()}, max={rotor.max_thrust_n:.1f} N"
                    )
                return "
".join(lines)

            def trim_hover_commands(self) -> np.ndarray:
                """Return a nominal hover thrust vector."""
                if not self.hover_indices:
                    raise RuntimeError("No vertical-lift rotors were detected.")
                commands = np.zeros(self.rotor_count, dtype=float)
                total_vertical_share = sum(
                    self.params.rotors[idx].normalized_axis()[2] for idx in self.hover_indices
                )
                thrust_per_unit = self.params.mass_kg * self.params.gravity_mps2 / total_vertical_share
                for idx in self.hover_indices:
                    axis_z = self.params.rotors[idx].normalized_axis()[2]
                    commands[idx] = thrust_per_unit
                    if axis_z <= 0.0:
                        commands[idx] = 0.0
                return np.clip(commands, 0.0, self.max_thrusts)

            def hover_allocation_matrix(self) -> np.ndarray:
                """Return the hover allocation matrix [Fz, tau_x, tau_y, tau_z]."""
                columns = []
                for rotor in self.params.rotors:
                    axis = rotor.normalized_axis()
                    torque = np.cross(rotor.position_m, axis)
                    torque[2] += rotor.direction * rotor.torque_coefficient
                    columns.append(np.array([axis[2], torque[0], torque[1], torque[2]], dtype=float))
                return np.column_stack(columns)

            def allocate_hover_wrench(self, total_vertical_force: float, torques: Sequence[float]) -> np.ndarray:
                """Allocate a hover wrench to rotor thrusts.

                Args:
                    total_vertical_force: Desired vertical force in body-z.
                    torques: Desired body torques [tau_x, tau_y, tau_z].

                Returns:
                    Rotor thrust command vector.
                """
                wrench = np.concatenate(([float(total_vertical_force)], _as_array(torques)))
                allocation = self.hover_allocation_matrix()
                commands, *_ = np.linalg.lstsq(allocation, wrench, rcond=None)
                return np.clip(commands, 0.0, self.max_thrusts)

            def body_force_and_torque(self, commands: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
                """Compute net body-frame force and torque from rotor thrusts."""
                commands = np.clip(_as_array(commands), 0.0, self.max_thrusts)
                total_force = np.zeros(3, dtype=float)
                total_torque = np.zeros(3, dtype=float)
                for thrust, rotor in zip(commands, self.params.rotors):
                    axis = rotor.normalized_axis()
                    force_i = thrust * axis
                    total_force += force_i
                    total_torque += np.cross(rotor.position_m, force_i)
                    total_torque[2] += rotor.direction * rotor.torque_coefficient * thrust
                return total_force, total_torque

            def derivatives(self, time_s: float, state: Sequence[float], commands: Sequence[float]) -> np.ndarray:
                """Evaluate the state derivatives for the ODE solver.

                Args:
                    time_s: Simulation time, unused but kept for solve_ivp compatibility.
                    state: Current 12-state vector.
                    commands: Rotor thrust vector in Newtons.

                Returns:
                    State derivative vector.
                """
                _ = time_s
                state = _as_array(state)
                pos = state[0:3]
                vel = state[3:6]
                euler = state[6:9]
                omega = state[9:12]

                force_body, torque_body = self.body_force_and_torque(commands)
                rotation = euler_to_rotation_matrix(*euler)
                gravity = np.array([0.0, 0.0, -self.params.gravity_mps2], dtype=float)
                linear_drag = -self.params.linear_drag * vel
                accel = gravity + (rotation @ force_body + linear_drag) / self.params.mass_kg

                euler_dot = body_rates_to_euler_rates(euler[0], euler[1], omega)
                coriolis = np.cross(omega, self.inertia @ omega)
                angular_drag = self.params.angular_drag * omega
                omega_dot = self.inertia_inv @ (torque_body - coriolis - angular_drag)

                derivatives = np.zeros_like(state)
                derivatives[0:3] = vel
                derivatives[3:6] = accel
                derivatives[6:9] = euler_dot
                derivatives[9:12] = omega_dot
                return derivatives
