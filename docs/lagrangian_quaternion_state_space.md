# Lagrangian Quaternion State-Space Model

This document is the research-grade dynamics deliverable for the T1 Ranger VTOL
scaffold.

The original Newton-Euler model stays in the study path because it is the
clearest first explanation of rigid-body force and moment balance. This model is
the next layer: a Lagrange-d'Alembert derivation written directly as a
quaternion state-space system.

## Why This Layer Exists

The Newton-Euler model is good for learning:

- force balance is visible
- torque from `r x F` is easy to inspect
- PID/LQR tests are simple to run
- beginner explanations stay compact

The Lagrangian/quaternion model is better for the research story:

- attitude avoids Euler-angle singularities
- energy terms are explicit
- moving tilt mechanisms can later become generalized coordinates
- the state-space form is ready for linearization, LQR, EKF, MPC, and fault
  detection work

## Frames And Conventions

- `I`: inertial/world frame
- `B`: aircraft body frame
- `p_I = [x, y, z]^T`: inertial position
- `v_I = dot(p_I)`: inertial velocity
- `q = [q_w, q_x, q_y, q_z]^T`: scalar-first unit quaternion mapping body
  vectors into inertial coordinates
- `R(q)`: body-to-inertial rotation matrix
- `omega_B = [p, q, r]^T`: body angular velocity
- `J`: body-frame inertia matrix

The quaternion must satisfy:

```math
q^T q = 1
```

The continuous state is:

```math
x =
\begin{bmatrix}
p_I \\
v_I \\
q \\
\omega_B
\end{bmatrix}
\in R^{13}
```

The input is the rotor thrust vector:

```math
u =
\begin{bmatrix}
T_1 & T_2 & \dots & T_n
\end{bmatrix}^T
```

## Rotor Wrench Model

Each rotor has:

- `r_i`: rotor position in body coordinates
- `a_i`: unit thrust axis in body coordinates
- `T_i`: thrust command in Newtons
- `d_i`: spin direction sign
- `c_tau,i`: yaw torque coefficient

The body force from all rotors is:

```math
F_B(u) = \sum_i T_i a_i
```

The body torque is:

```math
\tau_B(u) =
\sum_i r_i \times (T_i a_i)
+ \sum_i d_i c_{\tau,i} T_i e_z
```

Drag is modeled as:

```math
F_D = -D_v v_I
```

```math
\tau_D = -D_\omega \omega_B
```

## Lagrangian

The kinetic energy is:

```math
T =
\frac{1}{2} m v_I^T v_I
+ \frac{1}{2} \omega_B^T J \omega_B
```

The potential energy is:

```math
V = m g z
```

So the Lagrangian is:

```math
L = T - V
```

External generalized forces are supplied through Lagrange-d'Alembert:

```math
Q_p = R(q)F_B(u) + F_D
```

```math
Q_\omega = \tau_B(u) + \tau_D
```

Applying the translational Euler-Lagrange equations gives:

```math
m \dot{v}_I = R(q)F_B(u) + F_D + m g_I
```

where:

```math
g_I = [0, 0, -g]^T
```

Applying the rotational form on `SO(3)` gives:

```math
J \dot{\omega}_B + \omega_B \times J\omega_B =
\tau_B(u) + \tau_D
```

Therefore:

```math
\dot{\omega}_B =
J^{-1}
\left(
\tau_B(u)
- \omega_B \times J\omega_B
- D_\omega \omega_B
\right)
```

## Quaternion Kinematics

With scalar-first quaternion convention and body-frame angular velocity:

```math
\dot{q} =
\frac{1}{2} q \otimes
\begin{bmatrix}
0 \\
\omega_B
\end{bmatrix}
```

Expanded:

```math
\dot{q} =
\frac{1}{2}
\begin{bmatrix}
-q_x p - q_y q - q_z r \\
 q_w p + q_y r - q_z q \\
 q_w q - q_x r + q_z p \\
 q_w r + q_x q - q_y p
\end{bmatrix}
```

After numerical integration, the quaternion should be projected back to unit
norm:

```math
q <- \frac{q}{||q||}
```

## Nonlinear State-Space Form

The model can be written as:

```math
\dot{x} = f(x, u)
```

with:

```math
\dot{p}_I = v_I
```

```math
\dot{v}_I =
g_I + \frac{1}{m}
\left(
R(q)F_B(u) - D_v v_I
\right)
```

```math
\dot{q} =
\frac{1}{2} q \otimes
\begin{bmatrix}
0 \\
\omega_B
\end{bmatrix}
```

```math
\dot{\omega}_B =
J^{-1}
\left(
\tau_B(u)
- \omega_B \times J\omega_B
- D_\omega \omega_B
\right)
```

The Python evaluator lives in:

```text
src/dynamics/lagrangian_quaternion.py
```

## Hover Operating Point

A hover trim point is:

```math
x_0 =
\begin{bmatrix}
p_{I,0} \\
0 \\
[1, 0, 0, 0]^T \\
0
\end{bmatrix}
```

The trim input `u_0` is chosen so the vertical rotor force balances weight:

```math
R(q_0)F_B(u_0) + m g_I = 0
```

The helper method is:

```python
commands = model.trim_hover_commands()
```

## Linear State-Space Model

Around an operating point `(x_0, u_0)`, define:

```math
\delta x_e =
\begin{bmatrix}
\delta p_I \\
\delta v_I \\
\delta \theta_B \\
\delta \omega_B
\end{bmatrix}
\in R^{12}
```

```math
\delta u = u - u_0
```

The quaternion itself still lives in the nonlinear state, but the linear model
uses the local 3D attitude perturbation `delta theta_B` rather than four
independent quaternion components. This respects the unit-quaternion constraint:

```math
q = q_0 \otimes \delta q
```

```math
\delta q =
\begin{bmatrix}
\cos(||\delta\theta_B||/2) \\
\sin(||\delta\theta_B||/2)
\frac{\delta\theta_B}{||\delta\theta_B||}
\end{bmatrix}
```

For small attitude error:

```math
\delta q \approx
\begin{bmatrix}
1 \\
\frac{1}{2}\delta\theta_B
\end{bmatrix}
```

The local linear model is:

```math
\delta \dot{x}_e = A_e \delta x_e + B_e \delta u
```

where:

```math
A_e =
\left.
\frac{\partial f_e}{\partial \delta x_e}
\right|_{\delta x_e=0,u_0}
```

```math
B_e =
\left.
\frac{\partial f_e}{\partial u}
\right|_{\delta x_e=0,u_0}
```

The implementation computes these controller-facing Jacobians numerically:

```python
A, B = model.linearize(x0, u0)
```

This returns a `12 x 12` local-error `A` matrix and a `12 x n_rotors` `B`
matrix. The full 13-state quaternion is only used internally to evaluate the
nonlinear dynamics.

## Usage

```python
import numpy as np

from src.configuration import load_json_config
from src.dynamics import LagrangianQuaternionStateSpaceModel

config = load_json_config("config/default_params.json")
model = LagrangianQuaternionStateSpaceModel.from_config(config)

x0 = np.zeros(13)
x0[2] = -2.0
x0[6] = 1.0

u0 = model.trim_hover_commands()
xdot = model.derivatives(0.0, x0, u0)
A, B = model.linearize(x0, u0)
```

## Next Research Steps

1. Replace approximate mass and inertia with measured or Mission Planner-derived
   values.
2. Add tilt-gondola angles as generalized coordinates.
3. Add motor, ESC, and servo dynamics.
4. Add transition aerodynamics into `F_B`, `tau_B`, or separate aerodynamic
   wrench terms.
5. Use the 12-state quaternion error linearization for LQR and EKF design.
6. Validate hover and transition predictions against logs before claiming model
   fidelity.
