# Study Guide: T1 Ranger VTOL Research Framework

This guide is written for a mechatronics background with beginner-level VTOL
software experience. The goal is to help you move from "what is this code?" to
"I can explain, run, modify, and defend this project in a master's interview."

## Best Way To Read This

Use **Obsidian** for the best reading experience, especially for matrix-heavy
LaTeX:

- Open the folder `t1-ranger-vtol-research-python` as an Obsidian vault.
- Open this file.
- Switch to Reading View with `Ctrl+E`.

Keep **VS Code** open next to it for code navigation and terminal commands:

- Open the folder `t1-ranger-vtol-research-python`.
- Open this file.
- Press `Ctrl+Shift+V` for Markdown Preview.
- Use the terminal inside VS Code for the commands.

VS Code's built-in preview may show raw LaTeX for some block matrices. If you
want to stay inside VS Code, install a Markdown math extension; otherwise use
Obsidian as the main study viewer.

## What This Repository Is

This is the **research laboratory** for the T1 Ranger VTOL project.

It is not flight firmware. It is where you test ideas before deciding whether
they deserve to be ported into the C++/ArduPilot-side repository.

The main research loop is:

```text
vehicle model -> controller -> fault injection -> simulation -> metrics -> plots/logs
```

A professional research story looks like this:

1. Define a simplified but explainable VTOL model.
2. Implement baseline controllers, such as PID and LQR.
3. Add transition aerodynamics and motor-fault cases.
4. Compare controllers with quantitative metrics.
5. Use Mission Planner logs later to tune and validate the model.
6. Port only proven logic into the C++ firmware template.

## Repository Map

```text
.
├── config/
│   └── default_params.json
├── docs/
│   └── masters_research_upgrade_plan.md
├── notebooks/
├── src/
│   ├── control/
│   ├── dynamics/
│   ├── evaluation/
│   ├── faults/
│   ├── optim/
│   ├── plotting/
│   └── simulations/
├── tests/
├── README.md
└── requirements.txt
```

Read the folders in this order:

1. `config/default_params.json`
2. `src/dynamics/`
3. `src/control/`
4. `src/faults/`
5. `src/simulations/`
6. `src/evaluation/`
7. `tests/`
8. `docs/masters_research_upgrade_plan.md`

## Mental Model Of The Vehicle

The model is a simplified **T1 Ranger hover-mode tricopter surrogate**:

- Three hover rotors represent the T1 Ranger PNP tri-motor layout.
- The two front motors are treated as tilt-capable hardware, but in this first
  baseline their axes stay vertical for hover analysis.
- The rear motor is treated as a vertical lift motor for hover.
- The selected battery/test configuration uses the user-measured gross mass:
  `m = 1.4 kg`.
- This is heavier than the public `0.6-0.75 kg` takeoff-weight range, so treat
  it as a heavy-configuration simulation, not a manufacturer-envelope claim.
- The state has 12 elements:

```text
[x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
```

The convention is:

- `x, y, z`: position
- `vx, vy, vz`: velocity
- `roll, pitch, yaw`: Euler attitude
- `p, q, r`: body angular rates

The core rigid-body equations are the idea you should understand first:

$$
m \dot{\mathbf{v}}_I =
R_{IB}(\phi,\theta,\psi)\mathbf{F}_B
+ \mathbf{F}_{D,I}
+ m\mathbf{g}_I
$$

$$
J\dot{\boldsymbol{\omega}}_B =
\boldsymbol{\tau}_B
- \boldsymbol{\omega}_B \times J\boldsymbol{\omega}_B
- D_\omega\boldsymbol{\omega}_B
$$

Variables:

- `m`: vehicle mass. Current measured gross configuration: `1.4 kg`.
- `\mathbf{v}_I = [v_x, v_y, v_z]^T`: velocity in the inertial frame.
- `R_{IB}`: rotation matrix that maps body-frame forces into the inertial frame.
- `\mathbf{F}_B`: total rotor force expressed in the body frame.
- `\mathbf{F}_{D,I}`: simplified drag force in the inertial frame.
- `\mathbf{g}_I = [0, 0, -g]^T`: gravity vector.
- `J = diag(J_x, J_y, J_z)`: diagonal inertia approximation.
- `\boldsymbol{\omega}_B = [p, q, r]^T`: body angular-rate vector.
- `\boldsymbol{\tau}_B`: total body torque from rotor moment arms and yaw
  reaction torque.
- `D_\omega`: angular damping placeholder.

For hover, the basic weight balance is:

$$
\sum_{i=1}^{3} T_i \approx mg
$$

With `m = 1.4 kg`:

$$
mg = 1.4(9.81) = 13.73 \text{ N}
$$

So equal hover sharing would require:

$$
T_i \approx \frac{13.73}{3} = 4.58 \text{ N per motor}
$$

That number is a useful sanity check. The current config uses a placeholder
`7.0 N` max thrust per motor so the simulated heavy configuration can hover
with control margin. Replace that with thrust-stand data for the FX-1806 +
5126 propeller on the selected 4S battery.

This Newton-Euler form stays in the guide on purpose. It is the fastest way to
learn how forces, torques, gravity, drag, and rotor allocation interact.

The research deliverable then moves to a Lagrange-d'Alembert state-space model
with quaternion attitude:

- `docs/lagrangian_quaternion_state_space.md` 
- `src/dynamics/lagrangian_quaternion.py`

Read that after you are comfortable explaining the equations above. The physics
is connected, but the quaternion version is the better foundation for aggressive
attitude motion, state estimation, and thesis-level linearization.

Scope guardrail: the current research model is a rigid-body hover surrogate
with fixed rotor geometry. It does not yet implement the front tilt angle
`alpha` as a dynamic state, a full symbolic multibody model with moving
tilt-gondola coordinates, time-varying inertia matrix `M(q)`, or servo linkage
constraints. Those are valid future extensions, but the attainable first result
is the nonlinear quaternion state-space model, local linearization, simulation
runs, and controller/fault metrics.

## Current Hardware Baseline

Known data now reflected in `config/default_params.json`:

| Item | Value |
| --- | --- |
| Aircraft | Hee Wing T1 Ranger VTOL PNP with flight controller |
| Wingspan | `730 mm` |
| Length | `645 mm` |
| Height | `140 mm` |
| Fuselage size | `245 mm x 51 mm x 48 mm` |
| Motors | `3x FX-1806 2000KV brushless` |
| ESCs | `3x FX-25A brushless` |
| Propellers | `3x 5126` |
| Servos | `3x FX-7g digital metal gear` |
| Tilt servos | `2x metal gear tilt servos` |
| Flight controller | `FX-405`, F405-based VTOL controller |
| GPS | GPS/compass module |
| Battery selected | Gens Ace `4S 14.8 V 5000 mAh 45C`, T-style connector |
| Measured gross mass | `1.4 kg` |

Battery sanity checks:

$$
E_{nominal} = VQ = 14.8(5.0) = 74 \text{ Wh}
$$

$$
I_{max,theoretical} = C Q = 45(5.0) = 225 \text{ A}
$$

Do not confuse theoretical C-rating current with safe continuous aircraft
current. For modeling, battery logs under load are more trustworthy than the
label.

In code, these live in:

- `src/dynamics/vtol_model.py`
- `VTOLDynamicsModel.derivatives(...)`

What to study there:

- How rotor thrust becomes body force.
- How rotor position creates torque with a cross product.
- How gravity and drag enter the translational dynamics.
- How the allocation matrix maps motor thrusts to wrench.

## Rotor Allocation

Each rotor contributes force along its thrust axis:

$$
\mathbf{F}_i = T_i\mathbf{a}_i
$$

Where:

- `i` is the rotor index.
- `T_i` is rotor thrust in Newtons.
- `\mathbf{a}_i = [a_x, a_y, a_z]^T` is the rotor thrust-axis unit vector in
  body coordinates.
- In the current hover baseline, all three rotors use
  `\mathbf{a}_i = [0, 0, 1]^T`.

Each rotor also creates torque:

$$
\boldsymbol{\tau}_i =
\mathbf{r}_i \times \mathbf{F}_i
+ d_i c_{\tau,i}T_i\mathbf{e}_z
$$

Where:

- `\mathbf{r}_i = [x_i, y_i, z_i]^T` is the motor position relative to CG.
- `d_i` is motor spin direction sign.
- `c_{\tau,i}` is the yaw reaction-torque coefficient.
- `\mathbf{e}_z = [0, 0, 1]^T` is the body-z unit vector.

The code builds a hover allocation matrix:

$$
\begin{bmatrix}
F_z \\
\tau_x \\
\tau_y \\
\tau_z
\end{bmatrix}
=
A u
$$

For the current T1 hover baseline:

```text
u = [T_front_left, T_front_right, T_rear]^T
```

This is important because the fault-tolerant controller uses the same concept:
when a motor is degraded or disabled, it tries to reallocate the requested
wrench through the remaining effective motors.

## Controllers

The controllers live in `src/control/`.

### PID

File:

- `src/control/pid_velocity_controller.py`

The PID controller is a nested baseline:

```text
position error -> velocity command -> desired acceleration -> desired attitude -> torques -> rotor thrusts
```

This is the most intuitive controller and the best one to study first.

### LQR

File:

- `src/control/baseline_lqr_controller.py`

The LQR controller is a linear-control baseline. The core idea is:

$$
u = -Kx
$$

Where `K` is chosen to balance state error penalty `Q` and control effort
penalty `R`.

Study this after PID, not before.

### FTC: Fault-Tolerant Control Allocator

File:

- `src/control/fault_tolerant_allocator.py`

This is a research benchmark, not final flight code.

It wraps a nominal controller, currently configured around PID, and asks:

"If I know motor health, can I reallocate thrust to reduce damage from a motor
fault?"

The main logic is:

1. Ask the nominal controller for motor commands.
2. Convert those commands into a desired wrench.
3. Estimate which motors are healthy.
4. Solve a least-squares allocation problem for the healthy motors.

The high-level problem is:

$$
\min_u \| A_{\eta} u - w_{desired} \|^2 + \lambda \|u\|^2
$$

Where:

- `A_eta` is the allocation matrix adjusted by motor efficiencies.
- `u` is the new motor command vector.
- `w_desired` is the wrench requested by the nominal controller.
- `lambda` is a small regularization term.

Important research caveat:

The current FTC controller receives motor efficiency from the simulation fault
profile. That is an "oracle" estimate. It is useful for an upper-bound study,
but a real system would need a motor-health estimator.

## Fault Injection

Faults live in:

- `src/faults/fault_injector.py`
- `config/default_params.json`

A motor fault is represented as an efficiency:

```text
1.0 = healthy
0.5 = half thrust
0.0 = motor disabled
```

The current profiles include:

- `single_rotor_hover`
- `front_tilt_pair_degraded`
- `rear_lift_off`
- `front_right_tilt_off`

The rear-lift and front-tilt motor faults are intentionally severe in a
tricopter hover model because losing one motor removes both lift and torque
authority. Treat them as degradation studies first, not guaranteed recovery
cases.

## Transition Aerodynamics

File:

- `src/dynamics/transition_aero.py`

This module estimates aerodynamic force during the transition phase.

The key ratio is:

$$
\frac{F_a}{mg}
$$

Where:

- `F_a` is aerodynamic force magnitude.
- `m` is vehicle mass.
- `g` is gravity.

Interpretation:

- `Fa/(mg) = 0`: wings are carrying no meaningful weight.
- `Fa/(mg) = 0.5`: aerodynamics carry about half the weight.
- `Fa/(mg) = 1`: aerodynamics can theoretically carry vehicle weight.

The transition blend logic also uses support ratio:

$$
\frac{F_{lift,up}}{mg}
$$

This is safer than using total aerodynamic force alone because drag can be large
without helping the vehicle stay airborne.

## Simulation Scripts

Run these from the repo root.

### 1. Confirm The Environment

```powershell
C:\Python314\python.exe --version
C:\Python314\python.exe -m pytest tests -v
```

Expected current result:

```text
10 passed
```

### 2. Hover Baselines

```powershell
C:\Python314\python.exe src\simulations\run_hover.py --controller pid
C:\Python314\python.exe src\simulations\run_hover.py --controller lqr
```

Study question:

Which controller holds altitude and attitude with less oscillation?

### 3. Trajectory Tracking

```powershell
C:\Python314\python.exe src\simulations\run_trajectory_tracking.py --controller lqr --trajectory circle
```

Study question:

Does the model track the path because the controller is good, or because the
trajectory is easy?

### 4. Motor Fault Scenario

```powershell
C:\Python314\python.exe src\simulations\run_fault_scenarios.py --controller pid --fault single_rotor_hover
C:\Python314\python.exe src\simulations\run_fault_scenarios.py --controller ftc --fault single_rotor_hover
```

Study question:

Does FTC recover, or only reduce failure severity?

That distinction matters in a serious research presentation.

### 5. Motor Fault Campaign

```powershell
C:\Python314\python.exe src\simulations\run_motor_fault_campaign.py --duration 10 --controllers pid ftc --faults single_rotor_hover rear_lift_off front_right_tilt_off
```

This prints a table with:

- RMS position error
- maximum position error
- final position error
- maximum tilt
- maximum yaw error
- failure flag

Use this as your first quantitative comparison table.

### 6. Transition/HIL Packet Check

```powershell
C:\Python314\python.exe src\simulations\run_transition_hil.py
```

Study question:

At what airspeed and tilt angle does fixed-wing authority become dominant?

## How Mission Planner Fits

Mission Planner helps connect the simulation world to the real VTOL.

Use it for:

- Exporting real ArduPilot parameters.
- Downloading logs.
- Checking motor outputs, attitude, airspeed, altitude, and transitions.
- Planning waypoint missions.
- Later comparing real logs against Python simulation results.

The ideal validation loop is:

```text
Python simulation -> SITL/HIL -> Mission Planner logs -> parameter/model update -> repeat
```

## What To Learn In Order

### Stage 1: Code Orientation

Goal: Know where everything is.

Read:

- `README.md`
- `config/default_params.json`
- `src/dynamics/vtol_model.py`
- `src/simulations/core.py`

Run:

```powershell
C:\Python314\python.exe -m pytest tests -v
```

### Stage 2: Dynamics

Goal: Explain how motor thrust moves the vehicle.

Read:

- `VTOLDynamicsModel.body_force_and_torque`
- `VTOLDynamicsModel.hover_allocation_matrix`
- `VTOLDynamicsModel.derivatives`

Write in your notes:

- What is the difference between force and torque?
- Why does rotor position matter?
- Which rotor creates positive roll torque?

### Stage 3: Control

Goal: Explain PID and LQR baselines.

Read:

- `src/control/pid_velocity_controller.py`
- `src/control/baseline_lqr_controller.py`

Run:

```powershell
C:\Python314\python.exe src\simulations\run_hover.py --controller pid
C:\Python314\python.exe src\simulations\run_hover.py --controller lqr
```

### Stage 4: Fault Tolerance

Goal: Explain motor-fault degradation and control allocation.

Read:

- `src/faults/fault_injector.py`
- `src/control/fault_tolerant_allocator.py`
- `src/evaluation/fault_metrics.py`

Run:

```powershell
C:\Python314\python.exe src\simulations\run_motor_fault_campaign.py --duration 10 --controllers pid ftc --faults single_rotor_hover rear_lift_off front_right_tilt_off
```

### Stage 5: Transition

Goal: Explain why VTOL transition is not just "tilt motors forward."

Read:

- `src/dynamics/transition_aero.py`
- `src/simulations/run_transition_hil.py`

Understand:

$$
fixed\_wing\_weight = f(airspeed) f(tilt) f(F_{lift}/mg)
$$

This prevents the controller from switching to fixed-wing authority before the
wing can actually support the vehicle.

### Stage 6: Research Presentation

Goal: Make the project look like a master's-level research plan.

Read:

- `docs/masters_research_upgrade_plan.md`

Prepare a short explanation:

1. What problem are you solving?
2. What model did you build?
3. What controllers did you compare?
4. What faults did you inject?
5. What metrics prove improvement?
6. What remains before hardware?

## Beginner Glossary

- **Actuator:** A motor, servo, or control surface that applies force.
- **Allocation:** Converting desired force/torque into motor commands.
- **Attitude:** Roll, pitch, and yaw orientation.
- **Body frame:** Coordinate frame attached to the aircraft.
- **Fault tolerance:** Staying controlled after damage or degradation.
- **HIL:** Hardware-in-the-loop; real hardware tested with simulated signals.
- **SITL:** Software-in-the-loop; simulated autopilot and vehicle.
- **State:** The variables that describe the system right now.
- **Wrench:** Combined force and torque vector.

## What Not To Claim Yet

Be careful in an application or interview:

- Do not say this is flight-ready.
- Do not say single vertical-motor loss is solved.
- Do not say the model exactly matches the real T1 Ranger.
- Do not say the FTC controller has a real fault estimator yet.

Better phrasing:

"I built a simulation framework for VTOL control research, including transition
aerodynamics and motor-fault campaigns. The current fault-tolerant allocator is
an upper-bound benchmark that shows graceful degradation under lift-motor loss
and clean handling of pusher-motor shutdown. The next step is estimator-based
fault detection and validation against Mission Planner logs."
