# Study Guide: T1 Ranger VTOL Research Framework

This guide is written for a mechatronics background with beginner-level VTOL
software experience. The goal is to help you move from "what is this code?" to
"I can explain, run, modify, and defend this project in a master's interview."

## Best Way To Read This

Use **VS Code** for the best experience:

- Open the folder `t1-ranger-vtol-research-python`.
- Open this file.
- Press `Ctrl+Shift+V` for Markdown Preview.
- Use the terminal inside VS Code for the commands.

For equations, VS Code's built-in preview may show raw LaTeX. If you want nicer
math rendering, use one of these:

- GitHub web preview after pushing the repo.
- Obsidian with the repo folder opened as a vault.
- VS Code with a Markdown math extension.

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

The model is a simplified VTOL surrogate:

- Four vertical lift rotors handle hover.
- One pusher motor represents forward flight/transition experiments.
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
m \dot{v} = R(\phi,\theta,\psi) F_b + F_d + mg
$$

$$
I \dot{\omega} = \tau_b - \omega \times I\omega - D_\omega \omega
$$

This Newton-Euler form stays in the guide on purpose. It is the fastest way to
learn how forces, torques, gravity, drag, and rotor allocation interact.

The research deliverable then moves to a Lagrange-d'Alembert state-space model
with quaternion attitude:

- `docs/lagrangian_quaternion_state_space.md`
- `src/dynamics/lagrangian_quaternion.py`

Read that after you are comfortable explaining the equations above. The physics
is connected, but the quaternion version is the better foundation for aggressive
attitude motion, state estimation, and thesis-level linearization.

In code, these live in:

- `src/dynamics/vtol_model.py`
- `VTOLDynamicsModel.derivatives(...)`

What to study there:

- How rotor thrust becomes body force.
- How rotor position creates torque with a cross product.
- How gravity and drag enter the translational dynamics.
- How the allocation matrix maps motor thrusts to wrench.

## Rotor Allocation

Each rotor contributes force:

$$
F_i = u_i a_i
$$

Where:

- `u_i` is rotor thrust in Newtons.
- `a_i` is the rotor axis in body coordinates.

Each rotor also creates torque:

$$
\tau_i = r_i \times F_i + \tau_{yaw,i}
$$

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
- `dual_rotor_transition`
- `tail_pusher_off`
- `rear_right_off`

The tail/pusher-off case is useful because it tests a forward-flight actuator
failure without destroying hover lift. A vertical lift rotor loss is much harder
because it removes both lift and torque authority.

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
C:\Python314\python.exe src\simulations\run_motor_fault_campaign.py --duration 10 --controllers pid ftc --faults single_rotor_hover tail_pusher_off rear_right_off
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
C:\Python314\python.exe src\simulations\run_motor_fault_campaign.py --duration 10 --controllers pid ftc --faults single_rotor_hover tail_pusher_off rear_right_off
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
