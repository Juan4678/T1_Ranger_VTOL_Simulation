# Master's Research Upgrade Plan

This repository is strongest if it presents a disciplined validation chain:

1. Python research model for controller ideas and failure envelopes.
2. Fault campaign metrics that compare nominal PID/LQR against robust or FTC variants.
3. Transition-aerodynamics checks that expose `Fa/(m*g)` and fixed-wing blend weight.
4. C++ implementation only after the Python evidence is convincing.
5. SITL/HIL validation before any real hardware work.

## Dynamics direction

Keep the current Newton-Euler/Euler-angle model as the beginner study baseline.
It is readable and useful for debugging controllers.

For the master's-level deliverable, use the Lagrange-d'Alembert quaternion
state-space model in `docs/lagrangian_quaternion_state_space.md` and
`src/dynamics/lagrangian_quaternion.py`. That gives a cleaner research narrative:

- derive the dynamics from kinetic and potential energy
- represent attitude with a unit quaternion instead of Euler angles
- write the nonlinear system as `x_dot = f(x, u)`
- linearize around hover or transition trim points as `delta x_dot = A delta x + B delta u`
- use a 12-state local attitude-error model for LQR/EKF work instead of treating
  the four quaternion components as independent states

Use `docs/t1_characterization_data_requirements.md` as the measurement plan for
turning the current surrogate into a better-parameterized T1 Ranger PNP model.

## Fault-tolerant control direction

The current `ftc` controller is a benchmark wrapper. It uses the nominal PID
controller to request a hover wrench, then reallocates that wrench around
estimated motor efficiencies. In the simulation loop, the estimate is supplied
by the fault profile. That is intentional: it gives an upper-bound benchmark for
what perfect motor-health awareness could achieve.

Next research-grade steps:

- Replace oracle motor efficiencies with an estimator from IMU residuals, motor
  commands, and measured angular acceleration.
- Compare `pid`, `lqr`, and `ftc` across repeatable fault campaigns.
- Add Monte Carlo wind/noise sweeps around the worst cases.
- Record metrics: RMS position error, max attitude excursion, yaw error,
  altitude envelope, and recovery time.
- Treat single vertical-lift rotor loss as a graceful-degradation case unless
  the real airframe has enough actuator redundancy to recover all axes.

## C++ boundary

The FX-405 repository is the C++ integration target. Keep exploratory robust
control work in Python first. Port only compact, well-tested control allocation
or estimator logic into C++ after simulation, SITL, and HIL results justify it.
