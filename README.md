# T1 Ranger VTOL Research Framework (Python)

This repository is a lightweight research scaffold for autonomous VTOL control and fault-tolerant control studies inspired by the **Heewing T1 Ranger VTOL**.

It is intentionally **research-oriented rather than hardware-ready**. The model is simplified, the parameters are approximate, and the controllers are meant to provide a clean baseline for later refinement.

## Start here if you are learning

Read [STUDY_GUIDE.md](STUDY_GUIDE.md) first. It explains the repository from
VTOL dynamics to fault-tolerant control in a beginner-friendly order.

## Research goals

- Simulate a simplified VTOL rigid-body model in hover and low-speed trajectory tracking.
- Deliver a quaternion-based Lagrangian state-space model for research-grade derivation and linearization.
- Compare baseline controllers such as PID and LQR.
- Inject actuator and sensor faults, including rotor-loss scenarios.
- Tune gains with lightweight optimization tools.
- Export results for later analysis and thesis-quality plots.

## Key assumptions

- The study baseline is a simplified 6-DoF Newton-Euler rigid-body model with Euler-angle attitude states.
- The research dynamics layer derives a Lagrange-d'Alembert quaternion state-space model in `docs/lagrangian_quaternion_state_space.md` and `src/dynamics/lagrangian_quaternion.py`.
- The rotor layout is a research surrogate loosely inspired by the T1 Ranger VTOL configuration.
- Hover lift is modeled primarily through four vertical rotors, with an additional pusher rotor reserved for later transition experiments.
- This repository is for simulation and controller research only. It is **not** flight-ready and should not be used directly on hardware.

## Repository structure

```text
.
├── README.md
├── requirements.txt
├── config/
│   └── default_params.json
├── src/
│   ├── configuration.py
│   ├── dynamics/
│   ├── control/
│   ├── faults/
│   ├── optim/
│   ├── plotting/
│   └── simulations/
├── notebooks/
└── tests/
```

## Quick start

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

### Run a hover baseline

```bash
python src/simulations/run_hover.py --controller pid --plot
python src/simulations/run_hover.py --controller lqr --plot
```

### Run a trajectory tracking experiment

```bash
python src/simulations/run_trajectory_tracking.py --controller lqr --trajectory circle --plot
```

### Run a fault scenario

```bash
python src/simulations/run_fault_scenarios.py --controller pid --fault single_rotor_hover --plot
python src/simulations/run_fault_scenarios.py --controller ftc --fault tail_pusher_off
python src/simulations/run_motor_fault_campaign.py --controllers pid ftc --faults single_rotor_hover tail_pusher_off rear_right_off
```

The `ftc` controller is a Python-side research benchmark. It wraps the nominal
PID law and reallocates the requested hover wrench around estimated motor
efficiencies. This is intentionally separate from the FX-405 C++ controller
template; use it to test ideas before deciding what should be ported to C++.

For a master's application, the recommended narrative is in
`docs/masters_research_upgrade_plan.md`: Python evidence first, C++ port second,
then SITL/HIL before hardware.

Use `docs/t1_characterization_data_requirements.md` as the measurement checklist
for replacing approximate parameters with a better T1 Ranger PNP model.

### Inspect the Lagrangian/quaternion state-space model

Read `docs/lagrangian_quaternion_state_space.md` for the derivation. The
numerical model can be imported as:

```python
from src.dynamics import LagrangianQuaternionStateSpaceModel
```

It evaluates the 13-state nonlinear quaternion model and numerically linearizes
it into the 12-state local attitude-error form needed for LQR/EKF work.

### Run a transition-aerodynamics/HIL packet check

```bash
python src/simulations/run_transition_hil.py
python -m unittest tests.test_transition_aero
```

This transition check reports `Fa/(m*g)`, lift-support ratio, and fixed-wing
blend weight for a tilt-gondola transition. The last printed packet is compatible
with the FX-405 transition-reference HIL bridge.

## Suggested workflow

1. Start in `notebooks/01_baselines.ipynb` to compare PID and LQR in hover.
2. Move to `notebooks/02_fault_robustness.ipynb` for rotor-loss experiments.
3. Use `notebooks/03_optimization.ipynb` to tune gains and define custom costs.
4. Extend `PlaceholderAdvancedController` with robust, FTC, or MPC logic.

## Next extensions

- Replace controller-facing Euler-angle assumptions with quaternion error-state logic.
- Add transition-mode aerodynamics and fixed-wing lift approximations.
- Add controller examples that consume the 12-state quaternion error model.
- Add Monte Carlo disturbance sweeps and estimator prototypes.
