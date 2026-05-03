# T1 Ranger VTOL Research Framework (Python)

This repository is a lightweight research scaffold for autonomous VTOL control and fault-tolerant control studies inspired by the **Heewing T1 Ranger VTOL**.

It is intentionally **research-oriented rather than hardware-ready**. The model is simplified, the parameters are approximate, and the controllers are meant to provide a clean baseline for later refinement.

## Research goals

- Simulate a simplified VTOL rigid-body model in hover and low-speed trajectory tracking.
- Compare baseline controllers such as PID and LQR.
- Inject actuator and sensor faults, including rotor-loss scenarios.
- Tune gains with lightweight optimization tools.
- Export results for later analysis and thesis-quality plots.

## Key assumptions

- The dynamics are a simplified 6-DoF rigid-body model with Euler-angle attitude states.
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
```

## Suggested workflow

1. Start in `notebooks/01_baselines.ipynb` to compare PID and LQR in hover.
2. Move to `notebooks/02_fault_robustness.ipynb` for rotor-loss experiments.
3. Use `notebooks/03_optimization.ipynb` to tune gains and define custom costs.
4. Extend `PlaceholderAdvancedController` with robust, FTC, or MPC logic.

## Next extensions

- Replace Euler angles with quaternions if you need aggressive maneuvers.
- Add transition-mode aerodynamics and fixed-wing lift approximations.
- Linearize the model automatically around trimmed operating points.
- Add Monte Carlo disturbance sweeps and estimator prototypes.
