"""Command-line entry point for fault-injection experiments."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.configuration import load_json_config
from src.control import build_controller
from src.dynamics import VTOLDynamicsModel
from src.faults import FaultInjector
from src.plotting import plot_simulation_results
from src.simulations.core import simulate_experiment
from src.simulations.trajectories import hover_trajectory, line_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a fault scenario experiment.")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "default_params.json"))
    parser.add_argument("--controller", default="pid", choices=["pid", "lqr", "advanced", "ftc"])
    parser.add_argument(
        "--fault",
        default="single_rotor_hover",
        choices=["single_rotor_hover", "dual_rotor_transition", "tail_pusher_off", "rear_right_off"],
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    config = load_json_config(args.config)
    model = VTOLDynamicsModel.from_config(config)
    controller = build_controller(args.controller, model, config, log_enabled=False)
    duration = args.duration or float(config["simulations"]["duration_s"])
    dt = float(config["simulations"]["dt"])
    fault_profile = FaultInjector.from_dict(config["fault_examples"][args.fault], model.rotor_count)

    trajectory = hover_trajectory(position=(0.0, 0.0, -2.0))
    if args.fault in {"dual_rotor_transition", "tail_pusher_off"}:
        trajectory = line_trajectory(start=(0.0, 0.0, -2.0), end=(12.0, 0.0, -2.0), duration=duration)

    initial_state = np.zeros(12, dtype=float)
    initial_state[2] = -2.0

    results = simulate_experiment(
        model=model,
        controller=controller,
        fault_profile=fault_profile,
        trajectory=trajectory,
        initial_state=initial_state,
        duration=duration,
        dt=dt,
    )

    if args.save:
        save_dir = REPO_ROOT / config["simulations"]["save_dir"]
        base = f"fault_{args.fault}_{args.controller}"
        results.save_npz(save_dir / f"{base}.npz")
        results.save_csv(save_dir / f"{base}.csv")
    if args.plot:
        plot_simulation_results(results, title=f"Fault: {args.fault} ({args.controller.upper()})", show=True)


if __name__ == "__main__":
    main()
