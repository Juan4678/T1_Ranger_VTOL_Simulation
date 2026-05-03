"""Command-line entry point for hover simulations."""

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
from src.plotting import plot_simulation_results
from src.simulations.core import simulate_experiment
from src.simulations.trajectories import hover_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a hover simulation.")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "default_params.json"))
    parser.add_argument("--controller", default="pid", choices=["pid", "lqr", "advanced"])
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    config = load_json_config(args.config)
    model = VTOLDynamicsModel.from_config(config)
    controller = build_controller(args.controller, model, config, log_enabled=False)
    duration = args.duration or float(config["simulations"]["duration_s"])
    dt = float(config["simulations"]["dt"])
    initial_state = np.zeros(12, dtype=float)
    initial_state[2] = -2.0

    results = simulate_experiment(
        model=model,
        controller=controller,
        fault_profile=None,
        trajectory=hover_trajectory(position=(0.0, 0.0, -2.0)),
        initial_state=initial_state,
        duration=duration,
        dt=dt,
    )

    if args.save:
        save_dir = REPO_ROOT / config["simulations"]["save_dir"]
        results.save_npz(save_dir / f"hover_{args.controller}.npz")
        results.save_csv(save_dir / f"hover_{args.controller}.csv")
    if args.plot:
        plot_simulation_results(results, title=f"Hover ({args.controller.upper()})", show=True)


if __name__ == "__main__":
    main()
