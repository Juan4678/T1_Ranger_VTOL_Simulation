"""Run controller comparisons across motor-deactivation scenarios."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.configuration import load_json_config
from src.control import build_controller
from src.dynamics import VTOLDynamicsModel
from src.evaluation import summarize_fault_response
from src.faults import FaultInjector
from src.simulations.core import simulate_experiment
from src.simulations.trajectories import hover_trajectory, line_trajectory


def _trajectory_for_fault(name: str, duration_s: float):
    if "transition" in name or "pusher" in name or "tail" in name:
        return line_trajectory(start=(0.0, 0.0, -2.0), end=(10.0, 0.0, -2.0), duration=duration_s)
    return hover_trajectory(position=(0.0, 0.0, -2.0))


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare nominal and fault-tolerant controllers under motor faults.")
    parser.add_argument("--config", default=str(REPO_ROOT / "config" / "default_params.json"))
    parser.add_argument("--controllers", nargs="+", default=["pid", "ftc"], choices=["pid", "lqr", "ftc"])
    parser.add_argument(
        "--faults",
        nargs="+",
        default=["single_rotor_hover", "tail_pusher_off", "dual_rotor_transition"],
    )
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--save", action="store_true", help="Write CSV/NPZ outputs under the configured results folder.")
    args = parser.parse_args()

    config = load_json_config(args.config)
    model = VTOLDynamicsModel.from_config(config)
    duration = args.duration or float(config["simulations"]["duration_s"])
    dt = float(config["simulations"]["dt"])
    initial_state = np.zeros(12, dtype=float)
    initial_state[2] = -2.0

    summaries = []
    results_dir = REPO_ROOT / config["simulations"]["save_dir"] / "motor_fault_campaign"

    for fault_name in args.faults:
        fault_config = config["fault_examples"][fault_name]
        trajectory = _trajectory_for_fault(fault_name, duration)
        for controller_name in args.controllers:
            controller = build_controller(controller_name, model, config, log_enabled=True)
            fault_profile = FaultInjector.from_dict(fault_config, model.rotor_count)
            results = simulate_experiment(
                model=model,
                controller=controller,
                fault_profile=fault_profile,
                trajectory=trajectory,
                initial_state=initial_state,
                duration=duration,
                dt=dt,
            )
            summary = summarize_fault_response(results)
            summaries.append(summary)

            if args.save:
                base = f"{fault_name}_{controller_name}"
                results.save_npz(results_dir / f"{base}.npz")
                results.save_csv(results_dir / f"{base}.csv")

    print("controller,fault,rms_err,max_err,final_err,max_tilt_deg,max_yaw_err_deg,failed")
    for summary in summaries:
        print(
            f"{summary.controller},{summary.fault},"
            f"{summary.rms_position_error_m:.3f},{summary.max_position_error_m:.3f},"
            f"{summary.final_position_error_m:.3f},{summary.max_tilt_deg:.2f},"
            f"{summary.max_yaw_error_deg:.2f},{summary.failed}"
        )

    if args.save:
        results_dir.mkdir(parents=True, exist_ok=True)
        with (results_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
            fieldnames = list(summaries[0].to_dict().keys()) if summaries else []
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for summary in summaries:
                writer.writerow(summary.to_dict())


if __name__ == "__main__":
    main()
