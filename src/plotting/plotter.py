"""Matplotlib-based plotting utilities for simulation results."""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    """Creates compact research plots for one or two simulations."""

    def plot_simulation_results(self, results, title: str = "Hover", show: bool = False):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(title)

        axes[0, 0].plot(results.state[:, 0], results.state[:, 1], label="actual")
        axes[0, 0].plot(results.reference[:, 0], results.reference[:, 1], "--", label="reference")
        axes[0, 0].set_title("Trajectory (x-y)")
        axes[0, 0].set_xlabel("x [m]")
        axes[0, 0].set_ylabel("y [m]")
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(results.t, results.state[:, 2], label="z")
        axes[0, 1].plot(results.t, results.reference[:, 2], "--", label="z_ref")
        axes[0, 1].set_title("Altitude")
        axes[0, 1].set_xlabel("time [s]")
        axes[0, 1].set_ylabel("z [m]")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        for idx in range(results.control.shape[1]):
            axes[1, 0].plot(results.t, results.control[:, idx], label=f"rotor {idx}")
        axes[1, 0].set_title("Control signals")
        axes[1, 0].set_xlabel("time [s]")
        axes[1, 0].set_ylabel("thrust [N]")
        axes[1, 0].legend(loc="upper right", fontsize=8)
        axes[1, 0].grid(True, alpha=0.3)

        tracking_norm = np.linalg.norm(results.errors[:, 0:3], axis=1)
        axes[1, 1].plot(results.t, tracking_norm, label="position error norm")
        for idx in range(results.rotor_efficiency.shape[1]):
            axes[1, 1].plot(results.t, results.rotor_efficiency[:, idx], linestyle=":", alpha=0.7, label=f"eff {idx}")
        axes[1, 1].set_title("Tracking error and fault indicators")
        axes[1, 1].set_xlabel("time [s]")
        axes[1, 1].legend(loc="upper right", fontsize=8)
        axes[1, 1].grid(True, alpha=0.3)

        fig.tight_layout()
        if show:
            plt.show()
        return fig, axes

    def plot_comparison(self, results_pid, results_lqr, title: str = "PID vs LQR", show: bool = False):
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        fig.suptitle(title)

        axes[0].plot(results_pid.state[:, 0], results_pid.state[:, 1], label="PID")
        axes[0].plot(results_lqr.state[:, 0], results_lqr.state[:, 1], label="LQR")
        axes[0].plot(results_pid.reference[:, 0], results_pid.reference[:, 1], "--", label="Reference")
        axes[0].set_title("Trajectory comparison")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        pid_error = np.linalg.norm(results_pid.errors[:, 0:3], axis=1)
        lqr_error = np.linalg.norm(results_lqr.errors[:, 0:3], axis=1)
        axes[1].plot(results_pid.t, pid_error, label="PID")
        axes[1].plot(results_lqr.t, lqr_error, label="LQR")
        axes[1].set_title("Tracking error norm")
        axes[1].set_xlabel("time [s]")
        axes[1].set_ylabel("error [m]")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        fig.tight_layout()
        if show:
            plt.show()
        return fig, axes


def plot_simulation_results(results, title: str = "Hover", show: bool = False):
    return Plotter().plot_simulation_results(results, title=title, show=show)


def plot_comparison(results_pid, results_lqr, title: str = "PID vs LQR", show: bool = False):
    return Plotter().plot_comparison(results_pid, results_lqr, title=title, show=show)
