"""Minimal tests for PID and LQR controller output interfaces."""

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.configuration import load_json_config
from src.control import build_controller
from src.dynamics import VTOLDynamicsModel


def _load_model_and_config():
    config = load_json_config(REPO_ROOT / "config" / "default_params.json")
    model = VTOLDynamicsModel.from_config(config)
    return model, config


def test_pid_controller_output_dimensions_and_limits():
    model, config = _load_model_and_config()
    controller = build_controller("pid", model, config)
    state = np.zeros(12)
    state[2] = -2.0
    reference = {"position": np.array([0.0, 0.0, -2.0]), "velocity": np.zeros(3), "yaw": 0.0}
    commands = controller.compute_controls(state, reference)
    assert commands.shape == (model.rotor_count,)
    assert np.all(commands >= 0.0)
    assert np.all(commands <= model.max_thrusts + 1e-9)


def test_lqr_controller_output_dimensions_and_limits():
    model, config = _load_model_and_config()
    controller = build_controller("lqr", model, config)
    state = np.zeros(12)
    state[2] = -2.0
    reference = {"position": np.array([0.0, 0.0, -2.0]), "velocity": np.zeros(3), "yaw": 0.0}
    commands = controller.compute_controls(state, reference)
    assert commands.shape == (model.rotor_count,)
    assert np.all(commands >= 0.0)
    assert np.all(commands <= model.max_thrusts + 1e-9)
