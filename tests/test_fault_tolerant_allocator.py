"""Tests for the fault-tolerant allocation benchmark controller."""

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


def test_fault_tolerant_allocator_reduces_failed_motor_command():
    model, config = _load_model_and_config()
    controller = build_controller("ftc", model, config)
    controller.update_rotor_efficiency_estimate(np.array([0.0, 1.0, 1.0, 1.0, 1.0]), time_s=6.0)
    state = np.zeros(12)
    state[2] = -2.0
    reference = {"position": np.array([0.0, 0.0, -2.0]), "velocity": np.zeros(3), "yaw": 0.0}
    commands = controller.compute_controls(state, reference, t=6.0)
    assert commands[0] == 0.0
    assert np.all(commands >= 0.0)
    assert np.all(commands <= model.max_thrusts + 1e-9)


def test_fault_tolerant_allocator_keeps_command_shape_for_tail_off():
    model, config = _load_model_and_config()
    controller = build_controller("ftc", model, config)
    efficiency = np.ones(model.rotor_count)
    efficiency[4] = 0.0
    controller.update_rotor_efficiency_estimate(efficiency, time_s=8.0)
    state = np.zeros(12)
    state[2] = -2.0
    reference = {"position": np.array([0.0, 0.0, -2.0]), "velocity": np.zeros(3), "yaw": 0.0}
    commands = controller.compute_controls(state, reference, t=8.0)
    assert commands.shape == (model.rotor_count,)
    assert commands[4] == 0.0
