"""Minimal tests for the simplified VTOL dynamics."""

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.configuration import load_json_config
from src.dynamics import VTOLDynamicsModel


def test_dynamics_derivative_shape():
    config = load_json_config(REPO_ROOT / "config" / "default_params.json")
    model = VTOLDynamicsModel.from_config(config)
    state = np.zeros(12)
    state[2] = -2.0
    commands = model.trim_hover_commands()
    derivatives = model.derivatives(0.0, state, commands)
    assert derivatives.shape == (12,)


def test_hover_trim_has_small_vertical_acceleration():
    config = load_json_config(REPO_ROOT / "config" / "default_params.json")
    model = VTOLDynamicsModel.from_config(config)
    state = np.zeros(12)
    state[2] = -2.0
    derivatives = model.derivatives(0.0, state, model.trim_hover_commands())
    assert abs(derivatives[5]) < 0.5
