"""Tests for the quaternion Lagrangian state-space model."""

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.configuration import load_json_config
from src.dynamics import (
    LagrangianQuaternionStateSpaceModel,
    normalize_quaternion,
    quaternion_to_rotation_matrix,
    rotation_vector_to_quaternion,
)


def _build_model() -> LagrangianQuaternionStateSpaceModel:
    config = load_json_config(REPO_ROOT / "config" / "default_params.json")
    return LagrangianQuaternionStateSpaceModel.from_config(config)


def _hover_state() -> np.ndarray:
    state = np.zeros(13, dtype=float)
    state[2] = -2.0
    state[6] = 1.0
    return state


def test_quaternion_state_derivative_shape():
    model = _build_model()
    derivatives = model.derivatives(0.0, _hover_state(), model.trim_hover_commands())
    assert derivatives.shape == (13,)


def test_quaternion_hover_trim_has_small_vertical_acceleration():
    model = _build_model()
    derivatives = model.derivatives(0.0, _hover_state(), model.trim_hover_commands())
    assert abs(derivatives[5]) < 0.5


def test_quaternion_is_normalized_before_dynamics():
    model = _build_model()
    state = _hover_state()
    state[6:10] = np.array([2.0, 0.0, 0.0, 0.0])

    normalized_state = model.normalize_state(state)

    assert np.allclose(normalized_state[6:10], np.array([1.0, 0.0, 0.0, 0.0]))


def test_identity_quaternion_rotation_matrix():
    rotation = quaternion_to_rotation_matrix([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(rotation, np.eye(3))


def test_rotation_vector_maps_to_unit_quaternion():
    quaternion = rotation_vector_to_quaternion([0.0, 0.0, 0.1])
    assert quaternion.shape == (4,)
    assert np.isclose(np.linalg.norm(quaternion), 1.0)


def test_normalize_quaternion_rejects_zero_norm():
    try:
        normalize_quaternion([0.0, 0.0, 0.0, 0.0])
    except ValueError as exc:
        assert "near-zero" in str(exc)
    else:
        raise AssertionError("Expected zero quaternion normalization to fail.")


def test_quaternion_linearization_shapes():
    model = _build_model()
    a_matrix, b_matrix = model.linearize(_hover_state(), model.trim_hover_commands())

    assert a_matrix.shape == (12, 12)
    assert b_matrix.shape == (12, model.rotor_count)
    assert np.isfinite(a_matrix).all()
    assert np.isfinite(b_matrix).all()


def test_error_state_linearization_matches_default_api():
    model = _build_model()
    state = _hover_state()
    commands = model.trim_hover_commands()

    default_a, default_b = model.linearize(state, commands)
    error_a, error_b = model.linearize_error_state(state, commands)

    assert np.allclose(default_a, error_a)
    assert np.allclose(default_b, error_b)


def test_error_state_roundtrip_keeps_quaternion_constraint():
    model = _build_model()
    state = _hover_state()
    error = np.zeros(12)
    error[6:9] = np.array([0.01, -0.02, 0.03])

    perturbed = model.apply_error_state(state, error)

    assert perturbed.shape == (13,)
    assert np.isclose(np.linalg.norm(perturbed[6:10]), 1.0)
