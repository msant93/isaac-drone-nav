"""Tests for the Pegasus-compatible actuation model (allocation, mixer, drag)."""

from __future__ import annotations

import numpy as np
import pytest

from dronenav.model.quadrotor import QuadrotorModel


def pegasus_reference(model: QuadrotorModel, force: float, torque: np.ndarray) -> np.ndarray:
    """Line-by-line transcription of Pegasus' force_and_torques_to_velocities (v5.1.0)."""
    pos = model.config.rotor_positions
    k = np.full(4, model.k_f)
    aloc = np.zeros((4, 4))
    aloc[0, :] = k
    aloc[1, :] = pos[:, 1] * k
    aloc[2, :] = -pos[:, 0] * k
    aloc[3, :] = model.k_m * model.config.rotor_directions
    sq = np.linalg.pinv(aloc) @ np.array([force, *torque])
    sq[sq < 0] = 0.0
    max_sq = model.w_max**2
    if np.max(sq) >= max_sq:
        sq = sq / np.maximum(np.max(sq) / max_sq, 1.0)
    return np.sqrt(sq)


def test_hover_speeds(model: QuadrotorModel) -> None:
    w = model.hover_rotor_speeds()
    wrench = model.wrench_from_rotor_speeds(w)
    assert wrench.thrust == pytest.approx(model.weight)
    np.testing.assert_allclose(wrench.torque, 0.0, atol=1e-12)
    # About sqrt(m g / (4 k_f)) ~ 660 rad/s for the Iris, well below the limit.
    assert 600.0 < float(np.mean(w)) < 720.0


def test_mixer_round_trip_for_feasible_wrench(model: QuadrotorModel, rng: np.random.Generator) -> None:
    for _ in range(50):
        thrust = rng.uniform(0.6, 1.4) * model.weight
        torque = rng.uniform(-0.2, 0.2, size=3) * np.array([1.0, 1.0, 0.05])
        w = model.rotor_speeds_from_wrench(thrust, torque)
        if np.any(w <= 0.0) or np.any(w >= model.w_max):
            continue  # saturated: not a pure round trip
        wrench = model.wrench_from_rotor_speeds(w)
        assert wrench.thrust == pytest.approx(thrust, rel=1e-9)
        np.testing.assert_allclose(wrench.torque, torque, atol=1e-9)


def test_mixer_matches_pegasus_reference(model: QuadrotorModel, rng: np.random.Generator) -> None:
    for _ in range(100):
        thrust = rng.uniform(0.0, 3.5) * model.weight  # includes saturated cases
        torque = rng.normal(scale=[1.0, 1.0, 0.1])
        np.testing.assert_allclose(
            model.rotor_speeds_from_wrench(thrust, torque), pegasus_reference(model, thrust, torque), rtol=1e-12
        )


def test_saturation_respects_limit(model: QuadrotorModel) -> None:
    w = model.rotor_speeds_from_wrench(10.0 * model.weight, np.zeros(3))
    np.testing.assert_allclose(w, model.w_max)
    assert np.all(model.rotor_speeds_from_wrench(-5.0, np.zeros(3)) == 0.0)


def test_torque_signs_follow_flu_convention(model: QuadrotorModel) -> None:
    hover = model.hover_rotor_speeds()
    pos, dirs = model.config.rotor_positions, model.config.rotor_directions

    left = hover.copy()
    left[pos[:, 1] > 0] += 20.0  # speed up the left rotors (y > 0)
    assert model.wrench_from_rotor_speeds(left).torque[0] > 0.0  # positive roll: right side down

    rear = hover.copy()
    rear[pos[:, 0] < 0] += 20.0  # speed up the rear rotors
    assert model.wrench_from_rotor_speeds(rear).torque[1] > 0.0  # positive pitch: nose down (FLU)

    ccw = hover.copy()
    ccw[dirs > 0] += 20.0
    assert model.wrench_from_rotor_speeds(ccw).torque[2] > 0.0


def test_clip_and_drag(model: QuadrotorModel) -> None:
    np.testing.assert_allclose(model.clip_rotor_speeds([-10, 0, 500, 5000]), [0, 0, 500, model.w_max])
    np.testing.assert_allclose(model.drag_force_body([1.0, -2.0, 3.0]), [-0.5, 0.6, 0.0])
    with pytest.raises(ValueError):
        model.clip_rotor_speeds([1.0, 2.0])


def test_thrust_to_weight(model: QuadrotorModel) -> None:
    assert model.thrust_to_weight == pytest.approx(model.max_thrust / (1.52 * 9.81))
