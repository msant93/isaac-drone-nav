"""Tests for ENU/NED and FLU/FRD conversions."""

from __future__ import annotations

import numpy as np

from dronenav.core import frames as f
from dronenav.core.math3d import euler_to_quat, quat_rotate, rotation_angle_between


def test_known_vectors() -> None:
    np.testing.assert_allclose(f.enu_to_ned([1.0, 2.0, 3.0]), [2.0, 1.0, -3.0])
    np.testing.assert_allclose(f.flu_to_frd([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0])


def test_involutions(rng: np.random.Generator) -> None:
    v = rng.normal(size=3)
    np.testing.assert_allclose(f.ned_to_enu(f.enu_to_ned(v)), v)
    np.testing.assert_allclose(f.frd_to_flu(f.flu_to_frd(v)), v)


def test_attitude_conversion_consistency(rng: np.random.Generator) -> None:
    for _ in range(20):
        q = euler_to_quat(*rng.uniform(-1.0, 1.0, size=3))
        q_ned = f.attitude_enu_flu_to_ned_frd(q)
        v_flu = rng.normal(size=3)
        # Rotating an FRD body vector with the NED attitude == converting the ENU result.
        np.testing.assert_allclose(quat_rotate(q_ned, f.flu_to_frd(v_flu)), f.enu_to_ned(quat_rotate(q, v_flu)))
        assert rotation_angle_between(f.attitude_ned_frd_to_enu_flu(q_ned), q) < 1e-12


def test_level_east_facing_is_ned_yaw_90() -> None:
    # Facing east: ENU yaw 0 <=> NED yaw +90 deg (heading measured from north).
    q_ned = f.attitude_enu_flu_to_ned_frd(euler_to_quat(0.0, 0.0, 0.0))
    np.testing.assert_allclose(quat_rotate(q_ned, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)
