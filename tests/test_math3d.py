"""Tests for quaternion / SO(3) / Euler utilities, cross-checked against SciPy."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dronenav.core import math3d as m


def random_quats(rng: np.random.Generator, n: int = 50) -> list[np.ndarray]:
    return [m.quat_normalize(q) for q in rng.normal(size=(n, 4))]


def test_hat_vee_and_cross_product(rng: np.random.Generator) -> None:
    a, b = rng.normal(size=3), rng.normal(size=3)
    np.testing.assert_allclose(m.hat(a) @ b, np.cross(a, b))
    np.testing.assert_allclose(m.vee(m.hat(a)), a)


def test_quat_to_rotmat_matches_scipy(rng: np.random.Generator) -> None:
    for q in random_quats(rng):
        np.testing.assert_allclose(m.quat_to_rotmat(q), Rotation.from_quat(q).as_matrix(), atol=1e-12)


def test_rotmat_quat_round_trip(rng: np.random.Generator) -> None:
    for q in random_quats(rng):
        np.testing.assert_allclose(m.rotmat_to_quat(m.quat_to_rotmat(q)), q, atol=1e-12)


@pytest.mark.parametrize("angle", [np.pi, np.pi - 1e-9, 1e-10])
def test_rotmat_to_quat_edge_angles(angle: float) -> None:
    for axis in np.eye(3):
        r = Rotation.from_rotvec(axis * angle).as_matrix()
        assert m.rotation_angle_between(m.rotmat_to_quat(r), Rotation.from_matrix(r).as_quat()) < 1e-9


def test_quat_multiply_composes_rotations(rng: np.random.Generator) -> None:
    q1, q2 = random_quats(rng, 2)
    np.testing.assert_allclose(
        m.quat_to_rotmat(m.quat_multiply(q1, q2)), m.quat_to_rotmat(q1) @ m.quat_to_rotmat(q2), atol=1e-12
    )


def test_quat_rotate_and_conjugate(rng: np.random.Generator) -> None:
    q = random_quats(rng, 1)[0]
    v = rng.normal(size=3)
    np.testing.assert_allclose(m.quat_rotate(q, v), m.quat_to_rotmat(q) @ v, atol=1e-12)
    np.testing.assert_allclose(m.quat_rotate(m.quat_conjugate(q), m.quat_rotate(q, v)), v, atol=1e-12)


def test_euler_round_trip_and_scipy_convention(rng: np.random.Generator) -> None:
    for _ in range(50):
        roll, yaw = rng.uniform(-np.pi, np.pi, size=2)
        pitch = rng.uniform(-1.5, 1.5)
        q = m.euler_to_quat(roll, pitch, yaw)
        ref = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_quat()
        assert m.rotation_angle_between(q, ref) < 1e-12
        np.testing.assert_allclose(m.quat_to_euler(q), [roll, pitch, yaw], atol=1e-9)


def test_yaw_rotates_x_towards_y() -> None:
    # ENU: a +90 deg yaw turns the body x axis (forward) towards world +y (north).
    q = m.euler_to_quat(0.0, 0.0, np.pi / 2)
    np.testing.assert_allclose(m.quat_rotate(q, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)
    assert m.yaw_from_quat(q) == pytest.approx(np.pi / 2)


def test_so3_exp_log(rng: np.random.Generator) -> None:
    for _ in range(50):
        phi = rng.normal(size=3)
        phi *= rng.uniform(0.0, np.pi - 1e-3) / np.linalg.norm(phi)
        r = m.so3_exp(phi)
        np.testing.assert_allclose(r, Rotation.from_rotvec(phi).as_matrix(), atol=1e-12)
        np.testing.assert_allclose(m.so3_log(r), phi, atol=1e-9)
    np.testing.assert_allclose(m.so3_exp([1e-10, 0, 0]), np.eye(3), atol=1e-9)


def test_quat_integrate_constant_rate() -> None:
    q = np.array([0.0, 0.0, 0.0, 1.0])
    omega = np.array([0.0, 0.0, 0.5])
    for _ in range(1000):
        q = m.quat_integrate(q, omega, 0.001)
    assert m.yaw_from_quat(q) == pytest.approx(0.5, abs=1e-12)
    assert np.linalg.norm(q) == pytest.approx(1.0)


def test_orthonormalize_projects_to_so3(rng: np.random.Generator) -> None:
    r = m.quat_to_rotmat(random_quats(rng, 1)[0]) + 1e-3 * rng.normal(size=(3, 3))
    p = m.orthonormalize(r)
    np.testing.assert_allclose(p.T @ p, np.eye(3), atol=1e-12)
    assert np.linalg.det(p) == pytest.approx(1.0)


def test_wrap_angle() -> None:
    assert m.wrap_angle(3 * np.pi) == pytest.approx(np.pi)
    assert m.wrap_angle(-np.pi) == pytest.approx(np.pi)
    assert m.wrap_angle(-0.5) == pytest.approx(-0.5)


def test_invalid_inputs_raise() -> None:
    with pytest.raises(ValueError):
        m.quat_normalize([0, 0, 0, 0])
    with pytest.raises(ValueError):
        m.quat_from_axis_angle([0, 0, 0], 1.0)
    with pytest.raises(ValueError):
        m.hat([1.0, 2.0])
