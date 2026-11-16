"""3D rotation math: quaternions, rotation matrices (SO(3)) and Euler angles.

Conventions (identical to SciPy's ``Rotation`` and to Pegasus Simulator):

* Quaternions are **scalar-last** ``q = [x, y, z, w]`` and Hamiltonian.
* A quaternion / rotation matrix ``R`` maps body coordinates to world
  coordinates: ``v_world = R @ v_body``.
* Euler angles are ``(roll, pitch, yaw)`` of the intrinsic Z-Y-X sequence:
  ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.

All functions are pure, operate on single (non-batched) rotations and never
modify their inputs.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from dronenav.core.types import FloatArray

_EPS = 1e-12


def _as_vec(values: npt.ArrayLike, size: int) -> FloatArray:
    """Convert ``values`` to a float64 vector of the given size."""
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.shape != (size,):
        raise ValueError(f"expected {size} elements, got shape {arr.shape}")
    return arr


# --------------------------------------------------------------------------- #
# Skew-symmetric helpers
# --------------------------------------------------------------------------- #
def hat(v: npt.ArrayLike) -> FloatArray:
    """Map a 3-vector to its skew-symmetric matrix, so that ``hat(a) @ b == cross(a, b)``."""
    x, y, z = _as_vec(v, 3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def vee(m: npt.ArrayLike) -> FloatArray:
    """Inverse of :func:`hat`: extract the 3-vector of a skew-symmetric matrix."""
    mat = np.asarray(m, dtype=np.float64)
    return np.array([mat[2, 1], mat[0, 2], mat[1, 0]])


# --------------------------------------------------------------------------- #
# Quaternion algebra
# --------------------------------------------------------------------------- #
def quat_normalize(q: npt.ArrayLike) -> FloatArray:
    """Return the unit quaternion with the same orientation, with ``w >= 0``.

    The sign is canonicalised (``q`` and ``-q`` encode the same rotation) so
    that comparisons in tests and logs are unambiguous.

    Raises:
        ValueError: If the quaternion has (near) zero norm.
    """
    quat = _as_vec(q, 4)
    norm = float(np.linalg.norm(quat))
    if norm < _EPS:
        raise ValueError("cannot normalise a zero quaternion")
    quat = quat / norm
    return -quat if quat[3] < 0.0 else quat


def quat_multiply(q1: npt.ArrayLike, q2: npt.ArrayLike) -> FloatArray:
    """Hamilton product ``q1 ⊗ q2`` (apply ``q2`` first, then ``q1``)."""
    x1, y1, z1, w1 = _as_vec(q1, 4)
    x2, y2, z2, w2 = _as_vec(q2, 4)
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def quat_conjugate(q: npt.ArrayLike) -> FloatArray:
    """Conjugate (equal to the inverse for unit quaternions)."""
    x, y, z, w = _as_vec(q, 4)
    return np.array([-x, -y, -z, w])


def quat_rotate(q: npt.ArrayLike, v: npt.ArrayLike) -> FloatArray:
    """Rotate vector ``v`` by the unit quaternion ``q`` (body -> world)."""
    quat = _as_vec(q, 4)
    vec = _as_vec(v, 3)
    u, w = quat[:3], quat[3]
    t = 2.0 * np.cross(u, vec)
    out: FloatArray = vec + w * t + np.cross(u, t)
    return out


def quat_from_axis_angle(axis: npt.ArrayLike, angle: float) -> FloatArray:
    """Quaternion of a rotation of ``angle`` radians about ``axis``.

    Raises:
        ValueError: If ``axis`` has (near) zero norm.
    """
    ax = _as_vec(axis, 3)
    norm = float(np.linalg.norm(ax))
    if norm < _EPS:
        raise ValueError("rotation axis must be non-zero")
    half = 0.5 * angle
    return np.concatenate((np.sin(half) * ax / norm, [np.cos(half)]))


def quat_to_rotmat(q: npt.ArrayLike) -> FloatArray:
    """Convert a quaternion to a 3x3 rotation matrix (the input is normalised first)."""
    x, y, z, w = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def rotmat_to_quat(r: npt.ArrayLike) -> FloatArray:
    """Convert a rotation matrix to a unit quaternion (Shepperd's method).

    The branch is selected on the largest diagonal term, which is numerically
    stable for every rotation.
    """
    m = np.asarray(r, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    return quat_normalize(q)


# --------------------------------------------------------------------------- #
# Euler angles (intrinsic Z-Y-X: yaw, then pitch, then roll)
# --------------------------------------------------------------------------- #
def euler_to_quat(roll: float, pitch: float, yaw: float) -> FloatArray:
    """Quaternion from Z-Y-X Euler angles [rad]."""
    cr, sr = np.cos(0.5 * roll), np.sin(0.5 * roll)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    return quat_normalize(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


def quat_to_euler(q: npt.ArrayLike) -> FloatArray:
    """Z-Y-X Euler angles ``[roll, pitch, yaw]`` [rad] of a quaternion.

    Pitch is clipped to ``[-pi/2, pi/2]``; at exactly +-90 deg pitch (gimbal
    lock) roll and yaw are not unique, which is why the rest of the stack works
    with quaternions / rotation matrices instead.
    """
    x, y, z, w = quat_normalize(q)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def yaw_from_quat(q: npt.ArrayLike) -> float:
    """Heading (yaw) angle [rad] of a quaternion, in ``(-pi, pi]``."""
    return float(quat_to_euler(q)[2])


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    wrapped = float(np.arctan2(np.sin(angle), np.cos(angle)))
    return np.pi if np.isclose(wrapped, -np.pi) else wrapped


# --------------------------------------------------------------------------- #
# Exponential / logarithm maps and integration
# --------------------------------------------------------------------------- #
def so3_exp(phi: npt.ArrayLike) -> FloatArray:
    """Rodrigues' formula: rotation vector ``phi`` (axis * angle) -> rotation matrix."""
    vec = _as_vec(phi, 3)
    angle = float(np.linalg.norm(vec))
    k = hat(vec)
    if angle < 1e-8:
        return np.eye(3) + k + 0.5 * k @ k  # second-order Taylor expansion
    rot: FloatArray = np.eye(3) + (np.sin(angle) / angle) * k + ((1.0 - np.cos(angle)) / angle**2) * k @ k
    return rot


def so3_log(r: npt.ArrayLike) -> FloatArray:
    """Inverse of :func:`so3_exp`: rotation matrix -> rotation vector (angle in ``[0, pi]``).

    Implemented through the quaternion, which is well conditioned for every
    angle including ``pi``.
    """
    q = rotmat_to_quat(r)
    u, w = q[:3], q[3]
    s = float(np.linalg.norm(u))
    if s < 1e-12:
        return 2.0 * u  # small-angle limit
    angle = 2.0 * np.arctan2(s, w)
    phi: FloatArray = angle * u / s
    return phi


def quat_integrate(q: npt.ArrayLike, omega_body: npt.ArrayLike, dt: float) -> FloatArray:
    """Integrate the attitude for ``dt`` seconds at constant body angular velocity.

    Uses the exact exponential map ``q(t+dt) = q(t) ⊗ exp(omega * dt / 2)``,
    which keeps the quaternion on the unit sphere.

    Args:
        q: Current attitude (body -> world).
        omega_body: Angular velocity in the body frame [rad/s].
        dt: Time step [s].

    Returns:
        The propagated unit quaternion.
    """
    omega = _as_vec(omega_body, 3)
    angle = float(np.linalg.norm(omega)) * dt
    if angle < 1e-12:
        return quat_normalize(q)
    dq = quat_from_axis_angle(omega, angle)
    return quat_normalize(quat_multiply(q, dq))


def orthonormalize(r: npt.ArrayLike) -> FloatArray:
    """Project a nearly-orthogonal matrix onto SO(3) (closest rotation, via SVD)."""
    u, _, vt = np.linalg.svd(np.asarray(r, dtype=np.float64))
    rot: FloatArray = u @ vt
    if np.linalg.det(rot) < 0.0:
        u[:, -1] *= -1.0
        rot = u @ vt
    return rot


def rotation_angle_between(q1: npt.ArrayLike, q2: npt.ArrayLike) -> float:
    """Smallest rotation angle [rad] taking orientation ``q1`` to ``q2``."""
    dq = quat_multiply(quat_conjugate(quat_normalize(q1)), quat_normalize(q2))
    return float(2.0 * np.arctan2(np.linalg.norm(dq[:3]), abs(dq[3])))
