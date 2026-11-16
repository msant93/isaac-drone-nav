"""Rigid-body dynamics of a multirotor (Newton-Euler equations).

State vector ``x`` (13 elements)::

    x = [p (3, ENU), v (3, ENU), q (4, [x y z w] body->world), w (3, FLU body rates)]

Equations of motion for a body force ``F_b`` (rotor thrust along body z plus
drag) and body torque ``tau_b``::

    p_dot = v
    v_dot = R(q) @ F_b / m + g + a_ext
    q_dot = 1/2 * q ⊗ [w, 0]
    w_dot = J^-1 (tau_b - w x (J w))

The wrench from the rotors is held constant during a step (zero-order hold),
exactly like a physics engine that receives one command per step. The drag
force depends on the velocity and is therefore evaluated inside the
derivative.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from dronenav.core.math3d import quat_integrate, quat_normalize
from dronenav.core.types import FloatArray

StateDerivative = Callable[[FloatArray], FloatArray]


def pack_state(
    position: FloatArray, velocity: FloatArray, attitude: FloatArray, angular_velocity: FloatArray
) -> FloatArray:
    """Concatenate the state components into the 13-element state vector."""
    return np.concatenate((position, velocity, attitude, angular_velocity)).astype(np.float64)


def unpack_state(x: FloatArray) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """Split the 13-element state vector into ``(p, v, q, w)`` (copies)."""
    return x[0:3].copy(), x[3:6].copy(), x[6:10].copy(), x[10:13].copy()


def make_derivative(
    mass: float,
    inertia: FloatArray,
    inertia_inv: FloatArray,
    gravity: float,
    thrust: float,
    torque: FloatArray,
    drag: FloatArray,
    external_accel: FloatArray,
) -> StateDerivative:
    """Build ``f(x)`` for a constant rotor wrench over the step.

    Args:
        mass: Vehicle mass [kg].
        inertia: Inertia matrix [kg m^2].
        inertia_inv: Inverse of ``inertia`` (kept for API symmetry; unused for diagonal tensors).
        gravity: Gravity magnitude [m/s^2].
        thrust: Collective rotor thrust along body z [N].
        torque: Rotor torque in the body frame [N m].
        drag: Linear drag coefficients ``d`` (``F_drag = -diag(d) v_body``) [N s/m].
        external_accel: Additional world-frame acceleration (e.g. wind) [m/s^2].

    Returns:
        The derivative function of the 13-element state vector.
    """
    gz = -gravity
    ax_ext, ay_ext, az_ext = (float(a) for a in external_accel)
    dx, dy, dz = (float(d) for d in drag)
    tx, ty, tz = (float(t) for t in torque)
    jxx, jyy, jzz = float(inertia[0, 0]), float(inertia[1, 1]), float(inertia[2, 2])
    if not np.allclose(inertia, np.diag(np.diag(inertia))):
        raise ValueError("the simulator expects a diagonal inertia tensor (principal axes = body axes)")
    inv_m = 1.0 / mass

    def derivative(x: FloatArray) -> FloatArray:
        # Scalar arithmetic on purpose: for 3-vectors it is several times faster
        # than NumPy calls, and this function is evaluated 4x per RK4 step.
        _, _, _, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz = x.tolist()
        # Rotation matrix of the (unit) quaternion, body -> world.
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qw * qz)
        r02 = 2.0 * (qx * qz + qw * qy)
        r10 = 2.0 * (qx * qy + qw * qz)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qw * qx)
        r20 = 2.0 * (qx * qz - qw * qy)
        r21 = 2.0 * (qy * qz + qw * qx)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        # Body-frame velocity and body force (thrust along +z, linear drag).
        ub = r00 * vx + r10 * vy + r20 * vz
        vb = r01 * vx + r11 * vy + r21 * vz
        wb = r02 * vx + r12 * vy + r22 * vz
        fx, fy, fz = -dx * ub, -dy * vb, thrust - dz * wb
        # Translational dynamics (world frame).
        ax = (r00 * fx + r01 * fy + r02 * fz) * inv_m + ax_ext
        ay = (r10 * fx + r11 * fy + r12 * fz) * inv_m + ay_ext
        az = (r20 * fx + r21 * fy + r22 * fz) * inv_m + gz + az_ext
        # Quaternion kinematics: q_dot = 1/2 q ⊗ [w, 0] (scalar-last).
        qdx = 0.5 * (qw * wx + qy * wz - qz * wy)
        qdy = 0.5 * (qw * wy + qz * wx - qx * wz)
        qdz = 0.5 * (qw * wz + qx * wy - qy * wx)
        qdw = -0.5 * (qx * wx + qy * wy + qz * wz)
        # Rotational dynamics (Euler's equations): J w_dot = tau - w x (J w).
        hx, hy, hz = jxx * wx, jyy * wy, jzz * wz
        wdx = (tx - (wy * hz - wz * hy)) / jxx
        wdy = (ty - (wz * hx - wx * hz)) / jyy
        wdz = (tz - (wx * hy - wy * hx)) / jzz
        return np.array([vx, vy, vz, ax, ay, az, qdx, qdy, qdz, qdw, wdx, wdy, wdz])

    return derivative


def rk4_step(f: StateDerivative, x: FloatArray, dt: float) -> FloatArray:
    """Classic 4th-order Runge-Kutta step; the quaternion is re-normalised afterwards."""
    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x_next[6:10] = quat_normalize(x_next[6:10])
    return x_next


def semi_implicit_euler_step(f: StateDerivative, x: FloatArray, dt: float) -> FloatArray:
    """Semi-implicit (symplectic) Euler step, the scheme used by most game physics engines.

    Velocities are updated first and the new velocities are used to update the
    positions; the attitude is propagated with the exact exponential map.
    """
    dx = f(x)
    x_next = x.copy()
    x_next[3:6] = x[3:6] + dt * dx[3:6]
    x_next[10:13] = x[10:13] + dt * dx[10:13]
    x_next[0:3] = x[0:3] + dt * x_next[3:6]
    x_next[6:10] = quat_integrate(x[6:10], x_next[10:13], dt)
    return x_next
