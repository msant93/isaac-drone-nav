"""Coordinate-frame conventions and conversions.

The project uses **ENU** (x East, y North, z Up) for the world frame and
**FLU** (x Forward, y Left, z Up) for the body frame, the native conventions of
Isaac Sim and of Pegasus Simulator. PX4 / aerospace code uses **NED** world
and **FRD** body frames; the conversions below are only needed at that
boundary (optional PX4 phase) and in documentation/tests.

Both world conversions (ENU <-> NED) and body conversions (FLU <-> FRD) are
involutions: applying them twice returns the original value.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from dronenav.core.math3d import quat_multiply, quat_normalize, rotmat_to_quat
from dronenav.core.types import FloatArray

#: Rotation matrix taking ENU coordinates to NED coordinates (and back).
ENU_TO_NED: FloatArray = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
#: Rotation matrix taking FLU coordinates to FRD coordinates (and back).
FLU_TO_FRD: FloatArray = np.diag([1.0, -1.0, -1.0])

_Q_ENU_TO_NED = rotmat_to_quat(ENU_TO_NED)
_Q_FLU_TO_FRD = rotmat_to_quat(FLU_TO_FRD)


def enu_to_ned(v: npt.ArrayLike) -> FloatArray:
    """Convert a world vector from ENU to NED (``[x, y, z] -> [y, x, -z]``)."""
    out: FloatArray = ENU_TO_NED @ np.asarray(v, dtype=np.float64)
    return out


def ned_to_enu(v: npt.ArrayLike) -> FloatArray:
    """Convert a world vector from NED to ENU."""
    out: FloatArray = ENU_TO_NED @ np.asarray(v, dtype=np.float64)
    return out


def flu_to_frd(v: npt.ArrayLike) -> FloatArray:
    """Convert a body vector from FLU to FRD (``[x, y, z] -> [x, -y, -z]``)."""
    out: FloatArray = FLU_TO_FRD @ np.asarray(v, dtype=np.float64)
    return out


def frd_to_flu(v: npt.ArrayLike) -> FloatArray:
    """Convert a body vector from FRD to FLU."""
    out: FloatArray = FLU_TO_FRD @ np.asarray(v, dtype=np.float64)
    return out


def attitude_enu_flu_to_ned_frd(q: npt.ArrayLike) -> FloatArray:
    """Convert an attitude (FLU body -> ENU world) to PX4's convention (FRD -> NED).

    ``R_ned_frd = ENU_TO_NED @ R_enu_flu @ FLU_TO_FRD``.
    """
    return quat_normalize(quat_multiply(quat_multiply(_Q_ENU_TO_NED, q), _Q_FLU_TO_FRD))


def attitude_ned_frd_to_enu_flu(q: npt.ArrayLike) -> FloatArray:
    """Inverse of :func:`attitude_enu_flu_to_ned_frd`."""
    return quat_normalize(quat_multiply(quat_multiply(_Q_ENU_TO_NED, q), _Q_FLU_TO_FRD))
