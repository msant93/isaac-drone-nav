"""Physical constants shared by every module."""

from __future__ import annotations

import numpy as np

from dronenav.core.types import FloatArray

#: Standard gravity [m/s^2]. Same value as Pegasus Simulator.
GRAVITY: float = 9.81

#: Gravity acceleration vector in the ENU world frame [m/s^2].
GRAVITY_VECTOR_ENU: FloatArray = np.array([0.0, 0.0, -GRAVITY])

#: Unit vector along the world (ENU) / body (FLU) z axis.
E3: FloatArray = np.array([0.0, 0.0, 1.0])
