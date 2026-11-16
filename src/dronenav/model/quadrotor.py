"""Multirotor actuation model: rotor thrust curve, control allocation and drag.

This module reproduces the Pegasus Simulator v5.1.0 actuation model exactly,
so that the NumPy simulator and the Isaac Sim backend behave the same way:

* each rotor produces a thrust ``f_i = k_f * w_i^2`` along body ``+z``,
  applied at its hub position ``r_i`` (FLU, relative to the CoM);
* each rotor adds a reaction torque ``k_m * w_i^2 * dir_i`` about body ``z``;
* rotor speeds are clipped to ``[w_min, w_max]`` (no motor lag in Pegasus);
* a linear drag ``F_body = -diag(d) @ v_body`` acts on the body.

Allocation matrix (maps squared rotor speeds to collective thrust and body
torques)::

    [ T  ]   [  k_f        ...  k_f        ]   [ w_1^2 ]
    [ tx ] = [  k_f*y_1    ...  k_f*y_N    ] @ [  ...  ]
    [ ty ]   [ -k_f*x_1    ... -k_f*x_N    ]   [ w_N^2 ]
    [ tz ]   [  k_m*dir_1  ...  k_m*dir_N  ]

The rows for ``tx`` and ``ty`` come from ``r_i x (0, 0, f_i)``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from dronenav.config.schemas import VehicleConfig
from dronenav.core.constants import GRAVITY
from dronenav.core.types import FloatArray


@dataclass(frozen=True)
class Wrench:
    """Collective thrust and body torque produced by the rotors.

    Attributes:
        thrust: Total thrust along body +z [N].
        torque: Body torque, FLU [N m].
    """

    thrust: float
    torque: FloatArray


class QuadrotorModel:
    """Actuation model of a multirotor built from a :class:`VehicleConfig`.

    Despite the name it supports any number of rotors >= 3 with parallel
    thrust axes (quadrotors, hexarotors, ...).
    """

    def __init__(self, config: VehicleConfig, gravity: float = GRAVITY):
        """Create the model.

        Args:
            config: Validated vehicle parameters.
            gravity: Gravity magnitude [m/s^2].
        """
        self.config = config
        self.gravity = gravity
        self.mass = config.mass
        self.inertia: FloatArray = config.inertia_matrix
        self.inertia_inv: FloatArray = np.linalg.inv(self.inertia).astype(np.float64)
        self.num_rotors = config.num_rotors
        self.k_f = config.rotor_constant
        self.k_m = config.moment_constant
        self.w_min = config.min_rotor_velocity
        self.w_max = config.max_rotor_velocity
        self.drag: FloatArray = np.asarray(config.drag_coefficients, dtype=np.float64)

        pos = config.rotor_positions
        dirs = config.rotor_directions
        self.allocation: FloatArray = np.vstack(
            [
                np.full(self.num_rotors, self.k_f),
                self.k_f * pos[:, 1],
                -self.k_f * pos[:, 0],
                self.k_m * dirs,
            ]
        )
        self.allocation_pinv: FloatArray = np.linalg.pinv(self.allocation).astype(np.float64)
        if np.linalg.matrix_rank(self.allocation) < 4:
            raise ValueError("rotor geometry cannot generate independent thrust, roll, pitch and yaw")

    # -- Forward model ---------------------------------------------------------
    def clip_rotor_speeds(self, rotor_speeds: npt.ArrayLike) -> FloatArray:
        """Clip rotor speeds to the admissible range (Pegasus behaviour)."""
        speeds = np.asarray(rotor_speeds, dtype=np.float64).reshape(-1)
        if speeds.shape != (self.num_rotors,):
            raise ValueError(f"expected {self.num_rotors} rotor speeds, got shape {speeds.shape}")
        return np.clip(speeds, self.w_min, self.w_max)

    def rotor_thrusts(self, rotor_speeds: npt.ArrayLike) -> FloatArray:
        """Thrust of each rotor [N] for the given (clipped) speeds [rad/s]."""
        w = self.clip_rotor_speeds(rotor_speeds)
        out: FloatArray = self.k_f * w**2
        return out

    def wrench_from_rotor_speeds(self, rotor_speeds: npt.ArrayLike) -> Wrench:
        """Collective thrust and body torque for the given rotor speeds [rad/s]."""
        w = self.clip_rotor_speeds(rotor_speeds)
        u = self.allocation @ (w**2)
        return Wrench(thrust=float(u[0]), torque=u[1:4].copy())

    def drag_force_body(self, velocity_body: npt.ArrayLike) -> FloatArray:
        """Linear drag force in the body frame [N] for a body-frame velocity [m/s]."""
        out: FloatArray = -self.drag * np.asarray(velocity_body, dtype=np.float64)
        return out

    # -- Inverse model (mixer) ---------------------------------------------------
    def rotor_speeds_from_wrench(self, thrust: float, torque: npt.ArrayLike) -> FloatArray:
        """Rotor speeds [rad/s] that realise a desired thrust and body torque.

        Identical to Pegasus' ``Multirotor.force_and_torques_to_velocities``:
        least-squares inversion of the allocation matrix, negative squared
        speeds set to zero, then a uniform scaling of all squared speeds if the
        largest exceeds ``w_max^2`` (this preserves the torque/thrust ratios).

        Args:
            thrust: Desired collective thrust [N].
            torque: Desired body torque, FLU [N m].

        Returns:
            Rotor angular velocities [rad/s], one per rotor.
        """
        tau = np.asarray(torque, dtype=np.float64).reshape(3)
        w_sq = self.allocation_pinv @ np.concatenate(([thrust], tau))
        w_sq = np.maximum(w_sq, 0.0)
        max_sq = self.w_max**2
        peak = float(np.max(w_sq))
        if peak >= max_sq:
            w_sq = w_sq / (peak / max_sq)
        out: FloatArray = np.sqrt(w_sq)
        return out

    # -- Useful derived quantities ------------------------------------------------
    @property
    def weight(self) -> float:
        """Vehicle weight [N]."""
        return self.mass * self.gravity

    @property
    def max_thrust(self) -> float:
        """Maximum collective thrust [N]."""
        return self.num_rotors * self.k_f * self.w_max**2

    @property
    def thrust_to_weight(self) -> float:
        """Maximum thrust-to-weight ratio."""
        return self.max_thrust / self.weight

    def hover_rotor_speeds(self) -> FloatArray:
        """Rotor speeds [rad/s] producing thrust = weight and zero torque."""
        return self.rotor_speeds_from_wrench(self.weight, np.zeros(3))
