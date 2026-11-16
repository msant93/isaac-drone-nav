"""Typed, validated configuration schemas (pydantic v2).

Every YAML file is parsed into one of these models, so that a typo or an
inconsistent value fails immediately, with a clear message, instead of
producing a confusing behaviour in the middle of a simulation.

Units are SI throughout (m, s, kg, rad, N). Frames follow
:mod:`dronenav.core` (ENU world, FLU body).
"""

from __future__ import annotations

from typing import Annotated

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from dronenav.core.constants import GRAVITY
from dronenav.core.types import FloatArray

PositiveFloat = Annotated[float, Field(gt=0.0)]
NonNegativeFloat = Annotated[float, Field(ge=0.0)]
Vector3 = Annotated[list[float], Field(min_length=3, max_length=3)]


class StrictModel(BaseModel):
    """Base model: unknown keys are errors and instances are immutable."""

    model_config = ConfigDict(extra="forbid", frozen=True)


# --------------------------------------------------------------------------- #
# Vehicle
# --------------------------------------------------------------------------- #
class RotorConfig(StrictModel):
    """One rotor of the multirotor.

    Attributes:
        position: Rotor hub position in the body frame (FLU) relative to the
            centre of mass [m].
        direction: Spin direction used for the reaction torque about body z:
            ``+1`` or ``-1`` (Pegasus ``rot_dir`` convention).
    """

    position: Vector3
    direction: Annotated[int, Field(ge=-1, le=1)]

    @model_validator(mode="after")
    def _direction_is_unit(self) -> RotorConfig:
        if self.direction == 0:
            raise ValueError("rotor direction must be +1 or -1")
        return self


class VehicleConfig(StrictModel):
    """Physical parameters of a multirotor (Pegasus-compatible rotor model).

    Rotor model (quadratic thrust curve, identical to Pegasus'
    ``QuadraticThrustCurve``): ``thrust_i = rotor_constant * w_i^2`` along body
    z, reaction torque ``tau_z += moment_constant * w_i^2 * direction_i``.

    Attributes:
        name: Human-readable name.
        mass: Total mass [kg].
        inertia: Diagonal of the inertia tensor about the CoM, body frame [kg m^2].
        rotors: Rotor geometry and spin directions (ordered as in the simulator).
        rotor_constant: Thrust coefficient ``k_f`` [N s^2 / rad^2].
        moment_constant: Reaction-torque coefficient ``k_m`` [N m s^2 / rad^2].
        min_rotor_velocity: Lower rotor speed limit [rad/s].
        max_rotor_velocity: Upper rotor speed limit [rad/s].
        motor_time_constant: First-order motor lag [s]; ``0`` = instantaneous
            (Pegasus behaviour).
        drag_coefficients: Linear drag ``F_body = -diag(d) @ v_body`` [N s/m].
        min_thrust_to_weight: Validation threshold on the maximum thrust-to-weight ratio.
    """

    name: str = "quadrotor"
    mass: PositiveFloat
    inertia: Vector3
    rotors: Annotated[list[RotorConfig], Field(min_length=3)]
    rotor_constant: PositiveFloat
    moment_constant: NonNegativeFloat
    min_rotor_velocity: NonNegativeFloat = 0.0
    max_rotor_velocity: PositiveFloat
    motor_time_constant: NonNegativeFloat = 0.0
    drag_coefficients: Vector3 = [0.0, 0.0, 0.0]
    min_thrust_to_weight: PositiveFloat = 1.5

    @model_validator(mode="after")
    def _check_physics(self) -> VehicleConfig:
        jx, jy, jz = self.inertia
        if min(self.inertia) <= 0.0:
            raise ValueError("inertia diagonal entries must be > 0")
        # Triangle inequality for principal moments of a physical rigid body.
        if jx + jy < jz or jy + jz < jx or jx + jz < jy:
            raise ValueError("inertia violates the triangle inequality (not a physical rigid body)")
        if self.min_rotor_velocity >= self.max_rotor_velocity:
            raise ValueError("min_rotor_velocity must be < max_rotor_velocity")
        if any(d < 0.0 for d in self.drag_coefficients):
            raise ValueError("drag coefficients must be >= 0")
        ratio = self.max_thrust / (self.mass * GRAVITY)
        if ratio < self.min_thrust_to_weight:
            raise ValueError(
                f"maximum thrust-to-weight ratio is {ratio:.2f} < {self.min_thrust_to_weight}: "
                "the vehicle cannot hover with a safe margin"
            )
        return self

    # -- Convenience accessors (NumPy) ---------------------------------------
    @property
    def num_rotors(self) -> int:
        """Number of rotors."""
        return len(self.rotors)

    @property
    def max_thrust(self) -> float:
        """Total thrust with every rotor at maximum speed [N]."""
        return self.num_rotors * self.rotor_constant * self.max_rotor_velocity**2

    @property
    def inertia_matrix(self) -> FloatArray:
        """Inertia tensor as a 3x3 diagonal matrix [kg m^2]."""
        return np.diag(np.asarray(self.inertia, dtype=np.float64))

    @property
    def rotor_positions(self) -> FloatArray:
        """Rotor positions as an ``(N, 3)`` array [m]."""
        return np.array([r.position for r in self.rotors], dtype=np.float64)

    @property
    def rotor_directions(self) -> FloatArray:
        """Rotor spin directions as an ``(N,)`` array of +-1."""
        return np.array([r.direction for r in self.rotors], dtype=np.float64)


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #
class GroundConfig(StrictModel):
    """Simplified ground model of the NumPy simulator.

    Attributes:
        enabled: Whether a flat ground plane exists.
        height: Ground height in the world frame [m].
        crash_speed: Vertical impact speed above which a touchdown counts as a crash [m/s].
    """

    enabled: bool = True
    height: float = 0.0
    crash_speed: PositiveFloat = 2.0


class InitialStateConfig(StrictModel):
    """Initial state of the vehicle.

    Attributes:
        position: Initial position, ENU [m].
        velocity: Initial velocity, ENU [m/s].
        yaw: Initial heading [rad] (roll and pitch start at zero).
        angular_velocity: Initial body rates, FLU [rad/s].
    """

    position: Vector3 = [0.0, 0.0, 0.0]
    velocity: Vector3 = [0.0, 0.0, 0.0]
    yaw: float = 0.0
    angular_velocity: Vector3 = [0.0, 0.0, 0.0]


class PythonSimConfig(StrictModel):
    """Configuration of the pure-NumPy simulator backend.

    Attributes:
        physics_dt: Integration step [s] (250 Hz matches Pegasus).
        gravity: Gravity magnitude [m/s^2].
        integrator: ``"rk4"`` (default, accurate) or ``"euler"`` (semi-implicit, like PhysX).
        ground: Ground model.
        initial_state: Initial vehicle state.
    """

    physics_dt: Annotated[float, Field(gt=0.0, le=0.05)] = 0.004
    gravity: PositiveFloat = GRAVITY
    integrator: str = "rk4"
    ground: GroundConfig = GroundConfig()
    initial_state: InitialStateConfig = InitialStateConfig()

    @model_validator(mode="after")
    def _check_integrator(self) -> PythonSimConfig:
        if self.integrator not in ("rk4", "euler"):
            raise ValueError("integrator must be 'rk4' or 'euler'")
        if self.ground.enabled and self.initial_state.position[2] < self.ground.height:
            raise ValueError("initial position is below the ground")
        return self
