"""Shared data types for the whole autonomy stack.

All vectors are NumPy ``float64`` arrays expressed with the project-wide
conventions (see :mod:`dronenav.core`):

* positions / velocities / accelerations: ENU world frame;
* angular velocities, body forces and torques: FLU body frame;
* attitude: unit quaternion ``[qx, qy, qz, qw]`` rotating body -> world.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

#: Alias used in type hints for float arrays of any shape.
FloatArray = npt.NDArray[np.float64]


def vec3(values: npt.ArrayLike | None = None) -> FloatArray:
    """Return a new float64 3-vector (zeros when ``values`` is ``None``).

    Args:
        values: Anything convertible to an array of 3 floats.

    Returns:
        A fresh (copied) array of shape ``(3,)``.

    Raises:
        ValueError: If ``values`` does not contain exactly 3 elements.
    """
    if values is None:
        return np.zeros(3)
    out = np.array(values, dtype=np.float64).reshape(-1)
    if out.shape != (3,):
        raise ValueError(f"expected 3 elements, got shape {out.shape}")
    return out


def identity_quat() -> FloatArray:
    """Return the identity quaternion ``[0, 0, 0, 1]``."""
    return np.array([0.0, 0.0, 0.0, 1.0])


@dataclass
class VehicleState:
    """Rigid-body state of the vehicle.

    Attributes:
        position: Position of the body origin, ENU world frame [m].
        velocity: Linear velocity, ENU world frame [m/s].
        attitude: Unit quaternion ``[qx, qy, qz, qw]``, body (FLU) -> world (ENU).
        angular_velocity: Angular velocity of the body, FLU body frame [rad/s].
        acceleration: Linear acceleration, ENU world frame [m/s^2].
        timestamp: Simulation time of the sample [s].
    """

    position: FloatArray = field(default_factory=vec3)
    velocity: FloatArray = field(default_factory=vec3)
    attitude: FloatArray = field(default_factory=identity_quat)
    angular_velocity: FloatArray = field(default_factory=vec3)
    acceleration: FloatArray = field(default_factory=vec3)
    timestamp: float = 0.0

    def copy(self) -> VehicleState:
        """Return a deep copy (arrays are copied, not shared)."""
        return VehicleState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            attitude=self.attitude.copy(),
            angular_velocity=self.angular_velocity.copy(),
            acceleration=self.acceleration.copy(),
            timestamp=self.timestamp,
        )


@dataclass
class ImuMeasurement:
    """Inertial measurement in the FLU body frame.

    Attributes:
        specific_force: Accelerometer reading ``R^T (a - g)`` [m/s^2]
            (reads ``+9.81`` on z when the vehicle rests level).
        angular_velocity: Gyroscope reading [rad/s].
        timestamp: Simulation time of the sample [s].
    """

    specific_force: FloatArray
    angular_velocity: FloatArray
    timestamp: float


@dataclass
class SensorData:
    """Bundle of the sensor measurements available at one simulation step.

    Fields are ``None`` when the backend does not provide that sensor (yet).
    GPS, barometer, magnetometer, LiDAR and depth camera fields are added in
    later phases.

    Attributes:
        imu: Latest IMU measurement.
        timestamp: Simulation time of the bundle [s].
    """

    imu: ImuMeasurement | None = None
    timestamp: float = 0.0
