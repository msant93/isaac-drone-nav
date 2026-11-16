"""Pure-NumPy multirotor simulator implementing :class:`~dronenav.sim.base.SimBackend`.

Model:

* rigid-body Newton-Euler dynamics (:mod:`.dynamics`), RK4 or semi-implicit Euler;
* Pegasus-identical rotor model, allocation and linear drag
  (:class:`~dronenav.model.quadrotor.QuadrotorModel`);
* optional first-order motor lag (disabled by default, as in Pegasus);
* simplified flat ground: the vehicle cannot sink below the ground; while
  resting, horizontal motion and body rates are suppressed. A touchdown faster
  than ``crash_speed`` is flagged as a crash;
* ideal (noise-free) IMU; sensor noise models are added in Phase 4.

It is deliberately simple, deterministic and fast: thousands of times faster
than Isaac Sim, so it is used for unit tests, CI and controller tuning.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from dronenav.config.schemas import PythonSimConfig, VehicleConfig
from dronenav.core.math3d import euler_to_quat, quat_normalize, quat_to_rotmat
from dronenav.core.types import FloatArray, ImuMeasurement, SensorData, VehicleState, vec3
from dronenav.model.quadrotor import QuadrotorModel
from dronenav.sim.base import SimBackend, StepInfo
from dronenav.sim.python_sim.dynamics import (
    make_derivative,
    pack_state,
    rk4_step,
    semi_implicit_euler_step,
    unpack_state,
)

#: Below this vertical speed [m/s] a vehicle touching the ground is considered at rest.
_REST_SPEED = 1e-3


class PythonSimBackend(SimBackend):
    """Fast, deterministic multirotor simulator in pure NumPy."""

    def __init__(self, vehicle: VehicleConfig, config: PythonSimConfig | None = None):
        """Create the simulator and reset it to the configured initial state.

        Args:
            vehicle: Vehicle parameters.
            config: Simulator settings (defaults: 250 Hz, RK4, ground at z=0).
        """
        self.config = config if config is not None else PythonSimConfig()
        self.model = QuadrotorModel(vehicle, gravity=self.config.gravity)
        self._dt = self.config.physics_dt
        self._step_fn = rk4_step if self.config.integrator == "rk4" else semi_implicit_euler_step
        self._external_accel = vec3()
        self._time = 0.0
        self._x: FloatArray = np.zeros(13)
        self._rotor_speeds: FloatArray = np.zeros(self.model.num_rotors)
        self._accel: FloatArray = vec3()
        self._on_ground = False
        self._crashed = False
        self.reset()

    # -- SimBackend properties -------------------------------------------------
    @property
    def time(self) -> float:
        """Current simulation time [s]."""
        return self._time

    @property
    def dt(self) -> float:
        """Physics step [s]."""
        return self._dt

    @property
    def num_rotors(self) -> int:
        """Number of rotors."""
        return self.model.num_rotors

    @property
    def rotor_speeds(self) -> FloatArray:
        """Actual rotor speeds after saturation and motor lag [rad/s]."""
        return self._rotor_speeds.copy()

    @property
    def crashed(self) -> bool:
        """Whether a crash has been detected since the last reset."""
        return self._crashed

    # -- Configuration ---------------------------------------------------------------
    def default_initial_state(self) -> VehicleState:
        """Initial state described by the simulator configuration."""
        init = self.config.initial_state
        return VehicleState(
            position=vec3(init.position),
            velocity=vec3(init.velocity),
            attitude=euler_to_quat(0.0, 0.0, init.yaw),
            angular_velocity=vec3(init.angular_velocity),
        )

    def set_external_acceleration(self, accel_world: npt.ArrayLike) -> None:
        """Set a constant extra world-frame acceleration (e.g. a wind gust model) [m/s^2]."""
        self._external_accel = vec3(accel_world)

    # -- SimBackend methods ------------------------------------------------------------
    def reset(self, initial_state: VehicleState | None = None) -> None:
        """Reset time, state, rotor speeds and event flags."""
        state = initial_state if initial_state is not None else self.default_initial_state()
        self._x = pack_state(
            vec3(state.position),
            vec3(state.velocity),
            quat_normalize(state.attitude),
            vec3(state.angular_velocity),
        )
        self._time = float(state.timestamp)
        self._rotor_speeds = np.zeros(self.model.num_rotors)
        self._accel = vec3()
        self._crashed = False
        ground = self.config.ground
        self._on_ground = ground.enabled and self._x[2] <= ground.height + 1e-9
        self._external_accel = vec3()

    def step(self, rotor_speeds: npt.ArrayLike) -> StepInfo:
        """Apply rotor speed commands [rad/s] and integrate one physics step."""
        command = self.model.clip_rotor_speeds(rotor_speeds)
        self._update_motors(command)
        wrench = self.model.wrench_from_rotor_speeds(self._rotor_speeds)

        f = make_derivative(
            self.model.mass,
            self.model.inertia,
            self.model.inertia_inv,
            self.config.gravity,
            wrench.thrust,
            wrench.torque,
            self.model.drag,
            self._external_accel,
        )
        v_before = self._x[3:6].copy()
        x_next = self._step_fn(f, self._x, self._dt)
        x_next = self._apply_ground(x_next)
        self._accel = (x_next[3:6] - v_before) / self._dt
        self._x = x_next
        self._time += self._dt
        return StepInfo(on_ground=self._on_ground, crashed=self._crashed)

    def get_state(self) -> VehicleState:
        """Return a copy of the ground-truth state."""
        p, v, q, w = unpack_state(self._x)
        return VehicleState(
            position=p,
            velocity=v,
            attitude=q,
            angular_velocity=w,
            acceleration=self._accel.copy(),
            timestamp=self._time,
        )

    def get_sensors(self) -> SensorData:
        """Return an ideal IMU measurement (specific force and body rates)."""
        rot = quat_to_rotmat(self._x[6:10])
        gravity = np.array([0.0, 0.0, -self.config.gravity])
        imu = ImuMeasurement(
            specific_force=rot.T @ (self._accel - gravity),
            angular_velocity=self._x[10:13].copy(),
            timestamp=self._time,
        )
        return SensorData(imu=imu, timestamp=self._time)

    # -- Internals ------------------------------------------------------------------------
    def _update_motors(self, command: FloatArray) -> None:
        """First-order motor response, discretised exactly (instantaneous if tau = 0)."""
        tau = self.model.config.motor_time_constant
        if tau <= 0.0:
            self._rotor_speeds = command.copy()
        else:
            alpha = 1.0 - np.exp(-self._dt / tau)
            self._rotor_speeds = self._rotor_speeds + alpha * (command - self._rotor_speeds)

    def _apply_ground(self, x: FloatArray) -> FloatArray:
        """Enforce the flat-ground constraint and detect touchdowns / crashes."""
        ground = self.config.ground
        if not ground.enabled:
            self._on_ground = False
            return x
        if x[2] > ground.height:
            self._on_ground = False
            return x

        impact_speed = -float(x[5])
        if not self._on_ground and impact_speed > ground.crash_speed:
            self._crashed = True
        x = x.copy()
        x[2] = ground.height
        if x[5] < 0.0:
            x[5] = 0.0
        if abs(x[5]) < _REST_SPEED:
            # Resting on the ground: no sliding and no spinning (friction).
            x[3:5] = 0.0
            x[10:13] = 0.0
        self._on_ground = True
        return x
