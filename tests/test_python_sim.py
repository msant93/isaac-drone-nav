"""Physics tests of the NumPy simulator against analytical solutions."""

from __future__ import annotations

import numpy as np
import pytest

from dronenav.config.schemas import GroundConfig, PythonSimConfig, VehicleConfig
from dronenav.core.math3d import euler_to_quat, quat_to_rotmat, yaw_from_quat
from dronenav.core.types import VehicleState, vec3
from dronenav.sim.base import SimBackend
from dronenav.sim.python_sim import PythonSimBackend

G = 9.81


def make_sim(vehicle: VehicleConfig, **overrides: object) -> PythonSimBackend:
    return PythonSimBackend(vehicle, PythonSimConfig.model_validate(overrides))


def airborne(z: float = 10.0, **kwargs: object) -> VehicleState:
    return VehicleState(position=vec3([0.0, 0.0, z]), **kwargs)  # type: ignore[arg-type]


def test_backend_implements_interface(sim: PythonSimBackend) -> None:
    assert isinstance(sim, SimBackend)
    assert sim.num_rotors == 4 and sim.dt == pytest.approx(0.004)


def test_hover_equilibrium(sim: PythonSimBackend) -> None:
    sim.reset(airborne(5.0))
    sim.run_for(5.0, sim.model.hover_rotor_speeds())
    s = sim.get_state()
    np.testing.assert_allclose(s.position, [0.0, 0.0, 5.0], atol=1e-9)
    np.testing.assert_allclose(s.velocity, 0.0, atol=1e-9)
    np.testing.assert_allclose(s.angular_velocity, 0.0, atol=1e-9)
    assert sim.time == pytest.approx(5.0)


@pytest.mark.parametrize("integrator", ["rk4", "euler"])
def test_free_fall(iris: VehicleConfig, integrator: str) -> None:
    sim = make_sim(iris, integrator=integrator, ground={"enabled": False})
    sim.reset(airborne(100.0))
    t = 2.0
    sim.run_for(t, np.zeros(4))
    s = sim.get_state()
    tol = 1e-9 if integrator == "rk4" else 0.05  # Euler is only first-order accurate
    assert s.position[2] == pytest.approx(100.0 - 0.5 * G * t**2, abs=tol)
    assert s.velocity[2] == pytest.approx(-G * t, abs=1e-9)


def test_horizontal_drag_exponential_decay(sim: PythonSimBackend) -> None:
    # Level hover with initial vx: thrust is vertical, so vx(t) = vx0 * exp(-d_x t / m).
    sim.reset(airborne(5.0, velocity=vec3([2.0, 0.0, 0.0])))
    t = 3.0
    sim.run_for(t, sim.model.hover_rotor_speeds())
    expected = 2.0 * np.exp(-0.5 * t / 1.52)
    assert sim.get_state().velocity[0] == pytest.approx(expected, rel=1e-6)


def test_yaw_torque_spin_up(iris: VehicleConfig) -> None:
    sim = make_sim(iris, ground={"enabled": False})
    sim.reset(airborne())
    tau_z = 0.01
    w = sim.model.rotor_speeds_from_wrench(sim.model.weight, [0.0, 0.0, tau_z])
    t = 1.0
    sim.run_for(t, w)
    alpha = tau_z / iris.inertia[2]
    s = sim.get_state()
    assert s.angular_velocity[2] == pytest.approx(alpha * t, rel=1e-6)
    assert yaw_from_quat(s.attitude) == pytest.approx(0.5 * alpha * t**2, rel=1e-6)
    np.testing.assert_allclose(s.position, [0.0, 0.0, 10.0], atol=1e-6)


def test_torque_free_rotation_conserves_energy_and_momentum(iris: VehicleConfig) -> None:
    sim = make_sim(iris, gravity=1.0, ground={"enabled": False})
    w0 = vec3([1.0, -0.7, 2.0])  # not about a principal axis: the rotation is not trivial
    sim.reset(airborne(angular_velocity=w0))
    j = np.diag(iris.inertia)

    def invariants() -> tuple[float, np.ndarray]:
        s = sim.get_state()
        w = s.angular_velocity
        momentum_world = quat_to_rotmat(s.attitude) @ (j @ w)
        return 0.5 * float(w @ j @ w), momentum_world

    e0, h0 = invariants()
    sim.run_for(10.0, np.zeros(4))  # gravity does not affect rotation
    e1, h1 = invariants()
    assert e1 == pytest.approx(e0, rel=1e-6)
    np.testing.assert_allclose(h1, h0, rtol=1e-6, atol=1e-9)
    assert np.linalg.norm(sim.get_state().attitude) == pytest.approx(1.0, abs=1e-12)


def test_tilted_thrust_accelerates_horizontally(iris: VehicleConfig) -> None:
    # Pitch nose-down by 10 deg (positive pitch in FLU): thrust tilts towards +x.
    sim = make_sim(iris, ground={"enabled": False})
    sim.reset(airborne(attitude=euler_to_quat(0.0, np.deg2rad(10.0), 0.0)))
    sim.step(sim.model.hover_rotor_speeds())
    assert sim.get_state().acceleration[0] > 0.0


def test_ground_contact_rest_and_crash(sim: PythonSimBackend) -> None:
    sim.reset()  # starts on the ground
    info = sim.run_for(1.0, np.zeros(4))
    s = sim.get_state()
    assert info.on_ground and not info.crashed
    assert s.position[2] == pytest.approx(0.0) and s.velocity[2] == pytest.approx(0.0)

    sim.reset(airborne(0.5))  # gentle drop: v_impact = sqrt(2 g h) ~ 3.1 m/s -> crash
    info = sim.run_for(1.0, np.zeros(4))
    assert info.crashed and sim.crashed

    sim.reset(airborne(0.1))  # ~1.4 m/s impact: below the 2 m/s crash threshold
    assert not sim.run_for(1.0, np.zeros(4)).crashed


def test_takeoff_from_ground(sim: PythonSimBackend) -> None:
    sim.reset()
    w = sim.model.rotor_speeds_from_wrench(1.2 * sim.model.weight, np.zeros(3))
    info = sim.run_for(1.0, w)
    s = sim.get_state()
    expected = 0.5 * 0.2 * G * 1.0**2
    assert not info.on_ground
    assert s.position[2] == pytest.approx(expected, rel=1e-3)


def test_imu_readings(sim: PythonSimBackend) -> None:
    sim.reset(airborne(5.0))
    sim.step(sim.model.hover_rotor_speeds())
    imu = sim.get_sensors().imu
    assert imu is not None
    np.testing.assert_allclose(imu.specific_force, [0.0, 0.0, G], atol=1e-9)  # hover reads +g

    sim.step(np.zeros(4))
    imu = sim.get_sensors().imu
    assert imu is not None
    np.testing.assert_allclose(imu.specific_force, 0.0, atol=1e-9)  # free fall reads 0


def test_motor_lag_first_order_response(iris: VehicleConfig) -> None:
    lagged = iris.model_copy(update={"motor_time_constant": 0.05})
    sim = PythonSimBackend(lagged, PythonSimConfig(ground=GroundConfig(enabled=False)))
    sim.reset(airborne())
    sim.run_for(0.048, np.full(4, 500.0))  # 12 steps of 4 ms, about one time constant
    np.testing.assert_allclose(sim.rotor_speeds, 500.0 * (1.0 - np.exp(-0.048 / 0.05)), rtol=1e-9)


def test_external_acceleration(iris: VehicleConfig) -> None:
    sim = make_sim(iris, ground={"enabled": False})
    sim.reset(airborne())
    sim.set_external_acceleration([0.0, 1.0, 0.0])
    sim.run_for(1.0, sim.model.hover_rotor_speeds())
    assert sim.get_state().position[1] > 0.3


def test_reset_and_determinism(sim: PythonSimBackend, rng: np.random.Generator) -> None:
    commands = rng.uniform(600.0, 700.0, size=(200, 4))
    results = []
    for _ in range(2):
        sim.reset(airborne(3.0))
        for cmd in commands:
            sim.step(cmd)
        results.append(sim.get_state())
    np.testing.assert_array_equal(results[0].position, results[1].position)
    np.testing.assert_array_equal(results[0].attitude, results[1].attitude)
    sim.reset()
    assert sim.time == 0.0 and not sim.crashed


def test_get_state_returns_copies(sim: PythonSimBackend) -> None:
    s = sim.get_state()
    s.position[:] = 99.0
    assert sim.get_state().position[0] == 0.0
