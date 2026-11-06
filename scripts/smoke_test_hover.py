"""Phase 1 smoke test: Isaac Sim 5.1.0 + Pegasus Simulator v5.1.0.

The script checks that the whole simulation toolchain works on this machine
(local workstation, Docker container, AWS instance, ...):

1. Isaac Sim starts (headless, GUI or WebRTC livestream).
2. The Pegasus Simulator Python package can be imported.
3. The Pegasus "Iris" quadrotor spawns above a locally generated ground plane.
4. A custom Pegasus *Python backend* receives the vehicle state and commands
   rotor angular velocities: the drone takes off, hovers and lands.
5. Pass/fail criteria are evaluated and a JSON report plus a CSV trajectory
   are written to ``results/smoke_test/``.

The hover controller is a deliberately small geometric controller
(Lee et al., 2010) used only to validate the toolchain. The project's own
cascaded controller replaces it in Phase 3.

Usage (inside the container, or with a native Isaac Sim install)::

    /isaac-sim/python.sh scripts/smoke_test_hover.py              # headless
    /isaac-sim/python.sh scripts/smoke_test_hover.py --gui        # local GUI
    /isaac-sim/python.sh scripts/smoke_test_hover.py --livestream # WebRTC

Exit code: 0 if every acceptance criterion passes, 1 otherwise.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "smoke_test.yaml"


# --------------------------------------------------------------------------- #
# CLI and configuration (no Isaac Sim imports allowed before SimulationApp)
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="YAML configuration file.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true", help="Open the Isaac Sim GUI window.")
    mode.add_argument(
        "--livestream",
        action="store_true",
        help="Run headless and stream the viewport over WebRTC (remote machines).",
    )
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    """Load the smoke-test YAML configuration.

    Args:
        path: Path to the YAML file.

    Returns:
        The parsed configuration dictionary.
    """
    with path.open("r", encoding="utf-8") as stream:
        cfg: dict[str, Any] = yaml.safe_load(stream)
    return cfg


ARGS = parse_args()
CFG = load_config(ARGS.config)
HEADLESS = not ARGS.gui

# SimulationApp must be created before importing any other Omniverse module.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})

if ARGS.livestream:
    # WebRTC livestream: connect with the "Isaac Sim WebRTC Streaming Client".
    from isaacsim.core.utils.extensions import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.webrtc")

# --------------------------------------------------------------------------- #
# Imports that require a running SimulationApp
# --------------------------------------------------------------------------- #
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api.world import World  # noqa: E402
from pegasus.simulator.logic.backends import Backend  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.state import State  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

GRAVITY = 9.81  # [m/s^2]


# --------------------------------------------------------------------------- #
# Reference trajectory: takeoff ramp -> hover -> landing ramp
# --------------------------------------------------------------------------- #
@dataclass
class HoverProfile:
    """Piecewise-linear altitude profile for the smoke test.

    Attributes:
        start_xy: Horizontal hold position [m] (ENU).
        start_z: Initial altitude [m].
        takeoff_altitude: Hover altitude [m].
        climb_rate: Climb speed of the setpoint ramp [m/s].
        hover_duration: Duration of the hover phase [s].
        descent_rate: Descent speed of the setpoint ramp [m/s].
    """

    start_xy: np.ndarray
    start_z: float
    takeoff_altitude: float
    climb_rate: float
    hover_duration: float
    descent_rate: float

    @property
    def t_climb(self) -> float:
        """Duration of the climb ramp [s]."""
        return (self.takeoff_altitude - self.start_z) / self.climb_rate

    @property
    def t_hover_end(self) -> float:
        """Time at which the hover phase ends [s]."""
        return self.t_climb + self.hover_duration

    @property
    def t_land_end(self) -> float:
        """Time at which the landing ramp reaches the ground [s]."""
        return self.t_hover_end + self.takeoff_altitude / self.descent_rate

    def phase(self, t: float) -> str:
        """Return the mission phase name at time ``t``."""
        if t < self.t_climb:
            return "TAKEOFF"
        if t < self.t_hover_end:
            return "HOVER"
        if t < self.t_land_end:
            return "LAND"
        return "TOUCHDOWN"

    def reference(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Position and velocity reference at time ``t``.

        Args:
            t: Time since the start of the mission [s].

        Returns:
            Tuple ``(p_ref, v_ref)`` in the ENU world frame.
        """
        phase = self.phase(t)
        if phase == "TAKEOFF":
            z, vz = self.start_z + self.climb_rate * t, self.climb_rate
        elif phase == "HOVER":
            z, vz = self.takeoff_altitude, 0.0
        elif phase == "LAND":
            z = self.takeoff_altitude - self.descent_rate * (t - self.t_hover_end)
            vz = -self.descent_rate
        else:
            z, vz = 0.0, 0.0
        p_ref = np.array([self.start_xy[0], self.start_xy[1], z])
        v_ref = np.array([0.0, 0.0, vz])
        return p_ref, v_ref


# --------------------------------------------------------------------------- #
# Minimal geometric hover controller (pure function, easy to reason about)
# --------------------------------------------------------------------------- #
def vee(matrix: np.ndarray) -> np.ndarray:
    """Inverse of the hat map: skew-symmetric 3x3 matrix -> 3-vector."""
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def hover_control_law(
    p: np.ndarray,
    v: np.ndarray,
    rot: np.ndarray,
    omega: np.ndarray,
    p_ref: np.ndarray,
    v_ref: np.ndarray,
    yaw_ref: float,
    integral: np.ndarray,
    gains: dict[str, np.ndarray],
    mass: float,
) -> tuple[float, np.ndarray]:
    """Geometric position + attitude control law (Lee, Leok, McClamroch 2010).

    Args:
        p: Position, ENU world frame [m].
        v: Linear velocity, ENU world frame [m/s].
        rot: Rotation matrix body (FLU) -> world (ENU).
        omega: Angular velocity in the body frame [rad/s].
        p_ref: Position reference [m].
        v_ref: Velocity reference [m/s].
        yaw_ref: Yaw reference [rad].
        integral: Integral of the position error [m*s].
        gains: Diagonal gains ``kp, kd, ki, kr, kw`` as 3-vectors.
        mass: Vehicle mass [kg].

    Returns:
        Tuple ``(thrust, torque)``: collective thrust along body z [N] and body
        torque [N*m].
    """
    e_p = p - p_ref
    e_v = v - v_ref
    f_des = -gains["kp"] * e_p - gains["kd"] * e_v - gains["ki"] * integral + np.array([0.0, 0.0, mass * GRAVITY])
    z_body = rot[:, 2]
    thrust = float(max(f_des @ z_body, 0.0))

    # Desired attitude: body z along f_des, heading given by yaw_ref.
    z_des = f_des / max(float(np.linalg.norm(f_des)), 1e-6)
    x_heading = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])
    y_des = np.cross(z_des, x_heading)
    y_des /= max(float(np.linalg.norm(y_des)), 1e-6)
    x_des = np.cross(y_des, z_des)
    rot_des = np.column_stack((x_des, y_des, z_des))

    e_r = 0.5 * vee(rot_des.T @ rot - rot.T @ rot_des)
    torque = -gains["kr"] * e_r - gains["kw"] * omega
    return thrust, torque


# --------------------------------------------------------------------------- #
# Pegasus Python backend
# --------------------------------------------------------------------------- #
@dataclass
class LogSample:
    """One row of the trajectory log."""

    t: float
    phase: str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    x_ref: float
    y_ref: float
    z_ref: float


@dataclass
class SmokeResult:
    """Outcome of the smoke test."""

    passed: bool = False
    sim_time: float = 0.0
    wall_time: float = 0.0
    max_altitude: float = 0.0
    hover_alt_rmse: float = float("nan")
    hover_alt_max_error: float = float("nan")
    hover_xy_max_drift: float = float("nan")
    final_altitude: float = float("nan")
    landed: bool = False
    checks: dict[str, bool] = field(default_factory=dict)


class SmokeHoverBackend(Backend):  # type: ignore[misc]
    """Pegasus backend running the takeoff -> hover -> land profile.

    Pegasus calls, at every physics step: ``update_state`` (ground-truth
    state), ``update_sensor`` (IMU/GPS/baro/mag, ignored here),
    ``input_reference`` (rotor speeds to apply) and ``update`` (control step).
    """

    def __init__(self, profile: HoverProfile, gains: dict[str, np.ndarray], mass: float, i_limit: float):
        """Create the backend.

        Args:
            profile: Reference altitude profile.
            gains: Controller gains (see :func:`hover_control_law`).
            mass: Vehicle mass [kg].
            i_limit: Anti-windup clamp for the position integral [m*s].
        """
        super().__init__(config=None)
        self.profile = profile
        self.gains = gains
        self.mass = mass
        self.i_limit = i_limit
        self.input_ref: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.log: list[LogSample] = []
        self.done = False
        self._reset_internal()

    def _reset_internal(self) -> None:
        """Reset controller memory and the mission clock."""
        self.t = 0.0
        self.integral = np.zeros(3)
        self.has_state = False
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.rot = np.eye(3)
        self.omega = np.zeros(3)
        self.input_ref = [0.0, 0.0, 0.0, 0.0]
        self.done = False

    # -- Pegasus callbacks ------------------------------------------------- #
    def update_state(self, state: State) -> None:
        """Store the latest ground-truth state (ENU / FLU)."""
        self.p = np.asarray(state.position, dtype=float)
        self.v = np.asarray(state.linear_velocity, dtype=float)
        self.rot = Rotation.from_quat(state.attitude).as_matrix()  # [qx, qy, qz, qw]
        self.omega = np.asarray(state.angular_velocity, dtype=float)
        self.has_state = True

    def update_sensor(self, sensor_type: str, data: Any) -> None:
        """Ignore simulated sensors (state estimation arrives in Phase 4)."""

    def update_graphical_sensor(self, sensor_type: str, data: Any) -> None:
        """No cameras or LiDARs are used in the smoke test."""

    def input_reference(self) -> list[float]:
        """Return the rotor angular velocity references [rad/s]."""
        return self.input_ref

    def update(self, dt: float) -> None:
        """Run one control step of ``dt`` seconds."""
        if not self.has_state or self.done:
            return
        self.t += dt
        phase = self.profile.phase(self.t)
        p_ref, v_ref = self.profile.reference(self.t)

        self.integral = np.clip(self.integral + (self.p - p_ref) * dt, -self.i_limit, self.i_limit)
        thrust, torque = hover_control_law(
            self.p, self.v, self.rot, self.omega, p_ref, v_ref, 0.0, self.integral, self.gains, self.mass
        )
        if self.vehicle is not None:
            self.input_ref = list(self.vehicle.force_and_torques_to_velocities(thrust, torque))

        # The reference has reached the ground: cut the motors once low enough
        # (or after a timeout, so the test always terminates).
        if phase == "TOUCHDOWN" and (self.p[2] < 0.25 or self.t > self.profile.t_land_end + 3.0):
            self.input_ref = [0.0, 0.0, 0.0, 0.0]
            self.done = True

        self.log.append(LogSample(self.t, phase, *self.p.tolist(), *self.v.tolist(), *p_ref.tolist()))

    def start(self) -> None:
        """Reset the backend when Pegasus starts the simulation."""
        self._reset_internal()

    def stop(self) -> None:
        """Handle the simulation stop event (nothing to release)."""

    def reset(self) -> None:
        """Reset the backend when Pegasus resets the simulation."""
        self._reset_internal()


# --------------------------------------------------------------------------- #
# Evaluation and reporting
# --------------------------------------------------------------------------- #
def evaluate(log: list[LogSample], profile: HoverProfile, cfg: dict[str, Any], landed: bool) -> SmokeResult:
    """Compute the smoke-test metrics and pass/fail checks."""
    acc = cfg["acceptance"]
    result = SmokeResult(landed=landed)
    if not log:
        result.checks = {"received_state": False}
        return result

    t = np.array([s.t for s in log])
    z = np.array([s.z for s in log])
    xy = np.array([[s.x, s.y] for s in log])
    hover = (t >= profile.t_climb + acc["hover_settle_time"]) & (t < profile.t_hover_end)

    result.sim_time = float(t[-1])
    result.max_altitude = float(z.max())
    result.final_altitude = float(z[-1])
    if hover.any():
        err = z[hover] - profile.takeoff_altitude
        result.hover_alt_rmse = float(np.sqrt(np.mean(err**2)))
        result.hover_alt_max_error = float(np.abs(err).max())
        result.hover_xy_max_drift = float(np.linalg.norm(xy[hover] - profile.start_xy, axis=1).max())

    result.checks = {
        "received_state": True,
        "reached_hover_phase": bool(hover.any()),
        "hover_altitude_ok": bool(result.hover_alt_max_error <= acc["max_hover_alt_error"]),
        "hover_xy_drift_ok": bool(result.hover_xy_max_drift <= acc["max_hover_xy_drift"]),
        "landed": bool(landed and result.final_altitude <= acc["max_landing_alt"]),
    }
    result.passed = all(result.checks.values())
    return result


def write_outputs(log: list[LogSample], result: SmokeResult, out_dir: Path) -> None:
    """Write the CSV trajectory and the JSON report."""
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(LogSample.__dataclass_fields__))
        writer.writeheader()
        writer.writerows(asdict(s) for s in log)
    with (out_dir / "report.json").open("w", encoding="utf-8") as stream:
        json.dump(asdict(result), stream, indent=2)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def build_world(cfg: dict[str, Any]) -> tuple[PegasusInterface, World]:
    """Create the Pegasus interface, the physics world and a ground plane.

    The ground plane is generated locally (no Nucleus/S3 asset download), so
    the test also works on machines without access to NVIDIA's asset server.
    """
    pg = PegasusInterface()
    pg.set_world_settings(physics_dt=cfg["simulation"]["physics_dt"], rendering_dt=cfg["simulation"]["rendering_dt"])
    pg._world = World(**pg._world_settings)  # same pattern as the Pegasus examples
    world = pg.world
    try:
        from isaacsim.core.api.objects import GroundPlane

        GroundPlane(prim_path="/World/GroundPlane", z_position=0.0, size=50.0)
    except ImportError:  # pragma: no cover - API fallback
        carb.log_warn("GroundPlane class not found, falling back to the default ground plane asset.")
        world.scene.add_default_ground_plane()
    return pg, world


def main() -> int:
    """Run the smoke test and return the process exit code."""
    cfg = CFG
    veh, mis, ctl = cfg["vehicle"], cfg["mission"], cfg["controller"]
    spawn = np.array(veh["spawn_position"], dtype=float)
    profile = HoverProfile(
        start_xy=spawn[:2].copy(),
        start_z=float(spawn[2]),
        takeoff_altitude=float(mis["takeoff_altitude"]),
        climb_rate=float(mis["climb_rate"]),
        hover_duration=float(mis["hover_duration"]),
        descent_rate=float(mis["descent_rate"]),
    )
    gains = {k: np.array(ctl[k], dtype=float) for k in ("kp", "kd", "ki", "kr", "kw")}
    backend = SmokeHoverBackend(profile, gains, float(veh["mass"]), float(ctl["integral_limit"]))

    timeline = omni.timeline.get_timeline_interface()
    _, world = build_world(cfg)

    multirotor_cfg = MultirotorConfig()
    multirotor_cfg.backends = [backend]  # replaces the default PX4 MAVLink backend
    orientation = Rotation.from_euler("XYZ", [0.0, 0.0, veh["spawn_yaw_deg"]], degrees=True).as_quat()
    Multirotor(veh["prim_path"], ROBOTS["Iris"], 0, spawn.tolist(), orientation, config=multirotor_cfg)

    world.reset()
    timeline.play()
    render = not HEADLESS or ARGS.livestream
    max_steps = int(cfg["simulation"]["max_sim_time"] / cfg["simulation"]["physics_dt"])
    wall_start = time.perf_counter()
    carb.log_warn("[smoke_test] Simulation started.")

    for _ in range(max_steps):
        if not simulation_app.is_running() or backend.done:
            break
        world.step(render=render)

    timeline.stop()
    result = evaluate(backend.log, profile, cfg, landed=backend.done)
    result.wall_time = time.perf_counter() - wall_start
    out_dir = REPO_ROOT / cfg["output"]["results_dir"]
    write_outputs(backend.log, result, out_dir)

    print("\n================ SMOKE TEST ================")
    print(json.dumps(asdict(result), indent=2))
    print(f"Outputs written to: {out_dir}")
    print("RESULT:", "PASS" if result.passed else "FAIL")
    print("============================================\n")
    return 0 if result.passed else 1


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    finally:
        simulation_app.close()
    sys.exit(exit_code)
