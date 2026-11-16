"""Phase 2 demo: validate the NumPy simulator against analytical solutions.

Runs four open-loop experiments with the Iris parameters (no controller yet):

1. **Hover**: rotors at the hover speed; the altitude must stay constant.
2. **Free fall + touchdown**: motors off from 3 m; compared with ``z0 - g t^2 / 2``.
3. **Yaw torque step**: constant yaw torque; the yaw rate must grow as ``tau / Izz * t``.
4. **Horizontal drag**: level hover with initial ``vx``; it must decay as ``vx0 exp(-d t / m)``.

It prints the maximum error of each experiment and the real-time factor
(simulated seconds per wall-clock second), and saves a figure to
``results/phase2/python_sim_validation.png``.

Usage::

    python scripts/python_sim_demo.py
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # file output only, works on headless machines
import matplotlib.pyplot as plt
import numpy as np

from dronenav.config.loader import REPO_ROOT, load_config
from dronenav.config.schemas import PythonSimConfig, VehicleConfig
from dronenav.core.types import FloatArray, VehicleState, vec3
from dronenav.sim.python_sim import PythonSimBackend

OUT_DIR = REPO_ROOT / "results" / "phase2"

# Chart styling: validated two-series palette (simulated vs analytical).
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
SIMULATED = "#2a78d6"
ANALYTICAL = "#eb6834"


@dataclass
class Experiment:
    """Result of one experiment.

    Attributes:
        title: Panel title.
        ylabel: Y-axis label with units.
        t: Time samples [s].
        simulated: Simulated quantity.
        analytical: Analytical reference.
    """

    title: str
    ylabel: str
    t: FloatArray
    simulated: FloatArray
    analytical: FloatArray

    @property
    def max_error(self) -> float:
        """Maximum absolute error between simulation and reference."""
        return float(np.max(np.abs(self.simulated - self.analytical)))


def run(
    sim: PythonSimBackend,
    initial: VehicleState,
    command: FloatArray,
    duration: float,
    extract: Callable[[VehicleState], float],
) -> tuple[FloatArray, FloatArray]:
    """Run a constant-command experiment and sample one scalar per step."""
    sim.reset(initial)
    t, y = [0.0], [extract(sim.get_state())]
    for _ in range(round(duration / sim.dt)):
        sim.step(command)
        state = sim.get_state()
        t.append(state.timestamp)
        y.append(extract(state))
    return np.array(t), np.array(y)


def experiments(vehicle: VehicleConfig, config: PythonSimConfig) -> list[Experiment]:
    """Run the four validation experiments."""
    sim = PythonSimBackend(vehicle, config)
    g, m = config.gravity, vehicle.mass
    hover = sim.model.hover_rotor_speeds()
    results = []

    t, z = run(sim, VehicleState(position=vec3([0, 0, 2.0])), hover, 5.0, lambda s: float(s.position[2]))
    results.append(Experiment("Hover at 2 m", "altitude [m]", t, z, np.full_like(t, 2.0)))

    z0 = 3.0
    t, z = run(sim, VehicleState(position=vec3([0, 0, z0])), np.zeros(4), 1.2, lambda s: float(s.position[2]))
    results.append(Experiment("Free fall and touchdown", "altitude [m]", t, z, np.maximum(z0 - 0.5 * g * t**2, 0.0)))

    tau_z = 0.01
    w = sim.model.rotor_speeds_from_wrench(sim.model.weight, [0.0, 0.0, tau_z])
    t, r = run(sim, VehicleState(position=vec3([0, 0, 5.0])), w, 2.0, lambda s: float(s.angular_velocity[2]))
    results.append(Experiment("Yaw torque step", "yaw rate [rad/s]", t, r, tau_z / vehicle.inertia[2] * t))

    vx0, d = 2.0, vehicle.drag_coefficients[0]
    start = VehicleState(position=vec3([0, 0, 5.0]), velocity=vec3([vx0, 0, 0]))
    t, vx = run(sim, start, hover, 5.0, lambda s: float(s.velocity[0]))
    results.append(Experiment("Horizontal drag decay", "velocity x [m/s]", t, vx, vx0 * np.exp(-d * t / m)))
    return results


def real_time_factor(vehicle: VehicleConfig, config: PythonSimConfig, sim_seconds: float = 20.0) -> float:
    """Simulated seconds per wall-clock second for a hovering vehicle."""
    sim = PythonSimBackend(vehicle, config)
    sim.reset(VehicleState(position=vec3([0, 0, 2.0])))
    hover = sim.model.hover_rotor_speeds()
    start = time.perf_counter()
    sim.run_for(sim_seconds, hover)
    return sim_seconds / (time.perf_counter() - start)


def plot(results: list[Experiment], path: Path) -> None:
    """Save a 2x2 small-multiples figure (simulated vs analytical)."""
    plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "font.family": "sans-serif"})
    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5), facecolor=SURFACE)
    for ax, res in zip(axes.flat, results, strict=True):
        ax.set_facecolor(SURFACE)
        # Simulated first (wide, translucent), analytical dashed on top so both stay visible.
        ax.plot(res.t, res.simulated, color=SIMULATED, lw=4.0, alpha=0.55, label="Simulated")
        ax.plot(res.t, res.analytical, color=ANALYTICAL, lw=2.0, ls=(0, (4, 3)), label="Analytical")
        ax.set_title(res.title, color=TEXT_PRIMARY, loc="left", fontweight="bold")
        ax.set_title(f"max |error| {res.max_error:.1e}", color=TEXT_SECONDARY, loc="right", fontsize=9)
        ax.set_xlabel("time [s]", color=TEXT_SECONDARY)
        ax.set_ylabel(res.ylabel, color=TEXT_SECONDARY)
        ax.grid(color=GRID, lw=0.8)
        ax.tick_params(colors=TEXT_SECONDARY)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", frameon=False, ncol=2, labelcolor=TEXT_PRIMARY)
    fig.suptitle(
        "NumPy simulator vs analytical solutions (Iris, open loop)",
        x=0.01,
        ha="left",
        color=TEXT_PRIMARY,
        fontweight="bold",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


def main() -> None:
    """Run the experiments, print a summary and save the figure."""
    vehicle = load_config("drone_iris.yaml", VehicleConfig, section="vehicle")
    config = load_config("python_sim.yaml", PythonSimConfig, section="simulation")
    results = experiments(vehicle, config)
    print(f"{'experiment':<28}{'max abs error':>15}")
    for res in results:
        print(f"{res.title:<28}{res.max_error:>15.2e}")
    rtf = real_time_factor(vehicle, config)
    print(f"\nReal-time factor: {rtf:.0f}x (simulated seconds per wall-clock second, 250 Hz, RK4)")
    figure = OUT_DIR / "python_sim_validation.png"
    plot(results, figure)
    print(f"Figure saved to {figure}")


if __name__ == "__main__":
    main()
