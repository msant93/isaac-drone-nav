"""dronenav: autonomous quadrotor navigation stack for NVIDIA Isaac Sim.

The package is organised so that every algorithm (estimation, control,
planning, avoidance, mission logic) is simulator-agnostic and can be unit
tested in plain Python. Only :mod:`dronenav.sim.isaac_pegasus` depends on
Isaac Sim and Pegasus Simulator.
"""

from __future__ import annotations

__all__ = ["SUBPACKAGES", "__version__"]

__version__ = "0.1.0"

#: Simulator-agnostic subpackages (importable without Isaac Sim).
SUBPACKAGES: tuple[str, ...] = (
    "core",
    "config",
    "sim",
    "sim.python_sim",
    "scene",
    "estimation",
    "control",
    "perception",
    "planning",
    "avoidance",
    "mission",
    "telemetry",
    "apps",
)
