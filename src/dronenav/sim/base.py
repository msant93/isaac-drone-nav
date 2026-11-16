"""Abstract simulator interface used by the whole autonomy stack.

The autonomy code only talks to :class:`SimBackend`; it never knows whether it
is driving the NumPy simulator or Isaac Sim. The interface is *step-based*
(the caller owns the loop):

.. code-block:: python

    sim.reset()
    while running:
        state = sim.get_state()            # ground truth (debug / GT mode)
        sensors = sim.get_sensors()        # what the estimator may use
        cmd = controller.update(...)       # rotor speeds [rad/s]
        sim.step(cmd)

For Isaac Sim + Pegasus (Phase 3) the adapter stores the command in a
Pegasus ``Backend`` shim and calls ``world.step()``, so the same loop works
even though Pegasus internally uses callbacks.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy.typing as npt

from dronenav.core.types import SensorData, VehicleState


@dataclass(frozen=True)
class StepInfo:
    """Events reported by the simulator after a step.

    Attributes:
        on_ground: Vehicle is resting on / touching the ground.
        crashed: A crash was detected (e.g. impact faster than the crash speed).
    """

    on_ground: bool = False
    crashed: bool = False


class SimBackend(ABC):
    """Common interface of every simulator backend."""

    @property
    @abstractmethod
    def time(self) -> float:
        """Current simulation time [s]."""

    @property
    @abstractmethod
    def dt(self) -> float:
        """Physics step [s]."""

    @property
    @abstractmethod
    def num_rotors(self) -> int:
        """Number of rotors expected by :meth:`step`."""

    @abstractmethod
    def reset(self, initial_state: VehicleState | None = None) -> None:
        """Reset time and vehicle state (to ``initial_state`` or the configured default)."""

    @abstractmethod
    def step(self, rotor_speeds: npt.ArrayLike) -> StepInfo:
        """Apply rotor speed commands [rad/s] and advance the simulation by :attr:`dt`."""

    @abstractmethod
    def get_state(self) -> VehicleState:
        """Return a copy of the ground-truth vehicle state."""

    @abstractmethod
    def get_sensors(self) -> SensorData:
        """Return the latest sensor measurements."""

    def close(self) -> None:  # noqa: B027 - optional hook, intentionally empty
        """Release resources (no-op by default)."""

    def run_for(self, duration: float, rotor_speeds: npt.ArrayLike) -> StepInfo:
        """Apply a constant command for ``duration`` seconds (open loop).

        Args:
            duration: Duration [s]; rounded to an integer number of steps.
            rotor_speeds: Constant rotor speeds [rad/s].

        Returns:
            The :class:`StepInfo` of the last step.
        """
        info = StepInfo()
        for _ in range(round(duration / self.dt)):
            info = self.step(rotor_speeds)
        return info
