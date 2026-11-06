# Design decisions

Architecture Decision Records (ADR). Each record states the context, the
decision, the alternatives considered and the consequences. New records are
appended as the project evolves.

## ADR-001: Simulator version (Isaac Sim 5.1.0, pinned)
- **Context:** In Nov 2026 the newest Isaac Sim line is 6.x (6.0 GA mid-2026, 6.2
  reported in Oct 2026). Pegasus Simulator v5.1.0 is the latest *official*
  release and targets Isaac Sim 5.1.0 only; Isaac Sim 6.0 support exists as an
  open upstream pull request and a community fork.
- **Decision:** Pin **Isaac Sim 5.1.0 + Pegasus Simulator v5.1.0**.
- **Alternatives:** Isaac Sim 6.x with an unreleased Pegasus port (risk of
  breakage outside our control); Isaac Sim 4.5 (older, Python 3.10).
- **Consequences:** A stable, officially matched pair. All simulator-specific
  code lives in `dronenav.sim.isaac_pegasus`, so moving to 6.x later only
  touches that adapter.

## ADR-002: Integration approach (Pegasus with our own Python backend)
- **Context:** Options compared in Phase 0: (a) Pegasus Simulator, (b) Isaac Lab,
  (c) raw Isaac Sim API with forces written from scratch.
- **Decision:** (a) **Pegasus**, using a custom Python `Backend` that receives the
  state and sensor data and returns rotor angular velocities. Pegasus provides
  the rotor/drag dynamics and the IMU/GPS/barometer/magnetometer models; every
  autonomy component (estimation, control, perception, planning, avoidance,
  mission) is written in this project.
- **Alternatives:** Isaac Lab targets reinforcement learning (vectorised envs, the
  body wrench is applied directly), not missions. A from-scratch model would spend
  weeks on physics plumbing without adding value.
- **Consequences:** PX4 SITL remains possible as an optional bonus, because
  Pegasus ships a PX4 MAVLink backend.

## ADR-003: Portable runtime (official container)
- **Decision:** The runtime is a Docker image built `FROM nvcr.io/nvidia/isaac-sim:5.1.0`
  with Pegasus pinned by git tag. The repository is bind-mounted, so the same
  commands work on a local PC, on AWS or on any RTX Linux host.
- **Consequences:** Host requirements reduce to an NVIDIA driver, Docker and the
  NVIDIA Container Toolkit. The scientific stack inside Isaac Sim is frozen with
  a pip constraints file to avoid accidental upgrades.

## ADR-004: Host operating system (Ubuntu 22.04 LTS)
- **Context:** Pegasus v5.1.0 states Ubuntu 22.04 as its tested OS; Isaac Sim 6.2
  deprecates 22.04 in favour of 24.04.
- **Decision:** **Ubuntu 22.04 LTS** for native installs; with the container,
  22.04 or 24.04 hosts both work, because the OS inside the image is fixed.
- **Note:** this revises the Phase 0 proposal (24.04 first), after checking the
  Pegasus v5.1.0 README during Phase 1.

## ADR-005: Simulator-agnostic core + NumPy simulator
- **Decision:** Algorithms never import Isaac Sim. A pure-NumPy quadrotor model
  (Phase 2) implements the same backend interface.
- **Consequences:** Unit tests and CI run on any machine without a GPU, parameter
  sweeps are fast, and a cross-validation between the two simulators becomes a
  report section.

## ADR-006: Frame conventions
- **Decision:** World **ENU**, body **FLU**, scalar-last quaternions `[qx, qy, qz, qw]`.
- **Rationale:** These are the native conventions of Isaac Sim and of Pegasus'
  `State` object (it also offers NED/FRD getters for PX4). One convention end to
  end avoids a whole class of sign bugs.

## ADR-007: No ROS 2 in the main pipeline
- **Decision:** All modules run in-process within one Python application.
- **Rationale:** ROS 2 adds installation and debugging complexity without serving
  any project requirement. It can be added as an optional visualisation bridge.

## ADR-008: Sensors (3D LiDAR primary, depth camera secondary)
- **Decision:** A 360° RTX LiDAR is the primary obstacle sensor (it sees suspended
  obstacles and side and rear threats); a forward depth camera is fused into the
  same voxel map. A ground-truth perception mode exists for debugging only.
- **To verify in Phase 7:** the exact Isaac Sim 5.1 LiDAR API, and whether Pegasus
  5.1 exposes LiDAR as a graphical sensor or we attach it ourselves.
