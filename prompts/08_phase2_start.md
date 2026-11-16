# Prompt 08 — Phase 2: core math, configuration, simulator interface, NumPy simulator

- **Date:** 2026-11-16
- **Phase:** 2
- **Goal:** Implement the simulator-agnostic foundations of the autonomy stack.

## Exact prompt text

```text
ok, start phase 2
```

## Summary of the response
- **Verification:** the LLM first read the Pegasus v5.1.0 actuation source to match it exactly: `QuadraticThrustCurve` defaults, the `force_and_torques_to_velocities` allocation and saturation, `LinearDrag`, and per-rotor force application.
- **`core`:**
  - `types.py`: `VehicleState`, `ImuMeasurement`, `SensorData`;
  - `math3d.py`: quaternions, rotation matrices, Z-Y-X Euler angles, SO(3) exp/log, exact quaternion integration;
  - `frames.py`: ENU/NED and FLU/FRD conversions;
  - `constants.py`.
- **`config`:**
  - strict, immutable pydantic schemas (`VehicleConfig`, `PythonSimConfig`) with physical checks: positive mass, inertia triangle inequality, thrust-to-weight margin, rotor directions;
  - a YAML loader whose errors name the file and the field.
- **`model/quadrotor.py`:** a Pegasus-identical actuation model (allocation matrix, mixer, saturation, drag).
- **`sim/base.py`:** the step-based `SimBackend` interface.
- **`sim/python_sim`:** a Newton-Euler simulator with RK4 or semi-implicit Euler, an optional motor lag, a flat ground with crash detection, an ideal IMU and an external-acceleration hook.
- **Configs:** `config/drone_iris.yaml` (with documented parameter sources) and `config/python_sim.yaml`.
- **Tests:** 57 new tests (76 in total, 97% coverage), including a check that the mixer matches a transcription of the Pegasus code.
- **Demo:** `scripts/python_sim_demo.py` runs four open-loop experiments against analytical solutions (errors ≤ 1e-12) and saves `results/phase2/python_sim_validation.png`.
- **Performance:** the dynamics were rewritten with scalar arithmetic, going from 16x to about 55x real time.
- **Fixes during the phase:**
  - a test that simulated 0.048 s instead of 0.05 s;
  - mypy strictness issues;
  - overlapping labels in the figure.
- **Docs:** architecture (simulator interface section, new `model` package), ADR-009, new `docs/limitations.md`, README status and figure, `make sim-demo`.

## Files generated/modified
- `src/dronenav/core/{types,math3d,frames,constants}.py`
- `src/dronenav/config/{schemas,loader}.py`
- `src/dronenav/model/{__init__,quadrotor}.py`, `src/dronenav/__init__.py`
- `src/dronenav/sim/base.py`, `src/dronenav/sim/python_sim/{__init__,dynamics,simulator}.py`
- `config/drone_iris.yaml`, `config/python_sim.yaml`
- `tests/conftest.py`, `tests/test_{math3d,frames,config_schemas,quadrotor_model,python_sim}.py`
- `scripts/python_sim_demo.py`, `results/phase2/python_sim_validation.png`
- `docs/architecture.md`, `docs/design_decisions.md`, `docs/limitations.md`, `README.md`, `Makefile`
- `prompts/07_phase2_explanation.md`, `prompts/08_phase2_start.md`, `prompts/README.md`
