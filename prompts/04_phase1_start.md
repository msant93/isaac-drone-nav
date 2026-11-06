# Prompt 04 — Start Phase 1 (repository skeleton, Docker, CI, smoke test)

- **Date:** 2026-11-06
- **Phase:** 1
- **Goal:** Switch the conversation to English and ask the LLM to prepare the first content to push. This implicitly accepts the Phase 0 defaults: Isaac Sim 5.1.0 + Pegasus v5.1.0, PX4 as an optional final phase.

## Exact prompt text

```text
Now, we have to speak English in this conversation and English is the language of this project. Based on what you know about this project, you can start to tell me what you have to push
```

## Summary of the response
- **Verification:** the LLM read the Pegasus v5.1.0 sources on GitHub to confirm the APIs instead of assuming them:
  - the `Backend` interface;
  - the `State` frames (ENU/FLU, scalar-last quaternions);
  - `Multirotor` / `MultirotorConfig`;
  - `force_and_torques_to_velocities`;
  - the 250 Hz physics default;
  - the `ISAACSIM_PATH` requirement of `setup.py`;
  - Ubuntu 22.04 as the tested OS.
- **Generated:**
  - the repository skeleton (simulator-agnostic `dronenav` package with typed subpackages);
  - packaging (`pyproject.toml`), the conda and pip requirement files, the Makefile and pre-commit;
  - a Docker image (`FROM nvcr.io/nvidia/isaac-sim:5.1.0` + Pegasus v5.1.0, with numpy/scipy pinned through a constraints file) and run scripts for headless, WebRTC livestream and local GUI;
  - GitHub Actions CI (ruff, mypy --strict, pytest);
  - a host diagnostics script;
  - an Isaac Sim smoke test: the Pegasus Iris takes off, hovers and lands through a custom Python backend with a temporary geometric controller, and the test writes a pass/fail JSON report and a CSV trajectory;
  - unit tests;
  - docs: setup (local / AWS / native), architecture with Mermaid diagrams, ADRs;
  - this prompt log.
- **Change from Phase 0:** native host OS changed to Ubuntu 22.04 (ADR-004).
- **Checks:** lint, typecheck and tests pass locally (19 tests). The Isaac Sim smoke test has to be run by the user on a GPU machine.

## Files generated/modified
- `README.md`, `.gitignore`, `.dockerignore`, `.pre-commit-config.yaml`, `Makefile`
- `pyproject.toml`, `environment.yml`, `requirements.txt`, `requirements-isaac.txt`
- `.github/workflows/ci.yml`
- `docker/Dockerfile`, `docker/docker-compose.yml`, `docker/_common.sh`, `docker/run_headless.sh`, `docker/run_livestream.sh`, `docker/run_gui.sh`
- `config/smoke_test.yaml`
- `scripts/check_env.py`, `scripts/smoke_test_hover.py`
- `src/dronenav/**/__init__.py`, `src/dronenav/py.typed`
- `tests/test_package.py`, `tests/test_configs.py`, `tests/test_check_env.py`
- `docs/setup.md`, `docs/architecture.md`, `docs/design_decisions.md`, `docs/diagrams/README.md`, `docs/report/README.md`
- `scenarios/README.md`, `results/README.md`
- `prompts/README.md`, `prompts/01_initial_prompt.md` … `prompts/04_phase1_start.md`
