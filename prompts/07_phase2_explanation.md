# Prompt 07 — What is Phase 2?

- **Date:** 2026-11-16
- **Phase:** 2 (planning)
- **Goal:** Understand the content of Phase 2 and how it differs from Phase 1.

## Exact prompt text

```text
explain to me what is phase 2, What changes from phase 1
```

## Summary of the response
The LLM explained the difference between the two phases:
- **Phase 1** built the infrastructure: repository, Docker, CI, and a smoke test proving that Isaac Sim + Pegasus can fly a drone.
- **Phase 2** builds the foundations of the drone software:
  - core math (quaternions, SO(3), ENU/FLU frames, shared types);
  - typed pydantic configuration schemas with readable validation errors;
  - the abstract `SimBackend` interface that keeps the autonomy code simulator-independent;
  - a pure-NumPy 6-DoF quadrotor simulator with Pegasus-matching Iris parameters, used for speed, zero cost, CI flight tests and cross-validation in the report;
  - physics tests (hover, free fall, torque response, round trips, config validation).

It noted that Phase 2 needs no GPU and contains no controller (that is Phase 3), and listed what the user does in parallel (AWS quota, smoke test).

## Files generated/modified
None.
