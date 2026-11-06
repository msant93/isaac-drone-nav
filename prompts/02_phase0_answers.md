# Prompt 02 — Answers to the clarifying questions (Phase 0)

- **Date:** 2026-11-06
- **Phase:** 0
- **Goal:** Give the LLM the constraints it needs to propose the environment, the integration option, the architecture and the plan.

## Exact prompt text

```text
About the Hardware I don't have any specific constraint, I just know that I have to use NVIDIA Isaac Sim simulator. Also about OS, give me some advice about it. I may use AWS, but I don't know if I can use another type of remote workstation. But, I need the code to be not dependent on some specific remote workstation, like if I decide to run the code using AWS or another remote workstation, the code for the drone must work anyway. I have experience with OOP programming, linux command, control theory, git, github and docker, but I don't have any specific experience about aerial drone. The code doesn't need to be completely generated from scratch, so I can use third-party libraries or extensions. I aim to get a high score, so do this project like you were a great student in computer science who aims to achieve the top score. I don't have any specific constraints about the sensors, so you can choose what you consider more fit, based on my requirements. Besides the repo, it'd be great to have a demo video and a written report at the end of the project. The language of the project is English. You can write the files directly into a github repository, but you must have my authorization before pushing the files into the repository. You can create the new repository inside my github account
```

## Summary of the response
The LLM first checked the current release status on the web (Isaac Sim 6.x, Pegasus, Isaac Lab), then proposed the following.

- **Environment:**
  - Isaac Sim **5.1.0**, pinned, together with **Pegasus Simulator v5.1.0**. This is an officially matched pair; Pegasus support for Isaac Sim 6.x was still an open PR.
  - The official NVIDIA container is the portable runtime: it runs the same on a local PC, on AWS or on any RTX host.
  - Ubuntu (22.04/24.04), Python 3.11, conda for host-side tooling.
  - Hardware tiers. On AWS, use only GPUs with RT cores (g5/g6/g6e) and request the GPU quota early.
  - VS Code with Remote-SSH, GitHub Actions CI, a LaTeX report; no ROS 2; PX4 as an optional bonus.
- **Integration comparison:** Pegasus vs Isaac Lab vs the raw Isaac Sim API. Recommendation: **Pegasus with a custom Python backend**, plus a pure-NumPy simulator backend for tests and CI.
- **Architecture:**
  - simulator adapters;
  - error-state EKF;
  - cascaded control with an SO(3) attitude controller;
  - LiDAR + depth perception with a voxel log-odds map and a KF tracker;
  - A*/RRT* with smoothing;
  - APF + velocity obstacles with local-minimum escalation;
  - mission FSM + safety supervisor;
  - telemetry.
- **Plan:** the repository structure, 9 scenarios (S0–S8) with metrics and ablations, and a 12-phase plan plus an optional PX4 phase.
- Items to verify in the official docs were flagged explicitly.
- GitHub token in the session was invalid. The LLM explained how to reconnect GitHub and asked the user to create an empty repository.

## Files generated/modified
None (proposal only).
