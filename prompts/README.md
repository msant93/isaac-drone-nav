# Prompt log

Every prompt given to the LLM, in chronological order. Each file records the
date, the goal, the **exact prompt text**, a short summary of the response and
the files generated or modified.

| # | File | Phase | Topic |
|---|---|---|---|
| 01 | [01_initial_prompt.md](01_initial_prompt.md) | 0 | Role, requirements, workflow |
| 02 | [02_phase0_answers.md](02_phase0_answers.md) | 0 | Answers → environment, architecture, plan |
| 03 | [03_clone_repository.md](03_clone_repository.md) | 0→1 | Attach the GitHub repository |
| 04 | [04_phase1_start.md](04_phase1_start.md) | 1 | Skeleton, Docker, CI, smoke test |
| 05 | [05_project_explanation.md](05_project_explanation.md) | 1 | Detailed explanation of structure and next steps |
| 06 | [06_aws_budget_setup.md](06_aws_budget_setup.md) | 1 | Low-budget AWS workstation guide |
| 07 | [07_phase2_explanation.md](07_phase2_explanation.md) | 2 | What Phase 2 is and how it differs from Phase 1 |
| 08 | [08_phase2_start.md](08_phase2_start.md) | 2 | Core math, configs, simulator interface, NumPy simulator |
