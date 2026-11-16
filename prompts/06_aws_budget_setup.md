# Prompt 06 — Low-budget AWS workstation, step by step

- **Date:** 2026-11-16
- **Phase:** 1 (infrastructure)
- **Goal:** Get exact, cost-minimising instructions to run and test the project on AWS.

## Exact prompt text

```text
I Need to know step by step what I have to do to launch this project on AWS.  My budget is low, so I want to save as much as I can. I just need a workstation in which I can test the code
```

## Summary of the response
- **Pricing and support check:** the LLM looked up current EC2 GPU pricing and the Isaac Sim AWS guidance. The NVIDIA docs host was not reachable from its session, so unconfirmed items are marked ⚠ with where to check them.
- **Correction:** the Tesla T4 *has* RT cores. It is no longer flagged as unsupported in `scripts/check_env.py` and `docs/setup.md`, and `g4dn.2xlarge` becomes the cheapest candidate instance.
- **New guide** `docs/aws_budget_guide.md`:
  - cost strategy: AWS only for Isaac runs; `g4dn.2xlarge` ~$0.75/h on-demand, ~$0.25–0.32/h Spot; `g6.2xlarge` as fallback; 100 GB gp3 disk;
  - account security and a budget alert; GPU vCPU quota requests; SSH key;
  - launch settings with the free "Deep Learning Base OSS Nvidia Driver GPU AMI (Ubuntu 22.04)" and an optional persistent Spot request;
  - first setup and smoke test; a CloudWatch auto-stop alarm when idle;
  - daily workflow (start, VS Code Remote-SSH, run, stop); optional WebRTC livestream;
  - changing instance type; a checklist of what costs money; clean-up at the end; troubleshooting;
  - estimated total cost for the project: about $50–60.

## Files generated/modified
- `docs/aws_budget_guide.md` (new)
- `docs/setup.md`, `README.md` (link to the guide; T4 correction)
- `scripts/check_env.py` (T4 is RTX-capable)
- `prompts/05_project_explanation.md`, `prompts/06_aws_budget_setup.md`, `prompts/README.md`
