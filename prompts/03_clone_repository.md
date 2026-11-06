# Prompt 03 — Attach the GitHub repository

- **Date:** 2026-11-06
- **Phase:** 0 → 1 (setup)
- **Goal:** Give the LLM session access to the newly created GitHub repository.

## Exact prompt text

```text
Clona il repository msant93/isaac-drone-nav in questa sessione.
```

*(English translation: "Clone the repository msant93/isaac-drone-nav into this session.")*

## Summary of the response
- The LLM attached the repository and cloned it. It contained one commit with only an MIT `LICENSE`.
- It confirmed it would not push without explicit authorisation.
- It asked for confirmation of the Phase 0 choices: the 5.1.0 pin, the time assumptions and the optional PX4 phase.

## Files generated/modified
None.
