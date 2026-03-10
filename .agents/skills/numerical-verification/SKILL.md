---
name: numerical-verification
description: Add or refine verification coverage for scientific and numerical code. Use when introducing solver changes, regression tests, smoke tests, deterministic reference outputs, or validation rules for numerical workflows.
---

# Numerical Verification

## Goal

Ensure every solver-facing change is accompanied by a test or deterministic check that catches regressions early.

## Workflow

- Start with smoke coverage for buildability, importability, and basic data flow.
- Prefer deterministic inputs and outputs over probabilistic checks.
- Separate validation of schemas and units from validation of numerical results.
- Add explicit tolerances only when a calculation warrants them.
- Fail fast on missing required fields, invalid geometry, or non-monotonic trajectories.

## Done Criteria

- At least one automated check exercises the changed workflow end to end.
- Expected outputs or failure modes are explicit.
- Tests are small enough to run in local development without heavy setup.

