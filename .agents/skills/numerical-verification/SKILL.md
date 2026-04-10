---
name: numerical-verification
description: Use when changing solver equations, contact algorithms, constitutive laws, iterative coupling, or any numerical core behavior. Do not use for trivial docs-only edits.
---

# Goal

Verify numerical robustness and consistency after solver-side changes.

# Workflow

1. Identify affected equations and assumptions.
2. Check dimensional consistency.
3. Check sign conventions.
4. Check residual definitions, convergence criteria, and iteration caps for affected iterative paths.
5. Run targeted Python tests.
6. Run CLI smoke validation.
7. Summarize what changed, what passed, and what remains uncertain.

# Done criteria

- dimensional consistency checked
- convergence behavior checked for affected iterative paths
- relevant tests updated
- pytest/CLI pass
- uncertainties are explicitly listed
