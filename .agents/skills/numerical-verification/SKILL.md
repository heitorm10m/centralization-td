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
4. Run targeted C++ and Python tests.
5. Run CLI smoke validation.
6. Summarize what changed, what passed, and what remains uncertain.

# Done Criteria

- dimensional consistency checked
- relevant tests updated
- build, `ctest`, `pytest`, and CLI pass
- uncertainties are explicitly listed
