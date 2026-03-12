---
name: benchmark-cases
description: Curate benchmark or example cases for centralization and torque and drag studies. Use when creating YAML inputs, canonical datasets, case manifests, or benchmark documentation for repeatable solver comparisons.
---

# Benchmark Cases

## Goal

Maintain portable, versioned benchmark inputs that can be reused across phases of the project.

## Workflow

- Store well, string, and centralizer data as external configuration files.
- Keep example cases minimal but physically interpretable.
- Prefer one canonical manifest that references reusable config fragments.
- Document what each case is intended to exercise and what it does not yet verify.
- State when a benchmark supports internal trend checking or calibration only, and not external quantitative validation.
- Avoid embedding benchmark data directly in source code.

## Done Criteria

- Example or benchmark YAML files load without ad hoc path fixes.
- Each case has a stated purpose and traceable source files.
- The repository can run a baseline example from a clean checkout.

