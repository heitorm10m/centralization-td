---
name: repo-bootstrap
description: Scaffold or reorganize a scientific computing repository for Python and NumPy-based scientific development. Use when the task is to create base folders, build files, environment setup, examples, tests, and first-pass documentation.
---

# Repo Bootstrap

## Goal

Create a coherent repository skeleton that can build, run smoke tests, and evolve without rework.

## Workflow

- Inspect the current workspace before creating files.
- Preserve existing structure unless it blocks the requested architecture.
- Keep the numerical core in Python and use optimized numerical libraries such as NumPy for dense algebra.
- Add the minimum viable build, packaging, examples, and tests needed to prove the scaffold works.
- Prefer simple contracts and explicit TODOs over speculative implementation.

## Done Criteria

- The repository has a clear root layout, build entry points, examples, and smoke tests.
- Development environment files exist for repeatable onboarding.
- README-level instructions are enough for a new contributor to build and run the project.

