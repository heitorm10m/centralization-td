---
name: physics-modeling
description: Define or revise physical modeling boundaries for casing centralization and torque and drag software. Use when tasks involve assumptions, model scope, governing simplifications, solver responsibilities, or explicit physics TODOs.
---

# Physics Modeling

## Goal

Keep modeling assumptions explicit, staged, and physically defensible while the solver evolves.

## Workflow

- State the current model scope before changing equations or data structures.
- Keep internal units in SI and name physical quantities explicitly.
- Separate placeholder behavior from validated mechanics.
- Record missing physics as TODOs tied to stiff-string, contact, standoff, bow-spring behavior, torque and drag, or optimization.
- Avoid hardcoding benchmark wells or completion strings inside the solver.

## Done Criteria

- Assumptions and exclusions are documented close to the code or docs they affect.
- Data structures map cleanly to physical concepts.
- New modeling steps leave a traceable path for verification and future refinement.

