---
name: vector-frame-solver
description: Use when modifying the trajectory frame, local-coordinate geometry, vector lateral solver, vector contact handling, or any logic based on transverse degrees of freedom in the local well frame. Do not use for simple CLI or docs-only changes.
---

# Goal

Maintain and extend the vector structural solver in the local trajectory frame without regressing to scalar approximations.

# Rules

- Preserve the local-frame vector formulation with transverse components.
- Keep geometry and frame handling separate from contact, centralizer, and torque-drag code.
- Document the frame construction method and its numerical limitations.
- Do not present the current solver as a full 6-DOF spatial beam model unless that is actually implemented.
- Keep vector outputs explicit in direction, magnitude, and units.
- Add or update tests whenever frame or vector solver behavior changes.

# Done Criteria

- build passes
- solver tests pass
- vector outputs remain consistent
- docs mention new assumptions and remaining limitations
