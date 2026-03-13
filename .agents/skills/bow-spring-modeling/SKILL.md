---
name: bow-spring-modeling
description: Use when implementing or modifying detailed bow-spring centralizer behavior, including bow geometry, angular orientation, inner clearance, constitutive laws, and vector resultants. Do not use for generic rigid-support edits.
---

# Goal

Implement bow-spring centralizers as explicit, physically interpretable components rather than generic scalar supports.

# Rules

- Model bows individually when this skill is active.
- Do not collapse bow-by-bow behavior back into a single equivalent scalar support without explicit justification and honest labeling.
- Keep bow geometry, constitutive law, and resultant calculation in separate modules.
- Use honest names for provisional parameters and outputs.
- Support future calibration by keeping constitutive parameters explicit.
- Document what is geometric assumption and what is constitutive law.
- Keep axial and tangential centralizer effects distinguishable in outputs and docs.

# Preferred physics direction

- bow count
- angular orientation
- inner clearance to pipe
- nonlinear per-bow force law
- vector resultant in the transverse plane

# Done criteria

- bow-wise logic is explicit
- vector resultant is available
- tests cover symmetry and asymmetry cases
- docs explain assumptions and limits
