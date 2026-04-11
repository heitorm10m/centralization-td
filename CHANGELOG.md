# CHANGELOG — Implementation Status

This file tracks which parts of Dao et al. 2023 are implemented,
partially implemented, or missing. Update this file whenever
a new physics component is implemented or corrected.

---

## IMPLEMENTED (matches paper)

- [x] Blade angular orientation: αb,i = αb0 + (i-1)·2π/Nb
      Location: bow_spring.py:88
- [x] Restoring force as vector sum of blade forces (Eq. 1)
      Location: bow_spring.py:247
- [x] Blade loses contact independently when δ ≤ 0
      Location: bow_spring.py:109
- [x] Blade force power law Fi = kblade·δi^p (Eq. 4, qualitative)
      Location: bow_spring.py:133

## PARTIAL (exists but deviates from paper)

- [ ] Local frame (eu, ev, ew): parallel transport exists but not
      the paper's explicit high-side/right-side formula
      Location: frames.py:47
- [ ] Clearance φ between pipe and rings: value exists as input
      but not derived from diameters via Eq. 2
      Location: models.py:535
- [ ] Casing rotation inside rings: reduced torsional state exists
      but no explicit pipe-ring rotational DOF
      Location: torsional_reduced_model.py

## NOT IMPLEMENTED (missing from paper)

- [ ] 6-DOF Euler-Bernoulli beam FEM (Section 3.1)
- [ ] Global stiffness matrix from beam element matrices
- [ ] Initial curvature stress via step-by-step deformation
- [ ] Smooth Heaviside contact law h(x) with parameter λ (Eq. 17)
- [ ] Velocity-dependent friction μ(Vg) (Eq. 20)
- [ ] Directional friction split μθ and μz (Eq. 19)
- [ ] Blade position vector rb,i via Eq. 22
- [ ] Fully-compressed blade regime with kc switch (Eq. 24)
- [ ] Axial friction as algebraic scalar sum (Eq. 25)
- [ ] Blade-wise tangential friction vectors fcbθ,i (Eq. 28)
- [ ] Full nodal force and moment per blade (Eq. 29)
- [ ] Coefficients ccbu,i and ccbv,i
- [ ] Newton-Raphson residual/Jacobian solver (Eq. 31)
- [ ] Analytical Jacobian of body contact forces (Eq. 35)
- [ ] Analytical Jacobian of bow-spring contact forces (Eq. 36)
- [ ] Two-step API 10D calibration workflow (Eq. 9 + gradient method)
- [ ] Friction factor estimation from running force (Eq. 10)
- [ ] Open-hole vs cased-hole friction coefficient switch
---
