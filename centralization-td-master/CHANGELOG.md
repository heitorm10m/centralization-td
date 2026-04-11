# CHANGELOG - Implementation Status

This file tracks which parts of Dao et al. 2023 are implemented,
partially implemented, or missing. Update this file whenever
a new physics component is implemented or corrected.

---

## IMPLEMENTED (matches paper)

- [x] Smooth Heaviside function h(x) (Eq. 17)
      Location: physics/contact.py
- [x] Gravity and buoyancy nodal forces (Eq. 13)
      Location: physics/fem.py::equivalent_gravity_buoyancy_nodal_loads
- [x] Global body force vector assembly (Eq. 14)
      Location: physics/fem.py::build_gravity_buoyancy_force_vector
- [x] Local frame (eu, ev, ew) from Dao et al. Eq. 11
      Location: frames.py
- [x] Blade angular orientation: alpha_b,i = alpha_b0 + (i-1)*2pi/Nb
      Location: physics/bow_spring.py:88
- [x] Restoring force as vector sum of blade forces (Eq. 1)
      Location: physics/bow_spring.py:247
- [x] Blade loses contact independently when delta <= 0
      Location: physics/bow_spring.py:109
- [x] Blade deflection geometry (Eq. 5)
      Location: physics/bow_spring.py::blade_deflection_geometry_m
- [x] Blade force power law Fi = kblade * delta_i^p (Eq. 4, qualitative)
      Location: physics/bow_spring.py
- [x] 6-DOF beam element stiffness matrix (Section 3.1)
      Location: physics/beam_element.py
- [x] Local-to-global transformation (Section 3.1)
      Location: physics/beam_element.py::local_to_global_matrix
- [x] Global stiffness assembly (Section 3.1)
      Location: physics/beam_element.py::assemble_global_stiffness
- [x] Newton-Raphson solver (Eq. 31)
      Location: solver/mechanics.py::solve_6dof_newton_raphson
- [x] Body contact Jacobian (Eq. 35) ⚠️ PARTIAL
      Location: physics/contact.py
      Note: terms derived from Eq. 21 differentiation, some terms uncertain, marked in code with WARNING comments
- [x] Bow-spring contact Jacobian (Eq. 36) ⚠️ PARTIAL
      Location: physics/bow_spring.py
      Note: chain-rule form used for f_cbz,i terms, marked in code

## PARTIAL (exists but deviates from paper)

- [ ] Initial curvature stress via step-by-step deformation
      Location: physics/fem.py::build_initial_curvature_force
      Note: paper describes concept only, algorithm not algebraically complete in available extracts. Returns zero vector.
- [ ] Clearance phi between pipe and rings: value exists as input
      but not derived from diameters via Eq. 2
      Location: models.py:535
- [ ] Casing rotation inside rings: reduced torsional state exists
      but no explicit pipe-ring rotational DOF
      Location: torsional_reduced_model.py

## NOT IMPLEMENTED (missing from paper)

- [ ] Velocity-dependent friction mu(Vg) (Eq. 20)
- [ ] Directional friction split mu_theta and mu_z (Eq. 19)
- [ ] Blade position vector rb,i via Eq. 22
- [ ] Fully-compressed blade regime with kc switch (Eq. 24)
- [ ] Axial friction as algebraic scalar sum (Eq. 25)
- [ ] Blade-wise tangential friction vectors fcbtheta,i (Eq. 28)
- [ ] Full nodal force and moment per blade (Eq. 29)
- [ ] Coefficients ccbu,i and ccbv,i
- [ ] Two-step API 10D calibration workflow (Eq. 9 + gradient method)
- [ ] Friction factor estimation from running force (Eq. 10)
- [ ] Open-hole vs cased-hole friction coefficient switch
---
