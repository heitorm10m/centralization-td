# DECISIONS.md

This file records implementation decisions and the reasons behind them.
Before reverting or changing any decision listed here, consult
docs/PHYSICS_REFERENCE.md and the relevant paper equation.

---

## DEC-001 — Smooth Heaviside instead of linear penalty for contact

Status: PENDING IMPLEMENTATION
Paper reference: Dao et al. 2023, Eq. 17
Reason: The paper requires a smooth C1 approximation of the Heaviside
function for stability of the Newton-Raphson solver. Linear penalty
contact is not acceptable. Do not revert to linear penalty.

Formula:
  h(x) = (1 - 1/(1 + (x/λ)²)) · H(x)
  where h(λ) = 0.5 and h(x) = 0 exactly for x ≤ 0

---

## DEC-002 — Axial friction is a scalar sum, tangential is a vector sum

Status: PENDING IMPLEMENTATION
Paper reference: Dao et al. 2023, Eq. 25 (scalar) and Eq. 26 (vector)
Reason: The two friction types have physically different natures.
Axial friction acts on blades against the borehole (all in same
direction ew), so forces are summed algebraically as scalars.
Tangential friction acts between string and rings, so individual
blade force vectors must be summed as vectors before applying μθ.
Mixing these two summation types is a critical implementation error.

---

## DEC-003 — Analytical Jacobians, not numerical differentiation

Status: PENDING IMPLEMENTATION
Paper reference: Dao et al. 2023, Eq. 35 (body) and Eq. 36 (bow-spring)
Reason: The paper derives and provides all Jacobian expressions
analytically. Numerical differentiation is slower and less accurate
for a Newton-Raphson solver on this class of nonlinear contact problem.

---

## DEC-004 — Each blade treated independently

Status: PARTIAL (blade orientation and force exist, but Eq. 22-29
kinematics and fully-compressed kc regime are missing)
Paper reference: Dao et al. 2023, Section 2, Eqs. 22-29
Reason: The paper explicitly requires per-blade contact detection,
per-blade force computation, and per-blade moment projection.
A single resultant approach is not sufficient.

---

## DEC-005 — Velocity-dependent friction coefficient

Status: NOT IMPLEMENTED
Paper reference: Dao et al. 2023, Eq. 19 and Eq. 20
Reason: The paper requires friction to depend on slip velocity
direction and magnitude. Static and dynamic coefficients are
distinct. A constant friction coefficient is not acceptable.

---

## DEC-006 — 6-DOF Euler-Bernoulli beam elements

Status: NOT IMPLEMENTED (current code uses 2-DOF reduced model)
Paper reference: Dao et al. 2023, Section 3.1
Reason: The paper requires full 6-DOF stiff-string FEM with
bending moments, shear forces, and torsion. The current 2-DOF
transverse model is explicitly a reduced scaffold.
---
