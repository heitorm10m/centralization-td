# Roadmap

## Phase 1

- Repository scaffold at the workspace root
- Devcontainer and onboarding instructions
- CMake build for C++ core, pybind11 bindings, and smoke tests
- Python package with YAML I/O and Typer CLI
- Deterministic solver stub with explicit TODOs

## Phase 2

- Trajectory validation for MD, inclination, and azimuth
- Approximate coordinate accumulation from survey angles
- Interpolation, local tangent, and discrete curvature utilities
- Richer string sections with elastic, density, and friction metadata
- Richer centralizer specs with count hints or explicit installation MDs
- Geometric baseline report for clearance, curvature risk, and section summaries
- Verification cases for YAML parsing, geometry utilities, and CLI execution

## Phase 3

What exists:

- MD discretization into mechanical segments
- Section-wise `E`, `I`, `EI`, dry weight, and buoyant/effective line weight handling
- First quasi-static axial-load profile from buoyant tangential weight accumulation
- Curvature-driven bending moment and bending stress metrics
- First mechanically informed contact/standoff proxies

## Phase 4

What now exists:

- Separate modules for discretization, centralizer support, lateral equilibrium, contact model, and outputs
- Segmentwise lateral equilibrium from gravity loading, curvature loading, bending stiffness, and axial tension
- Annular restriction through pipe-body clearance and support-contact clearance
- Simple iterative penalty contact at the segment level
- Eccentricity estimate, contact state, normal reaction estimate, and standoff estimate
- Centralizer effect represented as nominal local centering stiffness plus support-contact onset
- Tests for vertical trivial behavior, curvature sensitivity, centralizer effect, standoff limits, and CLI/JSON execution

Explicit limitations of Phase 4:

- Survey geometry is still approximate.
- The solver is not a full global stiff-string equilibrium solver.
- Contact is still local and penalty-based, not a full nonlinear wall reaction solve.
- Friction is not propagated into torque and drag.
- Centralizers still do not use a detailed bow-spring constitutive model.

## Phase 5

What now exists:

- Reduced global scalar displacement solve along MD with coupling between neighboring nodes
- Global assembly with bending contribution, axial-tension geometric stiffening, and lateral equivalent loading
- Global active-set penalty contact against body clearance and centralizer support clearance
- Centralizers represented as nominal local spring supports in the global system
- Global eccentricity, standoff, contact-state, normal-reaction, bending-moment, bending-strain, and axial-load profiles
- Tests covering global assembly shape, vertical trivial behavior, curvature sensitivity, centralizer effect, physical limits, and CLI/JSON execution

Explicit limitations of Phase 5:

- The model is still scalar and reduced, not a full 3D annular stiff-string formulation.
- The bending term still uses the simply supported beam equivalence delta_max = 5 q L^4 / (384 E I).
- Contact is still penalty-based and only mildly iterative.
- Friction is still not propagated into torque and drag.
- Centralizers still do not use a detailed bow-spring constitutive model.

## Phase 6

What now exists:

- Reduced axial drag propagation for `run_in` and `pull_out` using the Phase 5 normal-reaction profile
- Reduced hookload estimates for run in and pull out
- Reduced rotational torque integration using `mu * N * r`
- JSON and CLI outputs for axial-force profiles, hookloads, torque profile, and surface torque
- Verification cases covering friction sensitivity, reaction sensitivity, torque sensitivity, and sign coherence

Explicit limitations of Phase 6:

- Torque and drag are still reduced and depend on the Phase 5 scalar normal-reaction model.
- Axial friction is propagated segmentwise and does not come from a full nonlinear 3D contact/friction solve.
- Rotational torque is quasi-static and does not include stick-slip, surge/swab, or detailed rotational operating modes.
- Centralizers still do not use a detailed bow-spring constitutive model.

## Phase 7

What now exists:

- First iterative coupling loop between the reduced global lateral/contact solver and the reduced torque/drag module
- Axial-profile update inside the coupling loop with configurable iteration count, tolerance, and relaxation factor
- Converged axial, normal-reaction, and torque profiles exported in JSON
- Explicit documentation of the physical meaning of `N` in the reduced torque/drag model:
  segment resultant normal reaction `[N]`, not distributed load `[N/m]`
- Explicit sign conventions for `run_in/slackoff` and `pull_out/pickup`

Explicit limitations of Phase 7:

- The lateral/contact model is still scalar and reduced.
- Friction is still reduced `mu * N` post-processing, not full 3D annular friction.
- Torque remains reduced `mu * N * r` integration.
- Centralizers still do not use a detailed bow-spring constitutive model.

## Phase 8

What now exists:

- Reduced local-frame construction along the trajectory using tangent plus transported normal/binormal directions
- Two transverse DOFs per node in the local `n-b` plane
- Vector global assembly with bending contribution, axial-tension geometric stiffening, and local spring support
- Vector annular contact with magnitude and direction of normal reaction
- Vector eccentricity, contact-direction, and normal-reaction profiles exported to JSON
- The reduced torque/drag coupling retained on top of the vector contact solution
- Tests covering frame orthonormality, vector eccentricity/contact behavior, centralizer effect, and CLI/JSON execution

Explicit limitations of Phase 8:

- The frame is still built with a reduced parallel-transport construction, not a full differential-geometry reference implementation.
- The structural solve is still reduced and does not yet use a full 6-DOF spatial beam element.
- Contact is still penalty-based and only mildly iterative.
- Torque and drag still use reduced `mu * N` and `mu * N * r` laws, not full vector tangential contact mechanics.
- Centralizers still do not use detailed bow-by-bow stiffness or force laws.

## Phase 9

What now exists:

- Detailed bow-spring data for each centralizer: number of bows, angular reference, internal pipe clearance, optional power-law parameters, and optional min/max contact diameters
- Uniform angular bow distribution in the local `n-b` frame:
  `alpha_i = alpha_ref + 2 pi i / number_of_bows`
- Reduced bow deflection law:
  `delta_i = max(0, e . r_i - (c_support + c_inner))`
- Reduced constitutive bow force law:
  `F_i = k_blade * delta_i^p`
- Bow-by-bow force vectors and vector resultant:
  `R_bow = sum_i F_i r_i`
- Detailed centralizer outputs in JSON: bow force vectors/magnitudes, bow resultant vector/magnitude, centralizer axial friction, centralizer tangential friction, and centralizer torque profile
- Reduced torque split between pipe-body contribution and detailed centralizer contribution

Explicit limitations of Phase 9:

- The structural solve still uses the reduced 2-DOF transverse field from Phase 8, not a full 6-DOF beam element.
- Bow forces are post-processed consistently in the local frame, but the global structural solve still uses a reduced equivalent support stiffness/onset.
- Tangential friction from bows is still reduced and quasi-static, not a full nonlinear vector friction/contact law.
- Global torque and drag remain reduced engineering estimates, not commercial-grade predictions.

## Phase 10

What now exists:

- Versioned benchmark cases in `benchmarks/` covering vertical, deviated, symmetric-centralizer, symmetry-breaking, and constitutive/friction sensitivity studies
- A benchmark-suite runner that executes the YAML cases, writes per-case JSON payloads, and evaluates cross-case validation rules
- Reduced bow-spring calibration utilities for fitting `k` and `p` from force-deflection pairs or reduced nominal force points
- Traceability fields in the JSON payload for resolved centralizer parameters, convergence criteria, and final coupling update magnitude
- Additional C++ and Python regression coverage for friction trend, bow-stiffness trend, calibration recovery, benchmark execution, CLI, and JSON outputs

Explicit limitations of Phase 10:

- The benchmark suite currently validates internal consistency and expected reduced-order trends, not equivalence to literature or commercial software.
- The calibration workflow only fits the reduced bow power-law parameters and does not yet ingest manufacturer curves, hysteresis, wear, or uncertainty bands.
- The structural/contact core is still the reduced Phase 9 model, not a full 6-DOF stiff-string beam/contact formulation.
- Global torque and drag remain reduced engineering estimates even when the benchmark checks pass.

## Working Rules

- Keep SI units internal even if external reports later expose oilfield units.
- Keep input data externalized in YAML instead of embedding cases in solver code.
- Treat Phase 9 mechanical and torque/drag outputs, and the Phase 10 validation overlays built on them, as limited reduced-order mechanics rather than final engineering predictions.
