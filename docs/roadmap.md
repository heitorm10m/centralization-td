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

- Fuller nonlinear contact and wall reaction iteration
- Torque and drag propagation with friction coupling
- Optimizer for centralizer spacing and placement
- Batch scenario execution and comparison
- Richer reporting outputs
- Calibration and validation against field or laboratory data

## Working Rules

- Keep SI units internal even if external reports later expose oilfield units.
- Keep input data externalized in YAML instead of embedding cases in solver code.
- Treat Phase 5 mechanical outputs as limited reduced-order mechanics, not final engineering predictions.
