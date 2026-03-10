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

- Stiff-string representation for tubular mechanics
- Contact detection and reaction forces along the well trajectory
- Real side-force prediction
- Real standoff estimation
- Torque and drag model coupled to axial load response
- Detailed bow-spring centralizer behavior
- Expanded benchmark library and regression baselines
- Plotting and report templates for engineering studies

## Phase 4

- Optimizer for centralizer spacing and placement
- Batch scenario execution and comparison
- Richer reporting outputs
- Calibration and validation against field or laboratory data

## Working Rules

- Keep SI units internal even if external reports later expose oilfield units.
- Keep input data externalized in YAML instead of embedding cases in solver code.
- Treat Phase 2 geometry outputs as scaffolding only until verified mechanics land.
