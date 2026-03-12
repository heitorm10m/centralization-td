# centraltd

Phase 10 validation and calibration infrastructure for a scientific project focused on casing centralization and torque & drag, using a hybrid C++ + Python architecture. The repository keeps the Phase 9 reduced vector local-frame solver core, detailed bow-spring centralizer layer, and reduced coupled torque & drag baseline, and now adds benchmark cases, cross-case validation checks, calibration utilities for the reduced bow constitutive law, and richer output traceability.

It still does not implement a commercial stiff-string solver, a full 6-DOF beam/contact formulation, or fully nonlinear 3D contact/friction torque and drag.

## Objective

Build the project in phases while keeping the numerical kernel in C++, and the orchestration, YAML I/O, CLI, plotting, and reporting in Python.

## Architecture

- `cpp/`: C++20 core for trajectory geometry, section properties, discretization, centralizer support, lateral equilibrium, contact model, mechanical baseline, pybind11 bindings, and C++ smoke tests.
- `python/`: Python package for YAML loading, validation, fallback mechanics, CLI orchestration, and packaging.
- `configs/`: Reusable YAML fragments for wells, strings, and centralizers.
- `examples/`: Case manifests that reference reusable config fragments.
- `docs/`: Roadmap and engineering notes.

## Current Phase

Phase 10 currently implements the Phase 9 solver core plus:

- trajectory validation from MD, inclination, and azimuth
- approximate trajectory geometry by balanced-tangent integration
- interpolation, local tangent, and discrete curvature utilities
- richer string section properties including elastic and buoyancy-related helpers
- richer centralizer specs with count hints or explicit installation MDs
- MD discretization into mechanical segments
- effective line weight from dry weight minus buoyancy
- bending stiffness from `E * I`
- simplified effective axial load integration from buoyant tangential weight
- reduced global scalar-to-vector transition with two transverse DOFs per node in the local trajectory frame
- local tangent, normal, and binormal frame construction by a reduced parallel-transport method
- vector lateral equivalent loading in the local `n-b` plane from gravity projection and curvature
- vector global assembly with bending contribution, axial-tension geometric stiffening, and local spring support
- vector annular contact with active-set penalty iteration and vector normal reaction
- vector eccentricity, contact direction, normal-reaction vector, and standoff estimates from the global solution
- bow-spring centralizer geometry with explicit `number_of_bows`, angular reference, internal pipe clearance, and optional power-law parameters
- bow-by-bow deflection and force evaluation in the local transverse plane using `delta_i = max(0, e . r_i - (c_support + c_inner))`
- nonlinear bow force law `F_i = k_blade * delta_i^p` with optional direct `k/p` input or fallback calibration from nominal restoring force
- vector summation of individual bow forces into a local centralizer resultant
- reduced axial drag propagation for `run_in` and `pull_out` using `mu * N`, where `N` is the resultant normal reaction per segment `[N]`
- reduced rotational torque integration using `mu * N * r`
- separation between pipe-body drag/torque and detailed centralizer drag/torque contributions
- iterative coupling between the selected axial profile, the vector lateral/contact solve, and the reduced T&D post-processing
- hookload estimates for run in and pull out, plus a reduced surface-torque estimate
- canonical benchmark YAML cases for vertical, deviated, symmetric-centralizer, symmetry-breaking, and constitutive/friction sensitivity studies
- a benchmark-suite runner with case checks, cross-case comparisons, and JSON summaries
- reduced bow-spring calibration utilities from force-deflection pairs or reduced nominal force points
- output traceability for resolved centralizer parameters, convergence criteria, and final coupling update magnitude

## Validation Status

What is now validated inside the repository:

- sign coherence for run in vs pull out, drag, and reduced torque outputs
- expected trend of increasing curvature toward higher reduced lateral severity and eccentricity
- expected trend of increasing friction toward higher hookload differential and surface torque
- symmetric vs symmetry-broken bow-resultant behavior
- reduced reference bow-support stiffness sensitivity to `number_of_bows` and explicit `blade_power_law_k`
- regression of the reduced bow power-law calibration against synthetic force-deflection data
- benchmark and calibration CLI JSON generation

What is still not validated:

- comparison against published stiff-string benchmarks or commercial software
- calibration against manufacturer or laboratory bow-spring force-deflection data
- uncertainty bounds, repeatability studies, or statistical error models
- full nonlinear tangential contact/friction validation in 3D

## Hypotheses And Limitations

The Phase 9 baseline is intentionally limited:

- survey-derived coordinates remain approximate and are not a survey-processing reference implementation
- the column response is quasi-static and reduced to two transverse displacement components in the local normal/binormal plane
- the bending term still uses the equivalent simply supported beam relation `delta_max = 5 q L^4 / (384 E I)`, so the `384/5` factor is an explicit reduced structural hypothesis
- the local frame uses a reduced parallel-transport construction, not a full differential-geometry reference implementation
- the solver is global and coupled along MD, but it is still not a full 3D stiff-string beam/contact solve in the annulus
- contact is handled by a simple global vector active-set penalty iteration, not a full nonlinear wall-reaction model
- each bow direction is distributed uniformly from the angular reference: `alpha_i = alpha_ref + 2 pi i / number_of_bows`
- the local bow unit vector is `r_i = [cos(alpha_i), sin(alpha_i)]` in the `n-b` plane
- bow deflection is reduced to the positive radial projection past the support onset:
  `delta_i = max(0, e . r_i - (c_support + c_inner))`
- bow force is reduced to `F_i = k_blade * delta_i^p`, with `F_i >= 0` and `delta_i >= 0`
- the centralizer resultant is `R_bow = sum_i F_i r_i`
- axial drag uses reduced `mu * N` propagation with operation-dependent sign conventions:
  run in/slackoff subtracts `mu * N` from the local hookload increment, while pull out/pickup adds `mu * N`
- pipe-body torque uses reduced `mu * N_body * r_body`
- centralizer torque uses the bow-resultant magnitude scaled by the nominal running/restoring-force ratio and multiplied by an effective centralizer contact radius
- torque remains reduced and does not yet use full tangential vector friction or a bow-by-bow dynamic/contact solve

Use these outputs as structured engineering scaffolding with traceable validation checks, not as final design predictions.

## Roadmap By Phase

- Phase 1: scaffold, environment, build, bindings, YAML schema, CLI, smoke tests, and docs
- Phase 2: trajectory geometry utilities, richer tubular and centralizer data, YAML validation, and geometric baseline reporting
- Phase 3: first simplified stiff-string mechanical baseline with discretization, buoyancy, bending stiffness, and curvature loading
- Phase 4: first local lateral-equilibrium baseline with contact iteration, standoff estimate, and normal reaction estimate
- Phase 5: reduced global stiff-string-like baseline with coupled lateral displacement, global contact iteration, and centralizer spring support
- Phase 6: reduced torque & drag layer with run-in/pull-out hookloads and reduced surface torque
- Phase 7: first iterative coupling between the reduced global lateral/contact solve and the reduced torque & drag layer
- Phase 8: vector local-frame structural/contact baseline with two transverse DOFs per node and vector normal reactions
- Phase 9: detailed bow-spring centralizer geometry, bow-by-bow nonlinear reduced forces, vector bow resultants, and centralizer-aware reduced torque contributions
- Phase 10: benchmark suite, reduced calibration utilities, output traceability, and validation-oriented regression checks

See [docs/roadmap.md](/c:/Users/heitor.matos/Downloads/CS_complete_v214-20260303T125845Z-1-001/centralization-td/docs/roadmap.md) for the detailed phase breakdown.

## Devcontainer

Open the repository in VS Code and choose `Reopen in Container`. The devcontainer installs `clang`, `cmake`, `ninja`, `gdb`, `lldb`, `python3-dev`, `pip`, and `git`, then installs the Python package in editable mode.

## Build

Configure and build the C++ targets from the repository root:

```bash
cmake -S . -B build -G Ninja -DCENTRALTD_BUILD_TESTS=ON
cmake --build build
```

This builds the `centraltd_core` library, the pybind11 module target, and the C++ smoke test.

## Python Package

Install the Python package in editable mode from the repository root:

```bash
python -m pip install -e python[dev]
```

The package uses `scikit-build-core` and the top-level CMake project to compile the extension module when build dependencies are available.

## CLI

Print a case summary:

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase9_stub.json
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

## Tests

Run the C++ smoke test:

```bash
ctest --test-dir build --output-on-failure
```

Run the Python tests:

```bash
pytest
```

## What Is Still Not Implemented

- fuller 3D stiff-string beam equilibrium in the annulus
- 6-DOF spatial beam kinematics with rotations/torsion
- more robust nonlinear contact and wall reaction iteration
- real side-force prediction with fuller friction coupling
- full torque and drag with stronger axial/rotational coupling
- fuller bow-by-bow tangential friction and torque resolution
- external literature benchmark alignment and manufacturer-data calibration
- quantified uncertainty/error bars for the reduced benchmark suite
- optimizer
