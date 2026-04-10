# centraltd

Phase 14 reduced local tangential-state consolidation for a scientific project focused on casing centralization and torque & drag, using a Python + NumPy architecture. The repository keeps the reduced vector local-frame solver, the Phase 10 benchmark/calibration infrastructure, and now carries a shared reduced local tangential-state layer on top of the bow-resultant, body-vs-centralizer torque partition, and reduced torsional-load/twist baseline.

It still does not implement a commercial stiff-string solver, a full 6-DOF beam/contact formulation, or fully nonlinear 3D contact/friction torque and drag.

## Objective

Build the project in phases with a Python numerical kernel, using optimized numerical libraries such as NumPy for dense linear algebra while keeping YAML I/O, CLI, plotting, and reporting in Python.

## Architecture

- `python/`: Python package for YAML loading, validation, NumPy-backed mechanics, CLI orchestration, calibration, benchmarking, plotting helpers, and packaging.
- `configs/`: Reusable YAML fragments for wells, strings, and centralizers.
- `examples/`: Case manifests that reference reusable config fragments.
- `docs/`: Roadmap and engineering notes.

## Current Phase

Phase 14 currently implements the Phase 10 infrastructure plus:

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
- explicit body-vs-centralizer torque partition in JSON/CLI outputs
- contact-informed reduced centralizer tangential law that blends the local bow-resultant radial direction with the local contact direction before rotating by 90 degrees in the local `n-b` plane
- explicit body torque profile, centralizer torque profile, body axial-friction profile, centralizer axial-friction profile, and reduced tangential-friction vector profile
- reduced axial-tangential friction-budget coupling inside the centralizer torque model, so tangential demand can reduce the remaining axial centralizer friction capacity
- reduced torsional-load accumulation plus a GJ-based twist indicator profile built from body + centralizer torque contributions
- carried reduced torsional state now fed into a shared reduced local tangential-state layer, with `slip_indicator`, bounded mobilization, traction-indicator proxy, and reduced regime classification before the body and centralizer tangential laws are applied
- iterative coupling between the selected axial profile, the vector lateral/contact solve, and the reduced T&D/torsional post-processing, with convergence checked on axial-profile, torque-profile, and carried torsional-load updates
- hookload estimates for run in and pull out, plus a reduced surface-torque estimate
- canonical benchmark YAML cases for vertical, deviated, symmetric-centralizer, symmetry-breaking, and constitutive/friction sensitivity studies
- a benchmark-suite runner with case checks, cross-case comparisons, and JSON summaries
- reduced bow-spring calibration utilities from force-deflection pairs or reduced nominal force points
- output traceability for resolved centralizer parameters, convergence criteria, final coupling update magnitudes, and per-placement centralizer torque breakdown

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
- quantitative external validation strong enough to claim equivalence with the full literature class or commercial software

## Hypotheses And Limitations

The Phase 14 baseline is intentionally limited:

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
- centralizer tangential friction uses a reduced local tangential direction `t_hat = rot90(r_eff)` in the local `n-b` plane, where `r_eff` blends the bow-resultant radial direction with the local contact direction
- centralizer tangential magnitude uses the projected contact-normal proxy `N_proj = max(0, R_bow . r_eff)` rather than only `|R_bow|`
- centralizer axial friction and tangential torque each use an explicit reduced force ratio when provided, or a shared running/restoring-force-ratio fallback when not yet calibrated separately
- reduced centralizer axial and tangential demands share a combined friction-budget cap, so the model can represent first-order competition between axial drag and tangential torque without a full 3D friction law
- the carried reduced torsional state now feeds a shared reduced local tangential-state layer with `slip_indicator = |twist_rate| r`, `mobilization = slip_indicator / (1 + slip_indicator)`, and a bounded traction-indicator proxy before the body and centralizer tangential laws are applied; pipe-body axial drag still remains reduced `mu * N_body`
- centralizer torque remains reduced and is accumulated as the sum of local tangential-force magnitudes times an effective contact radius
- torque remains reduced and does not yet use full tangential vector friction/contact mechanics or a bow-by-bow dynamic/contact solve; it now propagates through a reduced torsional-load/twist state, but still not through a full torsional structural solve

Use these outputs as structured engineering scaffolding with traceable validation checks, not as final design predictions.

## Roadmap By Phase

- Phase 1: scaffold, environment, Python package, YAML schema, CLI, smoke tests, and docs
- Phase 2: trajectory geometry utilities, richer tubular and centralizer data, YAML validation, and geometric baseline reporting
- Phase 3: first simplified stiff-string mechanical baseline with discretization, buoyancy, bending stiffness, and curvature loading
- Phase 4: first local lateral-equilibrium baseline with contact iteration, standoff estimate, and normal reaction estimate
- Phase 5: reduced global stiff-string-like baseline with coupled lateral displacement, global contact iteration, and centralizer spring support
- Phase 6: reduced torque & drag layer with run-in/pull-out hookloads and reduced surface torque
- Phase 7: first iterative coupling between the reduced global lateral/contact solve and the reduced torque & drag layer
- Phase 8: vector local-frame structural/contact baseline with two transverse DOFs per node and vector normal reactions
- Phase 9: detailed bow-spring centralizer geometry, bow-by-bow nonlinear reduced forces, vector bow resultants, and centralizer-aware reduced torque contributions
- Phase 10: benchmark suite, reduced calibration utilities, output traceability, and validation-oriented regression checks
- Phase 11: reduced vector centralizer torque from bow resultant plus local contact direction, explicit body-vs-centralizer torque partition, and a carried reduced torsional-load/twist state
- Phase 12: strengthened contact-informed centralizer tangential law and reduced torque feedback into the coupling loop
- Phase 13: torsional reduced state fed back into both centralizer and body tangential laws
- Phase 14: shared reduced local tangential-state layer with mobilization, traction-indicator proxy, regime traceability, and separated body-vs-centralizer local tangential outputs

See [docs/roadmap.md](docs/roadmap.md) for the detailed phase breakdown.

## Devcontainer

Open the repository in VS Code and choose `Reopen in Container`. The devcontainer installs Python, pip, Git, and the editable Python package with development dependencies.

## Python Package

Install the Python package in editable mode from the repository root:

```bash
python -m pip install -e python[dev]
```

The package uses a pure-Python packaging path with NumPy-backed numerical routines; no native project build step is required.

## CLI

Print a case summary:

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase14_stub.json
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

## Tests

Run the Python test suite:

```bash
pytest
```

## What Is Still Not Implemented

- fuller 3D stiff-string beam equilibrium in the annulus
- 6-DOF spatial beam kinematics with rotations/torsion
- more robust nonlinear contact and wall reaction iteration
- real side-force prediction with fuller friction coupling
- full torque and drag with stronger axial/rotational coupling
- fuller bow-by-bow tangential friction/contact resolution with stronger torsional feedback
- external literature benchmark alignment and manufacturer-data calibration
- quantified uncertainty/error bars for the reduced benchmark suite
- optimizer
