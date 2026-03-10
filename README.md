# centraltd

Phase 5 baseline for a scientific project focused on casing centralization and torque & drag, using a hybrid C++ + Python architecture. The repository now includes validated survey geometry utilities, richer string and centralizer data models, and a first reduced global stiff-string-like baseline with annular contact iteration and nominal centralizer support.

It still does not implement a full stiff-string solver or full torque and drag.

## Objective

Build the project in phases while keeping the numerical kernel in C++, and the orchestration, YAML I/O, CLI, plotting, and reporting in Python.

## Architecture

- `cpp/`: C++20 core for trajectory geometry, section properties, discretization, centralizer support, lateral equilibrium, contact model, mechanical baseline, pybind11 bindings, and C++ smoke tests.
- `python/`: Python package for YAML loading, validation, fallback mechanics, CLI orchestration, and packaging.
- `configs/`: Reusable YAML fragments for wells, strings, and centralizers.
- `examples/`: Case manifests that reference reusable config fragments.
- `docs/`: Roadmap and engineering notes.

## Current Phase

Phase 5 currently implements:

- trajectory validation from MD, inclination, and azimuth
- approximate trajectory geometry by balanced-tangent integration
- interpolation, local tangent, and discrete curvature utilities
- richer string section properties including elastic and buoyancy-related helpers
- richer centralizer specs with count hints or explicit installation MDs
- MD discretization into mechanical segments
- effective line weight from dry weight minus buoyancy
- bending stiffness from `E * I`
- simplified effective axial load integration from buoyant tangential weight
- reduced global scalar lateral equilibrium along MD with coupling between neighboring nodes
- bending contribution, axial-tension geometric stiffening, and lateral equivalent loading
- nominal centralizer centering/support effect as local spring support
- annular contact restriction by clearance with simple global active-set penalty iteration
- eccentricity, contact state, normal reaction, and standoff estimates from the global solution

## Hypotheses And Limitations

The Phase 4 baseline is intentionally limited:

- survey-derived coordinates remain approximate and are not a survey-processing reference implementation
- the column response is quasi-static and reduced to a scalar lateral displacement along MD
- the bending term still uses the equivalent simply supported beam relation `delta_max = 5 q L^4 / (384 E I)`, so the `384/5` factor is an explicit reduced structural hypothesis
- the solver is global and coupled along MD, but it is still not a full 3D stiff-string solve in the annulus
- contact is handled by a simple global active-set penalty iteration, not a full nonlinear wall-reaction model
- centralizers are represented only by nominal OD and nominal restoring support over an influence length
- `estimated_surface_torque_n_m` is intentionally `null` because torque and drag are not implemented yet

Use these outputs as structured engineering scaffolding, not as final design predictions.

## Roadmap By Phase

- Phase 1: scaffold, environment, build, bindings, YAML schema, CLI, smoke tests, and docs
- Phase 2: trajectory geometry utilities, richer tubular and centralizer data, YAML validation, and geometric baseline reporting
- Phase 3: first simplified stiff-string mechanical baseline with discretization, buoyancy, bending stiffness, and curvature loading
- Phase 4: first local lateral-equilibrium baseline with contact iteration, standoff estimate, and normal reaction estimate
- Phase 5: reduced global stiff-string-like baseline with coupled lateral displacement, global contact iteration, and centralizer spring support
- Phase 6: fuller contact/friction mechanics, benchmark growth, and design/optimization workflows

See [docs/roadmap.md](/c:/Users/heito/Downloads/centralization-td/docs/roadmap.md) for the detailed phase breakdown.

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
```

Run the Phase 5 global baseline and write JSON output:

```bash
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase5_stub.json
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

- fuller 3D stiff-string equilibrium in the annulus
- more robust nonlinear contact and wall reaction iteration
- real side-force prediction with friction coupling
- real torque and drag
- detailed bow-spring constitutive behavior
- optimizer
