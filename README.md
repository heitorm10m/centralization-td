# centraltd

Phase 2 baseline for a scientific project focused on casing centralization and torque & drag, using a hybrid C++ + Python architecture. The repository now includes validated trajectory geometry utilities, richer string and centralizer data models, a geometric baseline report, pybind11 bindings, YAML-driven inputs, CLI workflows, and executable tests. It still does not implement the full stiff-string, contact, or torque and drag solver.

## Objective

Build a clean base for future phases where the numerical kernel remains in C++, while Python handles orchestration, data loading, plotting, reporting, and user-facing workflows.

## Architecture

- `cpp/`: C++20 numerical core, data structures, solver stub, pybind11 module, and C++ smoke test.
- `python/`: Python package, YAML I/O, CLI, orchestration, optional plotting helpers, and packaging metadata.
- `configs/`: Reusable YAML fragments for wells, strings, and centralizers.
- `examples/`: Case manifests that reference reusable config fragments.
- `docs/`: Roadmap and engineering notes.
- `.agents/skills/`: Local skills describing repository bootstrap, physics modeling, verification, benchmarks, and reporting conventions.

## Roadmap By Phase

- Phase 1: Scaffold, environment, build, bindings, YAML schema, CLI, smoke tests, and docs.
- Phase 2: Trajectory geometry utilities, richer tubular and centralizer data, YAML validation, geometric baseline reporting, and richer verification.
- Phase 3: Torque and drag calculations, bow-spring detail, calibration workflows, and benchmark suite growth.
- Phase 4: Optimization, reporting automation, and scenario management for engineering studies.

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

Run the Phase 2 baseline and write JSON output:

```bash
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase2_stub.json
```

## Tests

Run the C++ smoke test:

```bash
ctest --test-dir build --output-on-failure
```

Run the Python smoke tests:

```bash
pytest
```

## Current Scope

The current baseline implements:

- trajectory validation from MD, inclination, and azimuth
- approximate trajectory geometry by balanced-tangent integration
- interpolation and discrete curvature utilities along MD
- richer string section properties and aggregate summaries
- richer centralizer specs with count hints or explicit installation MDs
- geometric placeholder metrics for clearance, curvature risk, and survey complexity

Important limitations:

- trajectory coordinates are approximated from survey angles and are not a survey-processing reference implementation
- nominal radial clearance is geometric only and is not real contact or standoff
- placeholder torque output is geometry-derived only and is not torque and drag physics

Explicit Phase 3 TODOs:

- stiff-string
- contact
- side force real
- standoff real
- torque and drag real
- optimizer

Those outputs are scaffolding aids, not validated engineering predictions.
