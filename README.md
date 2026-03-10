# centraltd

Phase 1 scaffold for a scientific project focused on casing centralization and torque & drag, using a hybrid C++ + Python architecture. This stage provides the repository structure, development environment, build system, pybind11 bindings, YAML-driven inputs, a Python CLI, examples, and smoke tests. It intentionally does not implement the full physical solver.

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
- Phase 2: Stiff-string geometry, contact detection, basic standoff estimation, and richer verification.
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

Run the solver stub and write JSON output:

```bash
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_stub.json
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

The Phase 1 solver is a deterministic stub. It validates basic inputs and emits placeholder outputs plus explicit TODOs for:

- stiff-string
- contact
- standoff
- bow-spring detailed model
- torque and drag
- optimizer

Those outputs are scaffolding aids, not validated engineering predictions.

