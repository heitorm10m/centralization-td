# Python Package

The `centraltd` package provides the YAML loader, CLI, orchestration helpers, optional plotting utilities, and access to the C++ stub through pybind11 when the extension is built.

## Install

```bash
python -m pip install -e python[dev]
```

## Commands

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_stub.json
```

If the compiled extension is unavailable, the package falls back to a deterministic Python implementation of the same placeholder formulas so the Phase 1 workflow remains executable.

