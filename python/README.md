# Python Package

The `centraltd` package provides the YAML loader, Phase 2 geometry baseline, CLI, orchestration helpers, optional plotting utilities, and access to the C++ core through pybind11 when the extension is built.

## Install

```bash
python -m pip install -e python[dev]
```

## Commands

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_stub.json
```

If the compiled extension is unavailable or still on the older Phase 1 API, the package falls back to a deterministic Python implementation of the same Phase 2 geometric baseline so the workflow remains executable.
