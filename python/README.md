# Python Package

The `centraltd` package provides the YAML loader, the Phase 9 reduced vector local-frame mechanical + torque and drag baseline, detailed bow-spring centralizer post-processing, CLI orchestration, optional plotting utilities, and access to the C++ core through pybind11 when the extension is built.

## Install

```bash
python -m pip install -e python[dev]
```

## Commands

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase9_stub.json
```

If the compiled extension is unavailable or does not expose the current Phase 9 API, the package falls back to a deterministic Python implementation of the same reduced vector local-frame lateral/contact, bow-spring, and reduced torque/drag baseline so the workflow remains executable.
