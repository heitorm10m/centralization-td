# Python Package

The `centraltd` package provides the YAML loader, the Phase 9 reduced vector local-frame mechanical + torque and drag baseline, detailed bow-spring centralizer post-processing, Phase 10 benchmark/calibration utilities, CLI orchestration, optional plotting utilities, and access to the C++ core through pybind11 when the extension is built.

## Install

```bash
python -m pip install -e python[dev]
```

## Commands

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase9_stub.json
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

`run-stub` now also reports convergence traceability and resolved centralizer parameters. `benchmark-suite` writes a suite summary plus per-case JSON payloads, and `calibrate-bow-spring` writes a reduced `k/p` fit result that can be transferred into centralizer YAMLs.

If the compiled extension is unavailable or does not expose the current Phase 10 API surface, the package falls back to a deterministic Python implementation of the same reduced vector local-frame lateral/contact, bow-spring, benchmark, calibration, and reduced torque/drag baseline so the workflow remains executable.
