# Python Package

The `centraltd` package provides the YAML loader, the Phase 14 reduced vector local-frame mechanical + torque and drag baseline, detailed bow-spring centralizer post-processing, the contact-informed reduced centralizer tangential law, the reduced torsional-load/twist layer carried by the coupling loop, the shared reduced local tangential-state layer used by both the local centralizer tangential demand and the local pipe-body tangential torque term, Phase 10 benchmark/calibration utilities, CLI orchestration, optional plotting utilities, and access to the C++ core through pybind11 when the extension is built.

## Install

```bash
python -m pip install -e python[dev]
```

## Commands

```bash
centraltd summary examples/minimal_case.yaml
centraltd run-stub examples/minimal_case.yaml --output examples/minimal_case_phase14_stub.json
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

`run-stub` now also reports convergence traceability, torque partitioning between pipe body and centralizers, reduced tangential-direction/vector profiles, `local_tangential_interaction_state`, `local_tangential_state`, `local_tangential_mobilization_profile`, `local_body_tangential_interaction_state`, `local_centralizer_tangential_interaction_state`, reduced torsional-load/twist profiles, resolved centralizer parameters, and per-placement centralizer torque breakdown. `benchmark-suite` writes a suite summary plus per-case JSON payloads, and `calibrate-bow-spring` writes a reduced `k/p` fit result that can be transferred into centralizer YAMLs.

If the compiled extension is unavailable or does not expose the current Phase 14 API surface, the package falls back to a deterministic Python implementation of the same reduced vector local-frame lateral/contact, bow-spring, benchmark, calibration, reduced torque/drag baseline, and reduced torsional state, including the body-vs-centralizer partition, the contact-informed tangential law, and the shared reduced local tangential-state layer used by both body and centralizer, so the workflow remains executable.
