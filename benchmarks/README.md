# Benchmarks

This directory stores the canonical benchmark and calibration assets for the current reduced solver phase.

## Current State

- `cases/` contains repeatable YAML manifests for vertical, deviated, symmetric-centralizer, symmetry-breaking, constitutive/friction sensitivity, reduced torque-ratio sensitivity, and reduced tangential-feedback sensitivity studies.
- `configs/` contains reusable well, string, and centralizer fragments referenced by the case manifests.
- `suites/phase10_validation.yaml` defines the current benchmark suite, including cross-case validation rules and broad order-of-magnitude bounds.
- `calibration/` contains example inputs for reduced bow-spring `k/p` fitting.
- The current suite validates internal consistency and reduced-order trends only; it is not yet an external literature benchmark pack.
- The current suite now also checks body-vs-centralizer torque partition consistency, reduced tangential-friction trends driven by the bow resultant, sensitivity to explicit reduced centralizer torque-force ratios, activation of the reduced axial-tangential friction budget, monotonic reduced torsional-load/twist responses, and the new shared reduced local tangential-state mobilization driven by the carried reduced torsional state for both pipe body and centralizer.

## Commands

```bash
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

