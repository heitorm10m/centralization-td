# Benchmarks

This directory stores the canonical benchmark and calibration assets for the current reduced solver phase.

## Current State

- `cases/` contains repeatable YAML manifests for vertical, deviated, symmetric-centralizer, symmetry-breaking, and constitutive/friction sensitivity studies.
- `configs/` contains reusable well, string, and centralizer fragments referenced by the case manifests.
- `suites/phase10_validation.yaml` defines the current benchmark suite, including cross-case validation rules and broad order-of-magnitude bounds.
- `calibration/` contains example inputs for reduced bow-spring `k/p` fitting.
- The current suite validates internal consistency and reduced-order trends only; it is not yet an external literature benchmark pack.

## Commands

```bash
centraltd benchmark-suite benchmarks/suites/phase10_validation.yaml --output-dir build/benchmarks/phase10
centraltd calibrate-bow-spring benchmarks/calibration/force_deflection_pairs.yaml --output build/calibration/force_pairs.json
```

