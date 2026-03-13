---
name: reporting-docs
description: Write or update technical documentation for the project roadmap, usage, and reporting outputs. Use when creating READMEs, roadmap notes, reporting conventions, or contributor guidance for the hybrid scientific stack.
---

# Reporting Docs

## Goal

Keep project documentation aligned with the implemented repository state and explicit about maturity, assumptions, next phases, and long-term product staging.

## Workflow

- Document architecture, commands, and current limitations in plain technical language.
- Distinguish implemented behavior from planned behavior.
- Keep near-term physics roadmap and long-term product roadmap separate from currently implemented capabilities.
- Keep internal benchmark/calibration evidence clearly separate from external quantitative validation claims.
- When mentioning future commercial-style features, state that they come after sufficient physics stability and validation/calibration maturity.
- Keep roadmaps phased and tied to concrete deliverables.
- Explain how to reproduce examples, tests, and generated outputs.
- Update docs whenever interfaces, file formats, or solver contracts change.

## Done Criteria

- A new contributor can build the code and run the example workflow from the docs.
- Phase boundaries and TODOs are visible.
- Documentation does not imply that placeholder outputs are validated physics.

