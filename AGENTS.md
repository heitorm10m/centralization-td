# AGENTS.md

- Keep the numerical core in C++.
- Keep interface, plotting, automation, and reporting in Python.
- Use SI units internally across the full stack.
- Do not hardcode well cases, strings, or centralizer layouts inside the solver.
- Require at least one automated test for every future solver change.
- Do not replace the planned stiff-string formulation with a soft-string formulation as the final solution.
- Document physical assumptions and exclusions explicitly in code, tests, or docs.
- Keep YAML inputs human-readable and version-controlled.
- Prefer deterministic placeholder behavior over hidden heuristics during scaffold phases.

