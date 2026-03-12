---
name: torque-drag-coupling
description: Use when modifying axial force propagation, torque integration, friction coupling, operational modes, or iterative coupling between contact and torque-drag. Do not use for geometry-only work.
---

# Goal

Keep torque and drag logic physically consistent, modular, and honest about what is reduced versus fully implemented.

# Rules

- Document the physical meaning and units of every normal-force input used by friction logic.
- Document sign conventions for run-in, pull-out, pickup, slackoff, and rotation.
- Separate axial propagation, torque integration, and coupling driver.
- Keep axial friction, tangential friction, and torque contributions distinguishable, especially for centralizer-related proxies.
- If a quantity is not yet physically defensible, prefer null or explicit status over a guessed number.
- Keep body contact contributions and centralizer contributions distinguishable.
- Update tests whenever signs, profiles, or operation modes change.

# Done criteria

- signs are documented
- units are documented
- build/tests/CLI pass
- output JSON remains consistent
- docs explain what remains reduced
