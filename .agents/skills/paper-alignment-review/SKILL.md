---
name: paper-alignment-review
description: Use when the task should be evaluated against the target literature direction of this repository: vector stiff-string progression, bow-by-bow centralizer modeling, and torque from vector resultants. Do not use for generic refactors with no physics impact.
---

# Goal

Review whether the implementation direction remains aligned with the target literature class and does not drift into misleading shortcuts.

# Rules

- Compare the current implementation goal against the repository roadmap.
- Compare the current implementation goal against the target sequence: local-frame vector solver -> bow-by-bow centralizers -> vector tangential torque -> contact/friction refinement -> calibration/validation -> commercial maturity.
- Explicitly state what is already aligned with the target literature direction.
- Explicitly state what is still reduced, missing, or only approximate.
- Treat internal benchmark/calibration evidence as coherence support, not as external validation.
- Do not claim equivalence to proprietary commercial software.
- Do not claim equivalence to a full literature model unless the implemented degrees of freedom, contact handling, and constitutive logic really support that claim.
- State the next technical step required to reduce the gap.

# Done criteria

- alignment summary written
- missing pieces identified
- claims kept honest
- roadmap-sequence progress or regression is clearly stated
- next technical step is clearly stated
