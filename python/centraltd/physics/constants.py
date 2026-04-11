"""
Numerical and physical constants for the bow-spring centralizer model.

All constants reference Dao et al. 2023 where applicable.
Do not change these values without updating DECISIONS.md and
verifying against the paper.
"""

import numpy as np

# Floating point precision for all numerical arrays
# Use this dtype everywhere instead of letting numpy choose
DTYPE = np.float64

# Numerical guard to avoid division by zero in friction velocity split
# Reference: Dao et al. 2023, Eq. 19
# Value stated explicitly in paper: 1e-7 m/s
V_EPSILON_G: float = 1e-7  # m/s

# Smoothing parameter for arctan friction transition
# Reference: Dao et al. 2023, Eq. 20
# Value stated explicitly in paper: 1 mm/s
V_LIM_G: float = 1e-3  # m/s

# Placeholder for smooth Heaviside tolerance parameter
# Reference: Dao et al. 2023, Eq. 17
# ⚠️ NOT SPECIFIED IN PAPER: λ value is not given explicitly.
# Must be calibrated or set as a solver parameter.
# Do not hardcode without documenting the chosen value here.
LAMBDA_CONTACT: float | None = None  # m — to be specified
