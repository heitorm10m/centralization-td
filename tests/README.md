# Tests

This folder will contain unit tests and integration tests for the 
physics implementation based on Dao et al. 2023.

## Test Cases to Be Implemented (in order)

1. Smooth Heaviside h(x): verify h(λ) = 0.5 and h(x) = 0 for x ≤ 0
2. Blade deflection geometry (Eq. 5): verify against paper Table 1 values
3. Blade force law (Eq. 4): verify linear case p=1 and nonlinear p>1
4. Blade loses contact independently: force = 0 when Δrb,i ≤ 0
5. Axial friction scalar sum (Eq. 25): verify sum is scalar not vector
6. Tangential friction vector resultant (Eq. 26): verify vector sum
7. Calibration output (Eq. 9): verify p=1.17509 and 
   kblade=154024.29 kgf/m^p for Dmax=7.35 in (paper Table 1/2)
8. Velocity-dependent friction (Eq. 20): verify μ(0)=0 and μ→μd at high speed
