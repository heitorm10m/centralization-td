"""
Physics property tests for Dao et al. 2023 implementation.

These tests verify physical correctness, not just input/output.
Each test references the specific equation it is testing.
Run with: pytest tests/test_physics_properties.py

Test data source: Dao et al. 2023, Table 1 and Table 2.
"""

import pytest
import numpy as np

from centraltd.physics.contact import smooth_heaviside


# ---------------------------------------------------------------------------
# Test data from paper Table 1 and Table 2
# Centralizer: Weatherford VariForm UR 4-1/2" x 7-1/4"
# ---------------------------------------------------------------------------

CENTRALIZER_PARAMS = {
    "n_blades": 5,
    "d_max_m": 0.189,        # 7.44 in
    "d_min_m": 0.1252,       # 4.93 in
    "d_int_m": 0.1162,       # 4.57 in
    "d_pipe_m": 0.1143,      # 4.5 in
    "api_restoring_force_n": 2065.5,  # 464 lbf = 210.467 kgf
}

CALIBRATED_PARAMS = {
    "d_max_used_m": 0.18669,  # 7.35 in
    "p": 1.17509,
    "k_blade": 154024.29,     # kgf/m^p
}

OPEN_HOLE_PARAMS = {
    "d_hole_m": 0.18415,      # 7.25 in
    "running_force_n": 600.1, # 61.18 kgf
    "restoring_force_67_n": 99797.0,  # 10.17 kN at 67% standoff
    "standoff_at_api_force": 0.93,    # 93%
}


# ---------------------------------------------------------------------------
# Smooth Heaviside — Eq. 17
# ---------------------------------------------------------------------------

class TestSmoothHeaviside:

    def test_zero_for_negative_input(self):
        """h(x) must be exactly 0 for all x <= 0. Eq. 17."""
        values = np.array([-1.0, -1e-6, 0.0])
        expected = np.zeros_like(values)
        actual = smooth_heaviside(values, 1e-3)
        assert np.array_equal(actual, expected)

    def test_half_at_lambda(self):
        """h(λ) must equal exactly 0.5. Eq. 17."""
        assert smooth_heaviside(1e-3, 1e-3) == pytest.approx(0.5)

    def test_approaches_one_for_large_input(self):
        """h(x) → 1 as x → ∞. Eq. 17."""
        assert smooth_heaviside(1e3, 1e-3) == pytest.approx(1.0, rel=0.0, abs=1e-12)

    def test_continuous_at_zero(self):
        """h(x) must be C1 continuous at x=0. Eq. 17."""
        epsilon = 1e-7
        lambda_contact = 1.0

        assert smooth_heaviside(0.0, lambda_contact) == pytest.approx(0.0)
        assert smooth_heaviside(-epsilon, lambda_contact) == pytest.approx(0.0)
        assert smooth_heaviside(epsilon, lambda_contact) == pytest.approx(0.0, abs=1e-12)

        left_derivative = (
            smooth_heaviside(0.0, lambda_contact) - smooth_heaviside(-epsilon, lambda_contact)
        ) / epsilon
        right_derivative = (
            smooth_heaviside(epsilon, lambda_contact) - smooth_heaviside(0.0, lambda_contact)
        ) / epsilon

        assert left_derivative == pytest.approx(0.0, abs=1e-12)
        assert right_derivative == pytest.approx(0.0, abs=1e-5)


# ---------------------------------------------------------------------------
# Blade deflection geometry — Eq. 5
# ---------------------------------------------------------------------------

class TestBladeDeflection:

    def test_zero_deflection_when_centered(self):
        """When pipe is centered, all blades have same deflection. Eq. 5."""
        pytest.skip("Implement when Eq. 5 geometry is in bow_spring.py")

    def test_top_blade_loses_contact_in_horizontal(self):
        """In horizontal section, top blade deflection can reach zero. Eq. 5."""
        pytest.skip("Implement when Eq. 5 geometry is in bow_spring.py")

    def test_deflection_non_negative(self):
        """Blade deflection must never be negative (max with 0). Eq. 5."""
        pytest.skip("Implement when Eq. 5 geometry is in bow_spring.py")


# ---------------------------------------------------------------------------
# Blade force law — Eq. 4
# ---------------------------------------------------------------------------

class TestBladeForceLaw:

    def test_linear_when_p_equals_one(self):
        """When p=1, force is linear in deflection. Eq. 4."""
        pytest.skip("Implement when Eq. 4 is in bow_spring.py")

    def test_zero_force_zero_deflection(self):
        """Zero deflection must produce zero force. Eq. 4."""
        pytest.skip("Implement when Eq. 4 is in bow_spring.py")

    def test_nonlinear_greater_stiffness_at_high_deflection(self):
        """For p>1, force increases faster at high deflection. Eq. 4."""
        pytest.skip("Implement when Eq. 4 is in bow_spring.py")


# ---------------------------------------------------------------------------
# Friction summation — Eqs. 25 and 26
# ---------------------------------------------------------------------------

class TestFrictionSummation:

    def test_axial_friction_is_scalar_sum(self):
        """
        Axial friction must be μz times the SCALAR sum of absolute
        blade forces. Not a vector norm. Eq. 25.
        Critical: confusing this with Eq. 26 is a known failure mode.
        """
        pytest.skip("Implement when Eq. 25 is in bow_spring.py")

    def test_tangential_friction_uses_vector_resultant(self):
        """
        Tangential friction must use the VECTOR sum of blade force
        vectors, not the scalar sum of magnitudes. Eq. 26.
        """
        pytest.skip("Implement when Eq. 26 is in bow_spring.py")

    def test_scalar_sum_geq_vector_norm(self):
        """
        The scalar sum Σ|fi| >= ||Σfi|| always. This is a sanity check
        that the two summations are implemented differently. Eqs. 25-26.
        """
        pytest.skip("Implement when both Eqs. 25 and 26 are in bow_spring.py")


# ---------------------------------------------------------------------------
# Velocity-dependent friction — Eq. 20
# ---------------------------------------------------------------------------

class TestVelocityFriction:

    def test_friction_zero_at_zero_velocity(self):
        """μ(0) = 0 because arctan(0) = 0. Eq. 20."""
        pytest.skip("Implement when Eq. 20 is in friction.py")

    def test_friction_approaches_dynamic_at_high_velocity(self):
        """μ(Vg→∞) → μd. The exp term vanishes. Eq. 20."""
        pytest.skip("Implement when Eq. 20 is in friction.py")

    def test_static_greater_than_dynamic(self):
        """μs > μd must always hold. Eq. 20."""
        pytest.skip("Implement when Eq. 20 is in friction.py")


# ---------------------------------------------------------------------------
# Calibration — Eq. 9
# ---------------------------------------------------------------------------

class TestCalibration:

    def test_calibrated_values_match_paper(self):
        """
        Using Table 1 and Table 2 inputs with Dmax=7.35 in,
        calibration must produce:
          p = 1.17509
          k_blade = 154024.29 kgf/m^p
        Reference: Dao et al. 2023, Section 4.1 and Fig. 12.
        """
        pytest.skip("Implement when two-step API 10D calibration exists")

    def test_blade_rigidity_from_67_percent_standoff(self):
        """
        Step 1 of calibration: kblade computed from restoring force
        at 67% standoff using Eq. 9. Result must be on the curve.
        """
        pytest.skip("Implement when Eq. 9 is in calibration.py")
