"""
Physics property tests for Dao et al. 2023 implementation.

These tests verify physical correctness, not just input/output.
Each test references the specific equation it is testing.
Run with: pytest tests/test_physics_properties.py

Test data source: Dao et al. 2023, Table 1 and Table 2.
"""

import pytest
import numpy as np

from centraltd.physics.bow_spring import (
    BowSpringContactModel,
    assemble_bow_spring_contact_force_and_jacobian,
    blade_deflection_geometry_m,
    blade_force_power_law_n,
    bow_contact_geometry,
    bow_contact_penetration_derivatives,
)
from centraltd.physics.contact import (
    BodyContactModel,
    assemble_body_contact_force_and_jacobian,
    smooth_heaviside,
)
from centraltd.solver.mechanics import solve_6dof_newton_raphson


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


def _finite_difference_jacobian(
    force_function,
    state_vector: np.ndarray,
    *,
    active_columns: tuple[int, ...] = (0, 1),
    step: float = 1.0e-7,
) -> np.ndarray:
    state_vector = np.asarray(state_vector, dtype=float)
    baseline_force = np.asarray(force_function(state_vector), dtype=float)
    jacobian = np.zeros((baseline_force.size, state_vector.size), dtype=float)

    for column_index in active_columns:
        perturbation = np.zeros_like(state_vector, dtype=float)
        perturbation[column_index] = step
        forward_force = np.asarray(force_function(state_vector + perturbation), dtype=float)
        backward_force = np.asarray(force_function(state_vector - perturbation), dtype=float)
        jacobian[:, column_index] = (forward_force - backward_force) / (2.0 * step)

    return jacobian


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
        ur_cent = np.array([0.0, 0.0])
        d_max = CENTRALIZER_PARAMS["d_max_m"]
        d_hole = OPEN_HOLE_PARAMS["d_hole_m"]
        angles = np.linspace(0.0, 2.0 * np.pi, num=5, endpoint=False)

        deflections = [
            blade_deflection_geometry_m(ur_cent, d_max, d_hole, angle) for angle in angles
        ]

        assert deflections == pytest.approx([deflections[0]] * len(deflections))
        assert deflections[0] == pytest.approx(max((d_max - d_hole) / 2.0, 0.0))

    def test_top_blade_loses_contact_in_horizontal(self):
        """In horizontal section, top blade deflection can reach zero. Eq. 5."""
        ur_cent = np.array([0.003, 0.0])
        d_max = CENTRALIZER_PARAMS["d_max_m"]
        d_hole = OPEN_HOLE_PARAMS["d_hole_m"]

        bottom_blade = blade_deflection_geometry_m(ur_cent, d_max, d_hole, 0.0)
        top_blade = blade_deflection_geometry_m(ur_cent, d_max, d_hole, np.pi)

        assert bottom_blade > 0.0
        assert top_blade == 0.0

    def test_deflection_non_negative(self):
        """Blade deflection must never be negative (max with 0). Eq. 5."""
        ur_cent = np.array([0.02, 0.0])
        d_max = CENTRALIZER_PARAMS["d_max_m"]
        d_hole = OPEN_HOLE_PARAMS["d_hole_m"]

        assert blade_deflection_geometry_m(ur_cent, d_max, d_hole, np.pi) == 0.0


# ---------------------------------------------------------------------------
# Blade force law — Eq. 4
# ---------------------------------------------------------------------------

class TestBladeForceLaw:

    def test_linear_when_p_equals_one(self):
        """When p=1, force is linear in deflection. Eq. 4."""
        force = blade_force_power_law_n(0.01, 1000.0, 1.0)
        assert force == pytest.approx(10.0)

    def test_zero_force_zero_deflection(self):
        """Zero deflection must produce zero force. Eq. 4."""
        assert blade_force_power_law_n(0.0, 1000.0, 1.17509) == 0.0

    def test_nonlinear_greater_stiffness_at_high_deflection(self):
        """For p>1, force increases faster at high deflection. Eq. 4."""
        force_low = blade_force_power_law_n(0.005, 1000.0, 2.0)
        force_high = blade_force_power_law_n(0.01, 1000.0, 2.0)
        assert force_high > (2.0 * force_low)


# ---------------------------------------------------------------------------
# Newton-Raphson solver — Eq. 31
# ---------------------------------------------------------------------------

class TestNewtonRaphsonSolver:

    def test_zero_load_zero_displacement(self):
        """With zero load and no contact, the converged state must remain zero. Eq. 31."""
        stiffness_matrix = np.eye(6)
        constant_force_vector = np.zeros(6)

        result = solve_6dof_newton_raphson(
            stiffness_matrix,
            constant_force_vector,
        )

        assert result.converged is True
        assert result.iteration_count == 0
        assert np.allclose(result.state_vector, 0.0)
        assert np.allclose(result.residual_vector, 0.0)

    def test_converges_on_trivial_no_contact_linear_case(self):
        """Without contact, Newton-Raphson must recover the linear solution in one update. Eq. 31."""
        stiffness_diagonal = np.array([10.0, 12.0, 15.0, 20.0, 25.0, 30.0])
        stiffness_matrix = np.diag(stiffness_diagonal)
        constant_force_vector = np.array([5.0, -6.0, 7.5, 8.0, -10.0, 12.0])

        result = solve_6dof_newton_raphson(
            stiffness_matrix,
            constant_force_vector,
        )

        assert result.converged is True
        assert result.iteration_count <= 1
        assert result.state_vector == pytest.approx(constant_force_vector / stiffness_diagonal)
        assert result.residual_norm == pytest.approx(0.0, abs=1.0e-12)


# ---------------------------------------------------------------------------
# Analytical Jacobians — Eq. 35 and Eq. 36
# ---------------------------------------------------------------------------

class TestAnalyticalJacobians:

    def test_body_contact_jacobian_matches_finite_difference(self):
        """Analytical body-contact Jacobian must match finite differences for active contact. Eq. 35."""
        rng = np.random.default_rng(12345)
        contact_model = BodyContactModel(
            node_index=0,
            outer_radius_m=0.100,
            wellbore_radius_m=0.110,
            contact_stiffness_n_per_m=2.5e6,
            friction_coefficient_theta=0.18,
            friction_coefficient_z=0.27,
            effective_contact_radius_m=0.100,
            lambda_contact=1.0e-3,
        )

        for _ in range(3):
            while True:
                trial_state = np.zeros(6)
                trial_state[0] = rng.uniform(0.012, 0.035)
                trial_state[1] = rng.uniform(0.008, 0.028)
                if np.hypot(trial_state[0], trial_state[1]) + contact_model.outer_radius_m > contact_model.wellbore_radius_m:
                    break

            analytical_force, analytical_jacobian = assemble_body_contact_force_and_jacobian(
                [contact_model],
                trial_state,
            )
            numerical_jacobian = _finite_difference_jacobian(
                lambda state: assemble_body_contact_force_and_jacobian([contact_model], state)[0],
                trial_state,
            )

            rows = np.array([0, 1, 2, 5])
            columns = np.array([0, 1])
            np.testing.assert_allclose(
                analytical_force,
                assemble_body_contact_force_and_jacobian([contact_model], trial_state)[0],
            )
            np.testing.assert_allclose(
                analytical_jacobian[np.ix_(rows, columns)],
                numerical_jacobian[np.ix_(rows, columns)],
                rtol=1.0e-6,
                atol=1.0e-8,
            )

    def test_bow_contact_penetration_derivatives_match_finite_difference(self):
        """The Eq. 36 derivatives of Δrb,i must match finite differences."""
        state_vector = np.array([0.024, 0.019, 0.0, 0.0, 0.0, 0.0])
        model = BowSpringContactModel(
            node_index=0,
            alpha_b_rad=0.63,
            radial_clearance_m=0.005,
            bow_radius_m=0.090,
            wellbore_radius_m=0.105,
            blade_stiffness_n=1.8e5,
            blade_power_p=1.35,
            delta_rb_max_m=0.002,
            contact_stiffness_n_per_m=4.0e6,
            friction_coefficient_theta=0.15,
            friction_coefficient_z=0.22,
            effective_contact_radius_m=0.090,
            lambda_contact=1.0e-3,
        )

        analytical_du, analytical_dv = bow_contact_penetration_derivatives(
            u_m=state_vector[0],
            v_m=state_vector[1],
            radial_clearance_m=model.radial_clearance_m,
            bow_radius_m=model.bow_radius_m,
            wellbore_radius_m=model.wellbore_radius_m,
            alpha_b_rad=model.alpha_b_rad,
            delta_rb_max_m=model.delta_rb_max_m,
        )

        step = 1.0e-7
        forward_u = bow_contact_geometry(
            u_m=state_vector[0] + step,
            v_m=state_vector[1],
            radial_clearance_m=model.radial_clearance_m,
            bow_radius_m=model.bow_radius_m,
            wellbore_radius_m=model.wellbore_radius_m,
            alpha_b_rad=model.alpha_b_rad,
            delta_rb_max_m=model.delta_rb_max_m,
        )["delta_rb_m"]
        backward_u = bow_contact_geometry(
            u_m=state_vector[0] - step,
            v_m=state_vector[1],
            radial_clearance_m=model.radial_clearance_m,
            bow_radius_m=model.bow_radius_m,
            wellbore_radius_m=model.wellbore_radius_m,
            alpha_b_rad=model.alpha_b_rad,
            delta_rb_max_m=model.delta_rb_max_m,
        )["delta_rb_m"]
        forward_v = bow_contact_geometry(
            u_m=state_vector[0],
            v_m=state_vector[1] + step,
            radial_clearance_m=model.radial_clearance_m,
            bow_radius_m=model.bow_radius_m,
            wellbore_radius_m=model.wellbore_radius_m,
            alpha_b_rad=model.alpha_b_rad,
            delta_rb_max_m=model.delta_rb_max_m,
        )["delta_rb_m"]
        backward_v = bow_contact_geometry(
            u_m=state_vector[0],
            v_m=state_vector[1] - step,
            radial_clearance_m=model.radial_clearance_m,
            bow_radius_m=model.bow_radius_m,
            wellbore_radius_m=model.wellbore_radius_m,
            alpha_b_rad=model.alpha_b_rad,
            delta_rb_max_m=model.delta_rb_max_m,
        )["delta_rb_m"]

        numerical_du = (forward_u - backward_u) / (2.0 * step)
        numerical_dv = (forward_v - backward_v) / (2.0 * step)

        assert analytical_du == pytest.approx(numerical_du, rel=1.0e-6, abs=1.0e-8)
        assert analytical_dv == pytest.approx(numerical_dv, rel=1.0e-6, abs=1.0e-8)

    def test_bow_contact_jacobian_matches_finite_difference(self):
        """Analytical bow-contact Jacobian must match finite differences for active smoothed contact. Eq. 36."""
        rng = np.random.default_rng(6789)
        contact_model = BowSpringContactModel(
            node_index=0,
            alpha_b_rad=0.71,
            radial_clearance_m=0.005,
            bow_radius_m=0.090,
            wellbore_radius_m=0.105,
            blade_stiffness_n=1.8e5,
            blade_power_p=1.35,
            delta_rb_max_m=0.002,
            contact_stiffness_n_per_m=4.0e6,
            friction_coefficient_theta=0.15,
            friction_coefficient_z=0.22,
            effective_contact_radius_m=0.090,
            lambda_contact=1.0e-3,
        )

        for _ in range(3):
            while True:
                trial_state = np.zeros(6)
                trial_state[0] = rng.uniform(0.018, 0.040)
                trial_state[1] = rng.uniform(0.010, 0.030)
                geometry = bow_contact_geometry(
                    u_m=trial_state[0],
                    v_m=trial_state[1],
                    radial_clearance_m=contact_model.radial_clearance_m,
                    bow_radius_m=contact_model.bow_radius_m,
                    wellbore_radius_m=contact_model.wellbore_radius_m,
                    alpha_b_rad=contact_model.alpha_b_rad,
                    delta_rb_max_m=contact_model.delta_rb_max_m,
                )
                if geometry["delta_rb_m"] > 0.0:
                    break

            analytical_jacobian = assemble_bow_spring_contact_force_and_jacobian(
                [contact_model],
                trial_state,
            )[1]
            numerical_jacobian = _finite_difference_jacobian(
                lambda state: assemble_bow_spring_contact_force_and_jacobian([contact_model], state)[0],
                trial_state,
            )

            rows = np.array([0, 1, 2, 5])
            columns = np.array([0, 1])
            np.testing.assert_allclose(
                analytical_jacobian[np.ix_(rows, columns)],
                numerical_jacobian[np.ix_(rows, columns)],
                rtol=1.0e-6,
                atol=1.0e-8,
            )


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
