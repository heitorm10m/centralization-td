"""
Bow-spring centralizer physics.

Reference: Dao et al. 2023, Geoenergy Science and Engineering 222, 211457

Equations implemented in this module:
  Eq. 1  — Restoring force as vector sum of blade forces
  Eq. 4  — Blade force power law Fi = kblade * delta_i^p
  Eq. 5  — Blade deflection geometry
  Eq. 22 — Blade position vector (PENDING)
  Eq. 23 — Blade radial contact force two-regime (PENDING)
  Eq. 24 — Smoothed blade radial contact force (PENDING)
  Eq. 25 — Axial friction scalar sum (PENDING)
  Eq. 26 — Tangential friction vector resultant (PENDING)
  Eq. 27 — Tangential friction force direction (PENDING)
  Eq. 28 — Individual blade tangential friction (PENDING)
  Eq. 29 — Full nodal force and moment per blade (PENDING)

See CHANGELOG.md for implementation status of each equation.
See DECISIONS.md for decisions about this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np

from .constants import DTYPE


def _norm2(vector: tuple[float, float]) -> float:
    return math.sqrt((vector[0] ** 2) + (vector[1] ** 2))


def blade_deflection_geometry_m(
    ur_cent_n_b_m: tuple[float, float] | np.ndarray,
    d_max_m: float,
    d_hole_m: float,
    alpha_i_rad: float,
) -> float:
    """
    Eq. 5 from PHYSICS_REFERENCE.md.

    delta_i = max(
        Dmax / 2
        - (
            sqrt((Dhole / 2)^2 - (Ur,cent * sin(alpha_i))^2)
            - Ur,cent * cos(alpha_i)
        ),
        0
    )

    The project reference differs from the user-provided inline formula,
    so this implementation follows PHYSICS_REFERENCE.md as ground truth.
    """
    ur_cent_vector = np.asarray(ur_cent_n_b_m, dtype=DTYPE)
    ur_cent_magnitude = DTYPE(np.sqrt(np.dot(ur_cent_vector, ur_cent_vector)))
    half_d_max_m = DTYPE(d_max_m) / DTYPE(2.0)
    half_d_hole_m = DTYPE(d_hole_m) / DTYPE(2.0)
    alpha_i_rad = DTYPE(alpha_i_rad)

    sin_alpha = DTYPE(np.sin(alpha_i_rad))
    cos_alpha = DTYPE(np.cos(alpha_i_rad))

    radicand = (half_d_hole_m * half_d_hole_m) - (
        (ur_cent_magnitude * sin_alpha) * (ur_cent_magnitude * sin_alpha)
    )
    radicand = DTYPE(max(float(radicand), 0.0))

    deformed_radius_m = DTYPE(np.sqrt(radicand)) - (ur_cent_magnitude * cos_alpha)
    deflection_m = half_d_max_m - deformed_radius_m
    if float(deflection_m) <= 0.0:
        return 0.0
    return float(DTYPE(deflection_m))


def blade_force_power_law_n(
    delta_i_m: float,
    k_blade: float,
    p: float,
) -> float:
    """
    Eq. 4 from PHYSICS_REFERENCE.md.

    F_i = k_blade * delta_i^p
    """
    delta_i_m = DTYPE(delta_i_m)
    assert float(delta_i_m) >= 0.0, "Eq. 4 requires delta_i >= 0."

    if float(delta_i_m) == 0.0:
        return 0.0

    k_blade = DTYPE(k_blade)
    p = DTYPE(p)

    if float(p) == 1.0:
        return float(DTYPE(k_blade * delta_i_m))

    return float(DTYPE(k_blade * np.power(delta_i_m, p)))


@dataclass(slots=True)
class BowForceDetailModel:
    source_name: str
    placement_measured_depth_m: float
    bow_index: int
    angle_rad: float
    direction_n_b: tuple[float, float]
    deflection_m: float
    force_magnitude_n: float
    force_vector_n_b: tuple[float, float]


@dataclass(slots=True)
class CentralizerPlacementResultantDetailModel:
    source_name: str
    placement_measured_depth_m: float
    effective_contact_radius_m: float
    axial_force_ratio: float
    tangential_force_ratio: float
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float


@dataclass(slots=True)
class BowSpringSegmentResultModel:
    support_outer_diameter_m: float
    support_contact_clearance_m: float
    equivalent_centering_stiffness_n_per_m: float
    nearby_centralizer_count: int
    support_present: bool
    bow_force_details: list[BowForceDetailModel]
    placement_resultant_details: list[CentralizerPlacementResultantDetailModel]
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float
    centralizer_axial_friction_n: float
    centralizer_tangential_friction_n: float
    centralizer_torque_increment_n_m: float
    effective_contact_radius_m: float


@dataclass(slots=True)
class BowSpringContactModel:
    node_index: int
    alpha_b_rad: float
    radial_clearance_m: float
    bow_radius_m: float
    wellbore_radius_m: float
    blade_stiffness_n: float
    blade_power_p: float
    delta_rb_max_m: float
    contact_stiffness_n_per_m: float
    friction_coefficient_theta: float
    friction_coefficient_z: float
    effective_contact_radius_m: float
    lambda_contact: float


def _extract_node_state_6dof(
    state_vector: np.ndarray,
    node_index: int,
) -> np.ndarray:
    start = 6 * node_index
    stop = start + 6
    if start < 0 or stop > state_vector.size:
        raise IndexError("6-DOF state vector does not contain the requested node index.")
    return state_vector[start:stop]


def bow_contact_geometry(
    *,
    u_m: float,
    v_m: float,
    radial_clearance_m: float,
    bow_radius_m: float,
    wellbore_radius_m: float,
    alpha_b_rad: float,
    delta_rb_max_m: float,
) -> dict[str, float]:
    radius_m = float(np.hypot(DTYPE(u_m), DTYPE(v_m)))
    if radius_m <= 1.0e-12:
        raise ValueError("Eq. 36 requires r > 0 for bow-spring contact derivatives.")

    u_m = float(DTYPE(u_m))
    v_m = float(DTYPE(v_m))
    radial_clearance_m = float(DTYPE(radial_clearance_m))
    bow_radius_m = float(DTYPE(bow_radius_m))
    alpha_b_rad = float(DTYPE(alpha_b_rad))

    cosine_alpha = float(DTYPE(np.cos(alpha_b_rad)))
    sine_alpha = float(DTYPE(np.sin(alpha_b_rad)))

    u_b_m = u_m - ((radial_clearance_m * u_m) / radius_m) + (bow_radius_m * cosine_alpha)
    v_b_m = v_m - ((radial_clearance_m * v_m) / radius_m) + (bow_radius_m * sine_alpha)
    r_b_m = float(np.hypot(DTYPE(u_b_m), DTYPE(v_b_m)))

    if r_b_m <= 1.0e-12:
        raise ValueError("Eq. 36 requires r_b,i > 0 for bow-spring contact derivatives.")

    delta_rb_m = r_b_m - float(DTYPE(wellbore_radius_m))
    delta_i_m = delta_rb_m - float(DTYPE(delta_rb_max_m))
    return {
        "u_m": u_m,
        "v_m": v_m,
        "r_m": radius_m,
        "u_b_m": u_b_m,
        "v_b_m": v_b_m,
        "r_b_m": r_b_m,
        "delta_rb_m": delta_rb_m,
        "delta_i_m": delta_i_m,
        "cosine_alpha": cosine_alpha,
        "sine_alpha": sine_alpha,
    }


def bow_contact_penetration_derivatives(
    *,
    u_m: float,
    v_m: float,
    radial_clearance_m: float,
    bow_radius_m: float,
    wellbore_radius_m: float,
    alpha_b_rad: float,
    delta_rb_max_m: float,
) -> tuple[float, float]:
    geometry = bow_contact_geometry(
        u_m=u_m,
        v_m=v_m,
        radial_clearance_m=radial_clearance_m,
        bow_radius_m=bow_radius_m,
        wellbore_radius_m=wellbore_radius_m,
        alpha_b_rad=alpha_b_rad,
        delta_rb_max_m=delta_rb_max_m,
    )

    r_m = DTYPE(geometry["r_m"])
    u_b_m = DTYPE(geometry["u_b_m"])
    v_b_m = DTYPE(geometry["v_b_m"])
    r_b_m = DTYPE(geometry["r_b_m"])
    u_m = DTYPE(geometry["u_m"])
    v_m = DTYPE(geometry["v_m"])
    radial_clearance_m = DTYPE(radial_clearance_m)

    derivative_u = (
        (((r_m**3) - (radial_clearance_m * (v_m**2))) * u_b_m)
        + (radial_clearance_m * u_m * v_m * v_b_m)
    ) / (r_b_m * (r_m**3))
    derivative_v = (
        (radial_clearance_m * u_m * v_m * u_b_m)
        + (((r_m**3) - (radial_clearance_m * (u_m**2))) * v_b_m)
    ) / (r_b_m * (r_m**3))
    return float(derivative_u), float(derivative_v)


def bow_radial_contact_force_n(
    *,
    delta_rb_m: float,
    delta_i_m: float,
    delta_rb_max_m: float,
    blade_stiffness_n: float,
    blade_power_p: float,
    contact_stiffness_n_per_m: float,
    lambda_contact: float,
) -> float:
    from .contact import smooth_heaviside

    positive_delta_rb_m = max(0.0, float(DTYPE(delta_rb_m)))
    blade_term_n = (
        float(DTYPE(blade_stiffness_n))
        * float(np.power(DTYPE(positive_delta_rb_m), DTYPE(blade_power_p)))
        * float(DTYPE(smooth_heaviside(delta_rb_m, lambda_contact)))
    )
    consolidated_term_n = (
        (
            (float(DTYPE(contact_stiffness_n_per_m)) * float(DTYPE(delta_i_m)))
            - (
                float(DTYPE(blade_stiffness_n))
                * float(np.power(DTYPE(positive_delta_rb_m), DTYPE(blade_power_p)))
            )
            + (
                float(DTYPE(blade_stiffness_n))
                * float(np.power(DTYPE(max(0.0, delta_rb_max_m)), DTYPE(blade_power_p)))
            )
        )
        * float(DTYPE(smooth_heaviside(delta_i_m, lambda_contact)))
    )
    return float(DTYPE(-(blade_term_n + consolidated_term_n)))


def bow_radial_contact_force_derivative_n_per_m(
    *,
    delta_rb_m: float,
    delta_i_m: float,
    delta_rb_max_m: float,
    blade_stiffness_n: float,
    blade_power_p: float,
    contact_stiffness_n_per_m: float,
    lambda_contact: float,
) -> float:
    from .contact import smooth_heaviside, smooth_heaviside_derivative

    positive_delta_rb_m = max(0.0, float(DTYPE(delta_rb_m)))
    positive_delta_rb_power = float(
        np.power(DTYPE(positive_delta_rb_m), DTYPE(blade_power_p))
    )
    if positive_delta_rb_m <= 0.0:
        positive_delta_rb_power_minus_one = 0.0
    else:
        positive_delta_rb_power_minus_one = float(
            np.power(DTYPE(positive_delta_rb_m), DTYPE(blade_power_p - 1.0))
        )

    h_delta_rb = float(DTYPE(smooth_heaviside(delta_rb_m, lambda_contact)))
    h_delta_i = float(DTYPE(smooth_heaviside(delta_i_m, lambda_contact)))
    h_prime_delta_rb = float(DTYPE(smooth_heaviside_derivative(delta_rb_m, lambda_contact)))
    h_prime_delta_i = float(DTYPE(smooth_heaviside_derivative(delta_i_m, lambda_contact)))

    return float(
        DTYPE(
            -float(DTYPE(blade_stiffness_n))
            * positive_delta_rb_power
            * (h_prime_delta_rb - h_prime_delta_i)
            - float(DTYPE(blade_stiffness_n))
            * float(DTYPE(blade_power_p))
            * positive_delta_rb_power_minus_one
            * (h_delta_rb - h_delta_i)
            - float(DTYPE(contact_stiffness_n_per_m))
            * (h_delta_i + (float(DTYPE(delta_i_m)) * h_prime_delta_i))
            - float(DTYPE(blade_stiffness_n))
            * float(np.power(DTYPE(max(0.0, delta_rb_max_m)), DTYPE(blade_power_p)))
            * h_prime_delta_i
        )
    )


def bow_contact_force_vector_local(
    *,
    u_m: float,
    v_m: float,
    radial_clearance_m: float,
    bow_radius_m: float,
    wellbore_radius_m: float,
    alpha_b_rad: float,
    blade_stiffness_n: float,
    blade_power_p: float,
    delta_rb_max_m: float,
    contact_stiffness_n_per_m: float,
    friction_coefficient_theta: float,
    friction_coefficient_z: float,
    effective_contact_radius_m: float,
    lambda_contact: float,
) -> np.ndarray:
    """
    Eq. 24 and Eq. 29 in the local FE frame for one blade.
    """
    geometry = bow_contact_geometry(
        u_m=u_m,
        v_m=v_m,
        radial_clearance_m=radial_clearance_m,
        bow_radius_m=bow_radius_m,
        wellbore_radius_m=wellbore_radius_m,
        alpha_b_rad=alpha_b_rad,
        delta_rb_max_m=delta_rb_max_m,
    )

    radial_force_n = bow_radial_contact_force_n(
        delta_rb_m=geometry["delta_rb_m"],
        delta_i_m=geometry["delta_i_m"],
        delta_rb_max_m=delta_rb_max_m,
        blade_stiffness_n=blade_stiffness_n,
        blade_power_p=blade_power_p,
        contact_stiffness_n_per_m=contact_stiffness_n_per_m,
        lambda_contact=lambda_contact,
    )
    radius_m = geometry["r_m"]
    mu_theta = float(DTYPE(friction_coefficient_theta))
    mu_z = float(DTYPE(friction_coefficient_z))
    c_cbu_i = geometry["cosine_alpha"] - (mu_theta * geometry["sine_alpha"])
    c_cbv_i = geometry["sine_alpha"] + (mu_theta * geometry["cosine_alpha"])
    u_over_r = geometry["u_m"] / radius_m
    v_over_r = geometry["v_m"] / radius_m
    effective_contact_radius_m = float(DTYPE(effective_contact_radius_m))

    return np.asarray(
        [
            radial_force_n * c_cbu_i,
            radial_force_n * c_cbv_i,
            radial_force_n * mu_z,
            effective_contact_radius_m * radial_force_n * mu_z * v_over_r,
            effective_contact_radius_m * radial_force_n * (-mu_z) * u_over_r,
            (
                effective_contact_radius_m
                * radial_force_n
                * mu_theta
                * ((c_cbv_i * u_over_r) - (c_cbu_i * v_over_r))
            ),
        ],
        dtype=DTYPE,
    )


def bow_contact_jacobian_local(
    *,
    u_m: float,
    v_m: float,
    radial_clearance_m: float,
    bow_radius_m: float,
    wellbore_radius_m: float,
    alpha_b_rad: float,
    blade_stiffness_n: float,
    blade_power_p: float,
    delta_rb_max_m: float,
    contact_stiffness_n_per_m: float,
    friction_coefficient_theta: float,
    friction_coefficient_z: float,
    effective_contact_radius_m: float,
    lambda_contact: float,
) -> np.ndarray:
    """
    Eq. 36 local Jacobian block for one blade in 6-DOF ordering.
    """
    jacobian = np.zeros((6, 6), dtype=DTYPE)
    geometry = bow_contact_geometry(
        u_m=u_m,
        v_m=v_m,
        radial_clearance_m=radial_clearance_m,
        bow_radius_m=bow_radius_m,
        wellbore_radius_m=wellbore_radius_m,
        alpha_b_rad=alpha_b_rad,
        delta_rb_max_m=delta_rb_max_m,
    )

    derivative_delta_rb_u, derivative_delta_rb_v = bow_contact_penetration_derivatives(
        u_m=u_m,
        v_m=v_m,
        radial_clearance_m=radial_clearance_m,
        bow_radius_m=bow_radius_m,
        wellbore_radius_m=wellbore_radius_m,
        alpha_b_rad=alpha_b_rad,
        delta_rb_max_m=delta_rb_max_m,
    )
    radial_force_n = DTYPE(
        bow_radial_contact_force_n(
            delta_rb_m=geometry["delta_rb_m"],
            delta_i_m=geometry["delta_i_m"],
            delta_rb_max_m=delta_rb_max_m,
            blade_stiffness_n=blade_stiffness_n,
            blade_power_p=blade_power_p,
            contact_stiffness_n_per_m=contact_stiffness_n_per_m,
            lambda_contact=lambda_contact,
        )
    )
    radial_force_derivative_n_per_m = DTYPE(
        bow_radial_contact_force_derivative_n_per_m(
            delta_rb_m=geometry["delta_rb_m"],
            delta_i_m=geometry["delta_i_m"],
            delta_rb_max_m=delta_rb_max_m,
            blade_stiffness_n=blade_stiffness_n,
            blade_power_p=blade_power_p,
            contact_stiffness_n_per_m=contact_stiffness_n_per_m,
            lambda_contact=lambda_contact,
        )
    )

    radius_m = DTYPE(geometry["r_m"])
    u_m = DTYPE(geometry["u_m"])
    v_m = DTYPE(geometry["v_m"])
    mu_theta = DTYPE(friction_coefficient_theta)
    mu_z = DTYPE(friction_coefficient_z)
    effective_contact_radius_m = DTYPE(effective_contact_radius_m)
    c_cbu_i = DTYPE(geometry["cosine_alpha"] - (float(mu_theta) * geometry["sine_alpha"]))
    c_cbv_i = DTYPE(geometry["sine_alpha"] + (float(mu_theta) * geometry["cosine_alpha"]))

    jacobian[0, 0] = radial_force_derivative_n_per_m * c_cbu_i * DTYPE(derivative_delta_rb_u)
    jacobian[0, 1] = radial_force_derivative_n_per_m * c_cbu_i * DTYPE(derivative_delta_rb_v)
    jacobian[1, 0] = radial_force_derivative_n_per_m * c_cbv_i * DTYPE(derivative_delta_rb_u)
    jacobian[1, 1] = radial_force_derivative_n_per_m * c_cbv_i * DTYPE(derivative_delta_rb_v)

    # WARNING: Eq. 36 term not fully verified against original source
    jacobian[2, 0] = mu_z * radial_force_derivative_n_per_m * DTYPE(derivative_delta_rb_u)
    # WARNING: Eq. 36 term not fully verified against original source
    jacobian[2, 1] = mu_z * radial_force_derivative_n_per_m * DTYPE(derivative_delta_rb_v)

    jacobian[5, 0] = mu_theta * effective_contact_radius_m * (
        (
            radial_force_derivative_n_per_m
            * (DTYPE(1.0) / radius_m)
            * DTYPE(derivative_delta_rb_u)
            * ((c_cbv_i * u_m) - (c_cbu_i * v_m))
        )
        + (
            radial_force_n
            * (v_m / (radius_m**3))
            * ((c_cbv_i * v_m) + (c_cbu_i * u_m))
        )
    )
    jacobian[5, 1] = mu_theta * effective_contact_radius_m * (
        (
            radial_force_derivative_n_per_m
            * (DTYPE(1.0) / radius_m)
            * DTYPE(derivative_delta_rb_v)
            * ((c_cbv_i * u_m) - (c_cbu_i * v_m))
        )
        - (
            radial_force_n
            * (u_m / (radius_m**3))
            * ((c_cbv_i * v_m) + (c_cbu_i * u_m))
        )
    )
    return jacobian


def assemble_bow_spring_contact_force_and_jacobian(
    contacts: Sequence[BowSpringContactModel],
    state_vector: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Assemble blade-wise bow-spring contact contributions for a global 6-DOF state.
    """
    state_vector = np.asarray(state_vector, dtype=DTYPE)
    force_vector = np.zeros_like(state_vector, dtype=DTYPE)
    jacobian_matrix = np.zeros((state_vector.size, state_vector.size), dtype=DTYPE)

    for contact in contacts:
        node_state = _extract_node_state_6dof(state_vector, contact.node_index)
        local_force = bow_contact_force_vector_local(
            u_m=float(node_state[0]),
            v_m=float(node_state[1]),
            radial_clearance_m=contact.radial_clearance_m,
            bow_radius_m=contact.bow_radius_m,
            wellbore_radius_m=contact.wellbore_radius_m,
            alpha_b_rad=contact.alpha_b_rad,
            blade_stiffness_n=contact.blade_stiffness_n,
            blade_power_p=contact.blade_power_p,
            delta_rb_max_m=contact.delta_rb_max_m,
            contact_stiffness_n_per_m=contact.contact_stiffness_n_per_m,
            friction_coefficient_theta=contact.friction_coefficient_theta,
            friction_coefficient_z=contact.friction_coefficient_z,
            effective_contact_radius_m=contact.effective_contact_radius_m,
            lambda_contact=contact.lambda_contact,
        )
        local_jacobian = bow_contact_jacobian_local(
            u_m=float(node_state[0]),
            v_m=float(node_state[1]),
            radial_clearance_m=contact.radial_clearance_m,
            bow_radius_m=contact.bow_radius_m,
            wellbore_radius_m=contact.wellbore_radius_m,
            alpha_b_rad=contact.alpha_b_rad,
            blade_stiffness_n=contact.blade_stiffness_n,
            blade_power_p=contact.blade_power_p,
            delta_rb_max_m=contact.delta_rb_max_m,
            contact_stiffness_n_per_m=contact.contact_stiffness_n_per_m,
            friction_coefficient_theta=contact.friction_coefficient_theta,
            friction_coefficient_z=contact.friction_coefficient_z,
            effective_contact_radius_m=contact.effective_contact_radius_m,
            lambda_contact=contact.lambda_contact,
        )

        start = 6 * contact.node_index
        stop = start + 6
        force_vector[start:stop] += local_force
        jacobian_matrix[start:stop, start:stop] += local_jacobian

    return force_vector, jacobian_matrix


def placement_proximity_weight(
    placement: Any,
    segment_center_md_m: float,
    segment_length_m: float,
) -> float:
    half_influence_length_m = max(0.5 * placement.influence_length_m, 0.5 * segment_length_m)
    if half_influence_length_m <= 0.0:
        return 0.0
    distance_to_segment_center_m = abs(placement.measured_depth_m - segment_center_md_m)
    if distance_to_segment_center_m > half_influence_length_m:
        return 0.0
    return max(0.0, 1.0 - (distance_to_segment_center_m / half_influence_length_m))


def centralizer_effective_contact_diameter_m(placement: Any) -> float:
    diameter_m = placement.support_outer_diameter_m
    if placement.min_contact_diameter_m is not None:
        diameter_m = max(diameter_m, placement.min_contact_diameter_m)
    if placement.max_contact_diameter_m is not None:
        diameter_m = min(diameter_m, placement.max_contact_diameter_m)
    return diameter_m


def centralizer_support_clearance_m(placement: Any, hole_radius_m: float) -> float:
    return max(0.0, hole_radius_m - (0.5 * centralizer_effective_contact_diameter_m(placement)))


def bow_contact_onset_clearance_m(placement: Any, hole_radius_m: float) -> float:
    return placement.inner_clearance_to_pipe_m + centralizer_support_clearance_m(
        placement,
        hole_radius_m,
    )


def build_bow_directions(placement: Any) -> list[tuple[int, float, tuple[float, float]]]:
    directions: list[tuple[int, float, tuple[float, float]]] = []
    angle_reference_rad = math.radians(placement.angular_orientation_reference_deg)
    angle_increment_rad = (2.0 * math.pi) / float(placement.number_of_bows)
    for bow_index in range(placement.number_of_bows):
        angle_rad = angle_reference_rad + (float(bow_index) * angle_increment_rad)
        directions.append((bow_index, angle_rad, (math.cos(angle_rad), math.sin(angle_rad))))
    return directions


def bow_projection_m(
    eccentricity_n_b_m: tuple[float, float],
    direction_n_b: tuple[float, float],
) -> float:
    return (eccentricity_n_b_m[0] * direction_n_b[0]) + (eccentricity_n_b_m[1] * direction_n_b[1])


def bow_deflection_m(
    placement: Any,
    hole_radius_m: float,
    eccentricity_n_b_m: tuple[float, float],
    direction_n_b: tuple[float, float],
) -> float:
    return max(
        0.0,
        bow_projection_m(eccentricity_n_b_m, direction_n_b)
        - bow_contact_onset_clearance_m(placement, hole_radius_m),
    )


def bow_reference_deflection_m(placement: Any, hole_radius_m: float) -> float:
    return max(
        bow_contact_onset_clearance_m(placement, hole_radius_m),
        max(0.05 * centralizer_effective_contact_diameter_m(placement), 1.0e-4),
    )


def resolved_blade_power_law_k(placement: Any, hole_radius_m: float) -> float:
    if placement.blade_power_law_k is not None:
        return placement.blade_power_law_k
    loaded_bow_count = max(1.0, math.ceil(0.5 * float(placement.number_of_bows)))
    reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m)
    return placement.nominal_restoring_force_n / (
        loaded_bow_count * (reference_deflection_m ** placement.blade_power_law_p)
    )


def bow_force_magnitude_n(
    placement: Any,
    hole_radius_m: float,
    deflection_m: float,
) -> float:
    if deflection_m <= 0.0:
        return 0.0
    return blade_force_power_law_n(
        deflection_m,
        resolved_blade_power_law_k(placement, hole_radius_m),
        placement.blade_power_law_p,
    )


def equivalent_bow_support_stiffness_n_per_m(placement: Any, hole_radius_m: float) -> float:
    loaded_bow_count = max(1.0, math.ceil(0.5 * float(placement.number_of_bows)))
    reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m)
    tangent_stiffness_per_bow_n_per_m = (
        resolved_blade_power_law_k(placement, hole_radius_m)
        * placement.blade_power_law_p
        * (reference_deflection_m ** (placement.blade_power_law_p - 1.0))
    )
    return loaded_bow_count * tangent_stiffness_per_bow_n_per_m


def centralizer_running_force_ratio(placement: Any) -> float:
    if placement.nominal_restoring_force_n <= 0.0:
        return 0.0
    return max(0.0, placement.nominal_running_force_n / placement.nominal_restoring_force_n)


def centralizer_axial_force_ratio(placement: Any) -> float:
    if getattr(placement, "axial_force_ratio", None) is not None:
        return max(0.0, float(placement.axial_force_ratio))
    return centralizer_running_force_ratio(placement)


def centralizer_tangential_force_ratio(placement: Any) -> float:
    if getattr(placement, "tangential_force_ratio", None) is not None:
        return max(0.0, float(placement.tangential_force_ratio))
    return centralizer_running_force_ratio(placement)


def evaluate_bow_spring_segment_result(
    segment: Any,
    placements: list[Any],
    eccentricity_n_b_m: tuple[float, float],
) -> BowSpringSegmentResultModel:
    result = BowSpringSegmentResultModel(
        support_outer_diameter_m=segment.section.outer_diameter_m,
        support_contact_clearance_m=0.0,
        equivalent_centering_stiffness_n_per_m=0.0,
        nearby_centralizer_count=0,
        support_present=False,
        bow_force_details=[],
        placement_resultant_details=[],
        bow_resultant_vector_n_b=(0.0, 0.0),
        bow_resultant_magnitude_n=0.0,
        centralizer_axial_friction_n=0.0,
        centralizer_tangential_friction_n=0.0,
        centralizer_torque_increment_n_m=0.0,
        effective_contact_radius_m=0.0,
    )
    if segment.reference_hole_diameter_m <= 0.0:
        return result

    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    bow_resultant_n = 0.0
    bow_resultant_b = 0.0

    for placement in placements:
        proximity_weight = placement_proximity_weight(
            placement,
            segment.measured_depth_center_m,
            segment.segment_length_m,
        )
        if proximity_weight <= 0.0:
            continue

        result.nearby_centralizer_count += 1
        result.support_present = True
        result.support_outer_diameter_m = max(
            result.support_outer_diameter_m,
            centralizer_effective_contact_diameter_m(placement),
        )
        placement_support_contact_clearance_m = bow_contact_onset_clearance_m(
            placement,
            hole_radius_m,
        )
        result.support_contact_clearance_m = (
            placement_support_contact_clearance_m
            if result.nearby_centralizer_count == 1
            else min(result.support_contact_clearance_m, placement_support_contact_clearance_m)
        )
        result.equivalent_centering_stiffness_n_per_m += (
            proximity_weight * equivalent_bow_support_stiffness_n_per_m(placement, hole_radius_m)
        )

        placement_resultant_n = 0.0
        placement_resultant_b = 0.0
        for bow_index, angle_rad, direction_n_b in build_bow_directions(placement):
            deflection_m = bow_deflection_m(
                placement,
                hole_radius_m,
                eccentricity_n_b_m,
                direction_n_b,
            )
            force_magnitude_n = proximity_weight * bow_force_magnitude_n(
                placement,
                hole_radius_m,
                deflection_m,
            )
            force_vector_n_b = (
                force_magnitude_n * direction_n_b[0],
                force_magnitude_n * direction_n_b[1],
            )
            placement_resultant_n += force_vector_n_b[0]
            placement_resultant_b += force_vector_n_b[1]
            result.bow_force_details.append(
                BowForceDetailModel(
                    source_name=placement.source_name,
                    placement_measured_depth_m=placement.measured_depth_m,
                    bow_index=bow_index,
                    angle_rad=angle_rad,
                    direction_n_b=direction_n_b,
                    deflection_m=deflection_m,
                    force_magnitude_n=force_magnitude_n,
                    force_vector_n_b=force_vector_n_b,
                )
            )

        placement_resultant_magnitude_n = _norm2((placement_resultant_n, placement_resultant_b))
        placement_axial_force_ratio = centralizer_axial_force_ratio(placement)
        placement_tangential_force_ratio = centralizer_tangential_force_ratio(placement)
        placement_effective_contact_radius_m = 0.5 * centralizer_effective_contact_diameter_m(
            placement
        )
        result.placement_resultant_details.append(
            CentralizerPlacementResultantDetailModel(
                source_name=placement.source_name,
                placement_measured_depth_m=placement.measured_depth_m,
                effective_contact_radius_m=placement_effective_contact_radius_m,
                axial_force_ratio=placement_axial_force_ratio,
                tangential_force_ratio=placement_tangential_force_ratio,
                bow_resultant_vector_n_b=(placement_resultant_n, placement_resultant_b),
                bow_resultant_magnitude_n=placement_resultant_magnitude_n,
            )
        )
        bow_resultant_n += placement_resultant_n
        bow_resultant_b += placement_resultant_b
        result.centralizer_axial_friction_n += (
            placement_axial_force_ratio * placement_resultant_magnitude_n
        )
        result.centralizer_tangential_friction_n += (
            placement_tangential_force_ratio * placement_resultant_magnitude_n
        )
        result.centralizer_torque_increment_n_m += (
            placement_tangential_force_ratio
            * placement_resultant_magnitude_n
            * placement_effective_contact_radius_m
        )
        result.effective_contact_radius_m = max(
            result.effective_contact_radius_m,
            placement_effective_contact_radius_m,
        )

    result.bow_resultant_vector_n_b = (bow_resultant_n, bow_resultant_b)
    result.bow_resultant_magnitude_n = _norm2(result.bow_resultant_vector_n_b)
    return result
