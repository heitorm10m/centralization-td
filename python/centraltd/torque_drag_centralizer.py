from __future__ import annotations

from dataclasses import dataclass
import math

from .local_tangential_model import (
    LocalTangentialModelInputs,
    evaluate_local_tangential_state,
)


def _norm2(vector: tuple[float, float]) -> float:
    return math.sqrt((vector[0] ** 2) + (vector[1] ** 2))


def _dot2(lhs: tuple[float, float], rhs: tuple[float, float]) -> float:
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1])


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _normalize2(vector: tuple[float, float]) -> tuple[float, float]:
    magnitude = _norm2(vector)
    if magnitude <= 1.0e-12:
        return (0.0, 0.0)
    return (vector[0] / magnitude, vector[1] / magnitude)


def _normalize2_or_fallback(
    vector: tuple[float, float],
    fallback: tuple[float, float],
) -> tuple[float, float]:
    normalized = _normalize2(vector)
    if _norm2(normalized) > 1.0e-12:
        return normalized
    return _normalize2(fallback)


def _rotate_ccw_90(vector: tuple[float, float]) -> tuple[float, float]:
    return (-vector[1], vector[0])


@dataclass(slots=True)
class CentralizerPlacementTorqueContributionModel:
    source_name: str
    placement_measured_depth_m: float
    effective_contact_radius_m: float
    axial_force_ratio: float
    tangential_force_ratio: float
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    torsional_slip_indicator: float
    tangential_mobilization: float
    torsional_tangential_demand_factor: float
    tangential_traction_indicator: float
    tangential_regime: str
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float
    radial_direction_n_b: tuple[float, float]
    local_contact_direction_n_b: tuple[float, float]
    effective_radial_direction_n_b: tuple[float, float]
    tangential_direction_n_b: tuple[float, float]
    tangential_friction_vector_n_b: tuple[float, float]
    tangential_friction_vector_magnitude_n: float
    local_contact_weight: float
    direction_alignment_cosine: float
    projected_contact_normal_n: float
    friction_interaction_scale: float
    axial_friction_n: float
    tangential_friction_n: float
    torque_increment_n_m: float
    active: bool
    status: str


@dataclass(slots=True)
class CentralizerTorqueContributionModel:
    placement_contributions: list[CentralizerPlacementTorqueContributionModel]
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    torsional_slip_indicator: float
    tangential_mobilization: float
    torsional_tangential_demand_factor: float
    tangential_traction_indicator: float
    tangential_regime: str
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float
    local_contact_direction_n_b: tuple[float, float]
    effective_radial_direction_n_b: tuple[float, float]
    tangential_direction_n_b: tuple[float, float]
    tangential_friction_vector_n_b: tuple[float, float]
    tangential_friction_vector_magnitude_n: float
    projected_contact_normal_n: float
    friction_interaction_scale: float
    axial_friction_n: float
    tangential_friction_n: float
    torque_increment_n_m: float
    effective_contact_radius_m: float
    active: bool
    status: str


def _evaluate_placement_contribution(
    *,
    source_name: str,
    placement_measured_depth_m: float,
    effective_contact_radius_m: float,
    axial_force_ratio: float,
    tangential_force_ratio: float,
    bow_resultant_vector_n_b: tuple[float, float],
    bow_resultant_magnitude_n: float,
    radial_direction_n_b: tuple[float, float],
    local_contact_direction_n_b: tuple[float, float],
    effective_radial_direction_n_b: tuple[float, float],
    tangential_direction_n_b: tuple[float, float],
    local_contact_weight: float,
    direction_alignment_cosine: float,
    projected_contact_normal_n: float,
    reduced_torsional_load_n_m: float,
    reduced_twist_rate_rad_per_m: float,
    active_status: str,
) -> CentralizerPlacementTorqueContributionModel:
    tangential_state = evaluate_local_tangential_state(
        LocalTangentialModelInputs(
            reduced_torsional_load_n_m=max(0.0, reduced_torsional_load_n_m),
            reduced_twist_rate_rad_per_m=abs(reduced_twist_rate_rad_per_m),
            effective_contact_radius_m=effective_contact_radius_m,
            normal_capacity_n=projected_contact_normal_n,
            baseline_tangential_demand_n=(
                tangential_force_ratio * projected_contact_normal_n
            ),
        )
    )
    contribution = CentralizerPlacementTorqueContributionModel(
        source_name=source_name,
        placement_measured_depth_m=placement_measured_depth_m,
        effective_contact_radius_m=effective_contact_radius_m,
        axial_force_ratio=axial_force_ratio,
        tangential_force_ratio=tangential_force_ratio,
        reduced_torsional_load_n_m=tangential_state.reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m=tangential_state.reduced_twist_rate_rad_per_m,
        torsional_slip_indicator=tangential_state.tangential_slip_indicator,
        tangential_mobilization=tangential_state.tangential_mobilization,
        torsional_tangential_demand_factor=tangential_state.tangential_demand_factor,
        tangential_traction_indicator=tangential_state.tangential_traction_indicator,
        tangential_regime=tangential_state.tangential_regime,
        bow_resultant_vector_n_b=bow_resultant_vector_n_b,
        bow_resultant_magnitude_n=bow_resultant_magnitude_n,
        radial_direction_n_b=radial_direction_n_b,
        local_contact_direction_n_b=local_contact_direction_n_b,
        effective_radial_direction_n_b=effective_radial_direction_n_b,
        tangential_direction_n_b=tangential_direction_n_b,
        tangential_friction_vector_n_b=(0.0, 0.0),
        tangential_friction_vector_magnitude_n=0.0,
        local_contact_weight=local_contact_weight,
        direction_alignment_cosine=direction_alignment_cosine,
        projected_contact_normal_n=projected_contact_normal_n,
        friction_interaction_scale=1.0,
        axial_friction_n=0.0,
        tangential_friction_n=0.0,
        torque_increment_n_m=0.0,
        active=False,
        status="inactive",
    )
    if bow_resultant_magnitude_n <= 0.0 or projected_contact_normal_n <= 1.0e-12:
        return contribution

    axial_friction_demand_n = axial_force_ratio * projected_contact_normal_n
    tangential_friction_demand_n = tangential_state.mobilized_tangential_demand_n
    combined_friction_demand_n = math.hypot(
        axial_friction_demand_n,
        tangential_friction_demand_n,
    )
    contribution.friction_interaction_scale = (
        1.0
        if projected_contact_normal_n <= 1.0e-12
        or combined_friction_demand_n <= projected_contact_normal_n
        else projected_contact_normal_n / combined_friction_demand_n
    )
    contribution.axial_friction_n = (
        contribution.friction_interaction_scale * axial_friction_demand_n
    )
    contribution.tangential_friction_n = (
        contribution.friction_interaction_scale * tangential_friction_demand_n
    )
    contribution.tangential_friction_vector_n_b = (
        contribution.tangential_friction_n * tangential_direction_n_b[0],
        contribution.tangential_friction_n * tangential_direction_n_b[1],
    )
    contribution.tangential_friction_vector_magnitude_n = _norm2(
        contribution.tangential_friction_vector_n_b
    )
    contribution.torque_increment_n_m = (
        contribution.tangential_friction_n * effective_contact_radius_m
    )
    contribution.active = True
    contribution.status = active_status
    return contribution


def _aggregate_contributions(
    placement_contributions: list[CentralizerPlacementTorqueContributionModel],
    *,
    fallback_bow_resultant_vector_n_b: tuple[float, float],
    fallback_local_contact_direction_n_b: tuple[float, float],
    active_status: str,
) -> CentralizerTorqueContributionModel:
    bow_resultant_vector_n_b = [0.0, 0.0]
    effective_radial_sum_n_b = [0.0, 0.0]
    projected_contact_normal_sum_n = 0.0
    friction_interaction_weighted_sum_n = 0.0
    torsional_slip_weighted_sum = 0.0
    tangential_mobilization_weighted_sum = 0.0
    torsional_demand_factor_weighted_sum = 0.0
    tangential_traction_indicator_weighted_sum = 0.0
    tangential_friction_n_vec = 0.0
    tangential_friction_b_vec = 0.0
    axial_friction_n = 0.0
    tangential_friction_n = 0.0
    torque_increment_n_m = 0.0
    effective_contact_radius_m = 0.0
    active = False
    reduced_torsional_load_n_m = 0.0
    reduced_twist_rate_rad_per_m = 0.0
    local_contact_direction_n_b = _normalize2(fallback_local_contact_direction_n_b)

    for contribution in placement_contributions:
        bow_resultant_vector_n_b[0] += contribution.bow_resultant_vector_n_b[0]
        bow_resultant_vector_n_b[1] += contribution.bow_resultant_vector_n_b[1]
        reduced_torsional_load_n_m = max(
            reduced_torsional_load_n_m,
            contribution.reduced_torsional_load_n_m,
        )
        reduced_twist_rate_rad_per_m = max(
            reduced_twist_rate_rad_per_m,
            contribution.reduced_twist_rate_rad_per_m,
        )
        if _norm2(local_contact_direction_n_b) <= 1.0e-12 and _norm2(
            contribution.local_contact_direction_n_b
        ) > 1.0e-12:
            local_contact_direction_n_b = _normalize2(
                contribution.local_contact_direction_n_b
            )
        if not contribution.active:
            continue
        active = True
        axial_friction_n += contribution.axial_friction_n
        tangential_friction_n += contribution.tangential_friction_n
        tangential_friction_n_vec += contribution.tangential_friction_vector_n_b[0]
        tangential_friction_b_vec += contribution.tangential_friction_vector_n_b[1]
        torque_increment_n_m += contribution.torque_increment_n_m
        effective_contact_radius_m = max(
            effective_contact_radius_m,
            contribution.effective_contact_radius_m,
        )
        projected_contact_normal_sum_n += contribution.projected_contact_normal_n
        friction_interaction_weighted_sum_n += (
            contribution.projected_contact_normal_n * contribution.friction_interaction_scale
        )
        torsional_slip_weighted_sum += (
            contribution.projected_contact_normal_n * contribution.torsional_slip_indicator
        )
        tangential_mobilization_weighted_sum += (
            contribution.projected_contact_normal_n * contribution.tangential_mobilization
        )
        torsional_demand_factor_weighted_sum += (
            contribution.projected_contact_normal_n
            * contribution.torsional_tangential_demand_factor
        )
        tangential_traction_indicator_weighted_sum += (
            contribution.projected_contact_normal_n
            * contribution.tangential_traction_indicator
        )
        effective_radial_sum_n_b[0] += (
            contribution.projected_contact_normal_n
            * contribution.effective_radial_direction_n_b[0]
        )
        effective_radial_sum_n_b[1] += (
            contribution.projected_contact_normal_n
            * contribution.effective_radial_direction_n_b[1]
        )

    bow_resultant_vector = (
        bow_resultant_vector_n_b[0],
        bow_resultant_vector_n_b[1],
    )
    if _norm2(bow_resultant_vector) <= 1.0e-12:
        bow_resultant_vector = fallback_bow_resultant_vector_n_b
    bow_resultant_magnitude_n = _norm2(bow_resultant_vector)
    effective_radial_direction_n_b = _normalize2_or_fallback(
        (effective_radial_sum_n_b[0], effective_radial_sum_n_b[1]),
        bow_resultant_vector,
    )
    tangential_friction_vector_n_b = (
        tangential_friction_n_vec,
        tangential_friction_b_vec,
    )
    tangential_friction_vector_magnitude_n = _norm2(tangential_friction_vector_n_b)
    if tangential_friction_vector_magnitude_n > 1.0e-12:
        tangential_direction_n_b = (
            tangential_friction_vector_n_b[0] / tangential_friction_vector_magnitude_n,
            tangential_friction_vector_n_b[1] / tangential_friction_vector_magnitude_n,
        )
    elif _norm2(effective_radial_direction_n_b) > 1.0e-12:
        tangential_direction_n_b = _rotate_ccw_90(effective_radial_direction_n_b)
    elif bow_resultant_magnitude_n > 1.0e-12:
        effective_radial_direction_n_b = _normalize2(bow_resultant_vector)
        tangential_direction_n_b = _rotate_ccw_90(effective_radial_direction_n_b)
    else:
        tangential_direction_n_b = (0.0, 0.0)
    tangential_traction_indicator = (
        0.0
        if projected_contact_normal_sum_n <= 1.0e-12
        else tangential_traction_indicator_weighted_sum / projected_contact_normal_sum_n
    )
    tangential_regime = (
        "inactive"
        if not active
        else "reduced-partial-adhesion-proxy"
        if tangential_traction_indicator < 0.25
        else "reduced-mobilizing-traction"
        if tangential_traction_indicator < 0.95
        else "reduced-slip-limited-traction"
    )

    return CentralizerTorqueContributionModel(
        placement_contributions=placement_contributions,
        reduced_torsional_load_n_m=reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
        torsional_slip_indicator=(
            0.0
            if projected_contact_normal_sum_n <= 1.0e-12
            else torsional_slip_weighted_sum / projected_contact_normal_sum_n
        ),
        tangential_mobilization=(
            0.0
            if projected_contact_normal_sum_n <= 1.0e-12
            else tangential_mobilization_weighted_sum / projected_contact_normal_sum_n
        ),
        torsional_tangential_demand_factor=(
            1.0
            if projected_contact_normal_sum_n <= 1.0e-12
            else torsional_demand_factor_weighted_sum / projected_contact_normal_sum_n
        ),
        tangential_traction_indicator=tangential_traction_indicator,
        tangential_regime=tangential_regime,
        bow_resultant_vector_n_b=bow_resultant_vector,
        bow_resultant_magnitude_n=bow_resultant_magnitude_n,
        local_contact_direction_n_b=local_contact_direction_n_b,
        effective_radial_direction_n_b=effective_radial_direction_n_b,
        tangential_direction_n_b=tangential_direction_n_b,
        tangential_friction_vector_n_b=tangential_friction_vector_n_b,
        tangential_friction_vector_magnitude_n=tangential_friction_vector_magnitude_n,
        projected_contact_normal_n=projected_contact_normal_sum_n,
        friction_interaction_scale=(
            1.0
            if projected_contact_normal_sum_n <= 1.0e-12
            else friction_interaction_weighted_sum_n / projected_contact_normal_sum_n
        ),
        axial_friction_n=axial_friction_n,
        tangential_friction_n=tangential_friction_n,
        torque_increment_n_m=torque_increment_n_m,
        effective_contact_radius_m=effective_contact_radius_m,
        active=active,
        status=active_status if active else "inactive",
    )


def evaluate_centralizer_torque_contribution(
    bow_segment_result: object,
    local_contact_direction_n_b: tuple[float, float],
    body_normal_reaction_vector_n_b: tuple[float, float],
) -> CentralizerTorqueContributionModel:
    placement_contributions: list[CentralizerPlacementTorqueContributionModel] = []
    normalized_contact_direction_n_b = _normalize2(local_contact_direction_n_b)
    body_normal_reaction_magnitude_n = _norm2(body_normal_reaction_vector_n_b)

    for placement_result in bow_segment_result.placement_resultant_details:
        if placement_result.bow_resultant_magnitude_n <= 0.0:
            placement_contributions.append(
                _evaluate_placement_contribution(
                    source_name=placement_result.source_name,
                    placement_measured_depth_m=placement_result.placement_measured_depth_m,
                    effective_contact_radius_m=placement_result.effective_contact_radius_m,
                    axial_force_ratio=placement_result.axial_force_ratio,
                    tangential_force_ratio=placement_result.tangential_force_ratio,
                    bow_resultant_vector_n_b=placement_result.bow_resultant_vector_n_b,
                    bow_resultant_magnitude_n=placement_result.bow_resultant_magnitude_n,
                    radial_direction_n_b=(0.0, 0.0),
                    local_contact_direction_n_b=normalized_contact_direction_n_b,
                    effective_radial_direction_n_b=(0.0, 0.0),
                    tangential_direction_n_b=(0.0, 0.0),
                    local_contact_weight=0.0,
                    direction_alignment_cosine=0.0,
                    projected_contact_normal_n=0.0,
                    reduced_torsional_load_n_m=0.0,
                    reduced_twist_rate_rad_per_m=0.0,
                    active_status="phase11-reduced-contact-informed-placement-vector-torque",
                )
            )
            continue

        radial_direction_n_b = _normalize2(placement_result.bow_resultant_vector_n_b)
        local_contact_weight = (
            0.0
            if body_normal_reaction_magnitude_n <= 1.0e-12
            else body_normal_reaction_magnitude_n
            / (body_normal_reaction_magnitude_n + placement_result.bow_resultant_magnitude_n)
        )
        effective_radial_direction_n_b = (
            radial_direction_n_b
            if _norm2(normalized_contact_direction_n_b) <= 1.0e-12
            else _normalize2_or_fallback(
                (
                    ((1.0 - local_contact_weight) * radial_direction_n_b[0])
                    + (local_contact_weight * normalized_contact_direction_n_b[0]),
                    ((1.0 - local_contact_weight) * radial_direction_n_b[1])
                    + (local_contact_weight * normalized_contact_direction_n_b[1]),
                ),
                radial_direction_n_b,
            )
        )
        direction_alignment_cosine = (
            1.0
            if _norm2(normalized_contact_direction_n_b) <= 1.0e-12
            else _clamp(
                _dot2(radial_direction_n_b, normalized_contact_direction_n_b),
                -1.0,
                1.0,
            )
        )
        projected_contact_normal_n = max(
            0.0,
            _dot2(
                placement_result.bow_resultant_vector_n_b,
                effective_radial_direction_n_b,
            ),
        )
        tangential_direction_n_b = _rotate_ccw_90(effective_radial_direction_n_b)
        placement_contributions.append(
            _evaluate_placement_contribution(
                source_name=placement_result.source_name,
                placement_measured_depth_m=placement_result.placement_measured_depth_m,
                effective_contact_radius_m=placement_result.effective_contact_radius_m,
                axial_force_ratio=placement_result.axial_force_ratio,
                tangential_force_ratio=placement_result.tangential_force_ratio,
                bow_resultant_vector_n_b=placement_result.bow_resultant_vector_n_b,
                bow_resultant_magnitude_n=placement_result.bow_resultant_magnitude_n,
                radial_direction_n_b=radial_direction_n_b,
                local_contact_direction_n_b=normalized_contact_direction_n_b,
                effective_radial_direction_n_b=effective_radial_direction_n_b,
                tangential_direction_n_b=tangential_direction_n_b,
                local_contact_weight=local_contact_weight,
                direction_alignment_cosine=direction_alignment_cosine,
                projected_contact_normal_n=projected_contact_normal_n,
                reduced_torsional_load_n_m=0.0,
                reduced_twist_rate_rad_per_m=0.0,
                active_status="phase11-reduced-contact-informed-placement-vector-torque",
            )
        )
    return _aggregate_contributions(
        placement_contributions,
        fallback_bow_resultant_vector_n_b=bow_segment_result.bow_resultant_vector_n_b,
        fallback_local_contact_direction_n_b=normalized_contact_direction_n_b,
        active_status="phase11-reduced-contact-informed-vector-centralizer-torque",
    )


def evaluate_centralizer_torque_contribution_from_details(
    placement_details: list[CentralizerPlacementTorqueContributionModel],
    *,
    reduced_torsional_load_n_m: float,
    reduced_twist_rate_rad_per_m: float,
) -> CentralizerTorqueContributionModel:
    placement_contributions: list[CentralizerPlacementTorqueContributionModel] = []
    fallback_bow_resultant_vector_n_b = [0.0, 0.0]
    fallback_local_contact_direction_n_b = (0.0, 0.0)

    for detail in placement_details:
        fallback_bow_resultant_vector_n_b[0] += detail.bow_resultant_vector_n_b[0]
        fallback_bow_resultant_vector_n_b[1] += detail.bow_resultant_vector_n_b[1]
        if _norm2(fallback_local_contact_direction_n_b) <= 1.0e-12 and _norm2(
            detail.local_contact_direction_n_b
        ) > 1.0e-12:
            fallback_local_contact_direction_n_b = _normalize2(
                detail.local_contact_direction_n_b
            )
        placement_contributions.append(
            _evaluate_placement_contribution(
                source_name=detail.source_name,
                placement_measured_depth_m=detail.placement_measured_depth_m,
                effective_contact_radius_m=detail.effective_contact_radius_m,
                axial_force_ratio=detail.axial_force_ratio,
                tangential_force_ratio=detail.tangential_force_ratio,
                bow_resultant_vector_n_b=detail.bow_resultant_vector_n_b,
                bow_resultant_magnitude_n=detail.bow_resultant_magnitude_n,
                radial_direction_n_b=detail.radial_direction_n_b,
                local_contact_direction_n_b=detail.local_contact_direction_n_b,
                effective_radial_direction_n_b=detail.effective_radial_direction_n_b,
                tangential_direction_n_b=detail.tangential_direction_n_b,
                local_contact_weight=detail.local_contact_weight,
                direction_alignment_cosine=detail.direction_alignment_cosine,
                projected_contact_normal_n=detail.projected_contact_normal_n,
                reduced_torsional_load_n_m=reduced_torsional_load_n_m,
                reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
                active_status="phase14-reduced-local-tangential-placement-vector-torque",
            )
        )

    return _aggregate_contributions(
        placement_contributions,
        fallback_bow_resultant_vector_n_b=(
            fallback_bow_resultant_vector_n_b[0],
            fallback_bow_resultant_vector_n_b[1],
        ),
        fallback_local_contact_direction_n_b=fallback_local_contact_direction_n_b,
        active_status="phase14-reduced-local-tangential-vector-centralizer-torque",
    )
