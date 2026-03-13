from __future__ import annotations

from dataclasses import dataclass

from .local_tangential_model import (
    LocalTangentialModelInputs,
    evaluate_local_tangential_state,
)


@dataclass(slots=True)
class BodyTorqueDragContributionModel:
    axial_friction_n: float
    tangential_friction_n: float
    torque_increment_n_m: float
    contact_radius_m: float
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    torsional_slip_indicator: float
    tangential_mobilization: float
    tangential_demand_factor: float
    tangential_traction_indicator: float
    tangential_regime: str
    feedback_applied: bool
    status: str


def evaluate_body_torque_drag_contribution(
    segment: object,
    section: object,
    *,
    reduced_torsional_load_n_m: float = 0.0,
    reduced_twist_rate_rad_per_m: float = 0.0,
) -> BodyTorqueDragContributionModel:
    axial_friction_n = max(0.0, section.friction_coefficient) * max(
        0.0,
        segment.body_normal_reaction_estimate_n,
    )
    contact_radius_m = max(0.0, 0.5 * section.outer_diameter_m)
    tangential_state = evaluate_local_tangential_state(
        LocalTangentialModelInputs(
            reduced_torsional_load_n_m=max(0.0, reduced_torsional_load_n_m),
            reduced_twist_rate_rad_per_m=abs(reduced_twist_rate_rad_per_m),
            effective_contact_radius_m=contact_radius_m,
            normal_capacity_n=max(0.0, segment.body_normal_reaction_estimate_n),
            baseline_tangential_demand_n=axial_friction_n,
        )
    )
    tangential_friction_n = tangential_state.mobilized_tangential_demand_n
    return BodyTorqueDragContributionModel(
        axial_friction_n=axial_friction_n,
        tangential_friction_n=tangential_friction_n,
        torque_increment_n_m=tangential_friction_n * contact_radius_m,
        contact_radius_m=contact_radius_m,
        reduced_torsional_load_n_m=tangential_state.reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m=tangential_state.reduced_twist_rate_rad_per_m,
        torsional_slip_indicator=tangential_state.tangential_slip_indicator,
        tangential_mobilization=tangential_state.tangential_mobilization,
        tangential_demand_factor=tangential_state.tangential_demand_factor,
        tangential_traction_indicator=tangential_state.tangential_traction_indicator,
        tangential_regime=tangential_state.tangential_regime,
        feedback_applied=tangential_state.feedback_applied,
        status=(
            "phase14-reduced-local-tangential-body-law"
            if tangential_state.feedback_applied
            else "phase14-reduced-local-tangential-body-law-baseline"
            if axial_friction_n > 0.0
            else "inactive"
        ),
    )
