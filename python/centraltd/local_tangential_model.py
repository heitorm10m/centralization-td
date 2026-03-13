from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class LocalTangentialModelInputs:
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    effective_contact_radius_m: float
    normal_capacity_n: float
    baseline_tangential_demand_n: float


@dataclass(slots=True)
class LocalTangentialStateModel:
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    effective_contact_radius_m: float
    normal_capacity_n: float
    baseline_tangential_demand_n: float
    tangential_slip_indicator: float
    tangential_mobilization: float
    tangential_demand_factor: float
    mobilized_tangential_demand_n: float
    tangential_traction_indicator: float
    active: bool
    feedback_applied: bool
    tangential_regime: str
    status: str


def evaluate_local_tangential_state(
    inputs: LocalTangentialModelInputs,
) -> LocalTangentialStateModel:
    reduced_torsional_load_n_m = max(0.0, inputs.reduced_torsional_load_n_m)
    reduced_twist_rate_rad_per_m = abs(inputs.reduced_twist_rate_rad_per_m)
    effective_contact_radius_m = max(0.0, inputs.effective_contact_radius_m)
    normal_capacity_n = max(0.0, inputs.normal_capacity_n)
    baseline_tangential_demand_n = max(0.0, inputs.baseline_tangential_demand_n)
    tangential_slip_indicator = (
        reduced_twist_rate_rad_per_m * effective_contact_radius_m
    )
    tangential_mobilization = tangential_slip_indicator / (
        1.0 + tangential_slip_indicator
    )
    tangential_demand_factor = 1.0 + tangential_mobilization
    mobilized_tangential_demand_n = (
        baseline_tangential_demand_n * tangential_demand_factor
    )
    tangential_traction_indicator = (
        0.0
        if normal_capacity_n <= 1.0e-12
        else min(1.0, mobilized_tangential_demand_n / normal_capacity_n)
    )
    active = normal_capacity_n > 1.0e-12 and effective_contact_radius_m > 1.0e-12
    feedback_applied = (
        active
        and tangential_mobilization > 1.0e-12
        and baseline_tangential_demand_n > 1.0e-12
    )
    if not active or mobilized_tangential_demand_n <= 1.0e-12:
        tangential_regime = "inactive"
        status = "inactive"
    elif tangential_traction_indicator < 0.25:
        tangential_regime = "reduced-partial-adhesion-proxy"
        status = (
            "phase14-reduced-local-tangential-state-mobilized"
            if feedback_applied
            else "phase14-reduced-local-tangential-state-baseline"
        )
    elif tangential_traction_indicator < 0.95:
        tangential_regime = "reduced-mobilizing-traction"
        status = (
            "phase14-reduced-local-tangential-state-mobilized"
            if feedback_applied
            else "phase14-reduced-local-tangential-state-baseline"
        )
    else:
        tangential_regime = "reduced-slip-limited-traction"
        status = (
            "phase14-reduced-local-tangential-state-mobilized"
            if feedback_applied
            else "phase14-reduced-local-tangential-state-baseline"
        )
    return LocalTangentialStateModel(
        reduced_torsional_load_n_m=reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
        effective_contact_radius_m=effective_contact_radius_m,
        normal_capacity_n=normal_capacity_n,
        baseline_tangential_demand_n=baseline_tangential_demand_n,
        tangential_slip_indicator=tangential_slip_indicator,
        tangential_mobilization=tangential_mobilization,
        tangential_demand_factor=tangential_demand_factor,
        mobilized_tangential_demand_n=mobilized_tangential_demand_n,
        tangential_traction_indicator=tangential_traction_indicator,
        active=active,
        feedback_applied=feedback_applied,
        tangential_regime=tangential_regime,
        status=status,
    )
