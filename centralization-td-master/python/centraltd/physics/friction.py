"""
Friction laws for string-borehole and string-centralizer interaction.

Reference: Dao et al. 2023, Geoenergy Science and Engineering 222, 211457

Equations implemented in this module:
  Eq. 6  — Axial friction from blades scalar sum (PENDING)
  Eq. 7  — Tangential friction from restoring force (PENDING)
  Eq. 8  — Friction torque on casing (PENDING)
  Eq. 10 — Test friction coefficient from running force (PENDING)
  Eq. 19 — Directional friction split (PENDING)
  Eq. 20 — Velocity-dependent friction μ(Vg) (PENDING)

See CHANGELOG.md for implementation status of each equation.
See DECISIONS.md for decisions about this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..models import LoadedCase, StringSectionModel

if TYPE_CHECKING:
    from ..solver.mechanics import MechanicalSegmentResultModel
    from ..torque_drag import TorquePointModel


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
    tangential_slip_indicator = reduced_twist_rate_rad_per_m * effective_contact_radius_m
    tangential_mobilization = tangential_slip_indicator / (1.0 + tangential_slip_indicator)
    tangential_demand_factor = 1.0 + tangential_mobilization
    mobilized_tangential_demand_n = baseline_tangential_demand_n * tangential_demand_factor
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


@dataclass(slots=True)
class ReducedTorqueAccumulationPointModel:
    measured_depth_m: float
    body_torque_increment_n_m: float
    centralizer_torque_increment_n_m: float
    local_torque_increment_n_m: float
    raw_cumulative_torque_n_m: float
    carried_torsional_load_n_m: float
    status: str


@dataclass(slots=True)
class TorsionalStatePointModel:
    measured_depth_m: float
    segment_length_m: float
    section_name: str
    body_torque_increment_n_m: float
    centralizer_torque_increment_n_m: float
    local_torque_increment_n_m: float
    reduced_torsional_load_n_m: float
    section_torsional_stiffness_n_m2: float
    reduced_twist_rate_rad_per_m: float
    reduced_twist_increment_rad: float
    cumulative_reduced_twist_rad: float
    status: str


@dataclass(slots=True)
class TorsionalReducedResultModel:
    status: str
    reduced_torque_accumulation_profile: list[ReducedTorqueAccumulationPointModel]
    torsional_state_profile: list[TorsionalStatePointModel]
    estimated_surface_torsional_load_n_m: float
    maximum_cumulative_reduced_twist_rad: float


def _section_for_name(loaded_case: LoadedCase, section_name: str) -> StringSectionModel:
    for section in loaded_case.string.sections:
        if section.name == section_name:
            return section
    raise ValueError("Reduced torsional model could not resolve the string section name.")


def run_reduced_torsional_model(
    loaded_case: LoadedCase,
    mechanical_profile: list[MechanicalSegmentResultModel],
    torque_profile: list[TorquePointModel],
    *,
    carried_torsional_load_profile_n_m: list[float] | None = None,
) -> TorsionalReducedResultModel:
    if len(mechanical_profile) != len(torque_profile):
        raise ValueError(
            "Reduced torsional model requires mechanical and torque profiles with matching sizes."
        )
    if (
        carried_torsional_load_profile_n_m is not None
        and len(carried_torsional_load_profile_n_m) != len(torque_profile)
    ):
        raise ValueError(
            "Reduced torsional model received a carried torsional-load profile with incompatible size."
        )

    reduced_torque_accumulation_profile: list[ReducedTorqueAccumulationPointModel] = [
        ReducedTorqueAccumulationPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=0.0,
            raw_cumulative_torque_n_m=0.0,
            carried_torsional_load_n_m=0.0,
            status="inactive",
        )
        for segment in mechanical_profile
    ]
    torsional_state_profile: list[TorsionalStatePointModel] = [
        TorsionalStatePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            segment_length_m=segment.segment_length_m,
            section_name=segment.section_name,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=0.0,
            reduced_torsional_load_n_m=0.0,
            section_torsional_stiffness_n_m2=0.0,
            reduced_twist_rate_rad_per_m=0.0,
            reduced_twist_increment_rad=0.0,
            cumulative_reduced_twist_rad=0.0,
            status="inactive",
        )
        for segment in mechanical_profile
    ]

    cumulative_twist_below_rad = 0.0
    maximum_cumulative_reduced_twist_rad = 0.0
    for segment_index in range(len(torque_profile) - 1, -1, -1):
        segment = mechanical_profile[segment_index]
        torque_point = torque_profile[segment_index]
        section = _section_for_name(loaded_case, segment.section_name)

        raw_cumulative_torque_n_m = max(0.0, torque_point.cumulative_torque_n_m)
        carried_torsional_load_n_m = (
            raw_cumulative_torque_n_m
            if carried_torsional_load_profile_n_m is None
            else max(0.0, carried_torsional_load_profile_n_m[segment_index])
        )
        section_torsional_stiffness_n_m2 = max(0.0, section.torsional_stiffness_n_m2)
        reduced_twist_rate_rad_per_m = (
            0.0
            if section_torsional_stiffness_n_m2 <= 1.0e-12
            else carried_torsional_load_n_m / section_torsional_stiffness_n_m2
        )
        reduced_twist_increment_rad = reduced_twist_rate_rad_per_m * segment.segment_length_m
        status = (
            "phase11-reduced-torsional-load-and-twist-state"
            if carried_torsional_load_n_m > 0.0 or raw_cumulative_torque_n_m > 0.0
            else "inactive"
        )
        reduced_torque_accumulation_profile[segment_index] = ReducedTorqueAccumulationPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            body_torque_increment_n_m=torque_point.body_torque_increment_n_m,
            centralizer_torque_increment_n_m=torque_point.centralizer_torque_increment_n_m,
            local_torque_increment_n_m=torque_point.local_torque_increment_n_m,
            raw_cumulative_torque_n_m=raw_cumulative_torque_n_m,
            carried_torsional_load_n_m=carried_torsional_load_n_m,
            status=status,
        )
        torsional_state_profile[segment_index] = TorsionalStatePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            segment_length_m=segment.segment_length_m,
            section_name=segment.section_name,
            body_torque_increment_n_m=torque_point.body_torque_increment_n_m,
            centralizer_torque_increment_n_m=torque_point.centralizer_torque_increment_n_m,
            local_torque_increment_n_m=torque_point.local_torque_increment_n_m,
            reduced_torsional_load_n_m=carried_torsional_load_n_m,
            section_torsional_stiffness_n_m2=section_torsional_stiffness_n_m2,
            reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
            reduced_twist_increment_rad=reduced_twist_increment_rad,
            cumulative_reduced_twist_rad=(
                cumulative_twist_below_rad + (0.5 * reduced_twist_increment_rad)
            ),
            status=status,
        )
        cumulative_twist_below_rad += reduced_twist_increment_rad
        maximum_cumulative_reduced_twist_rad = max(
            maximum_cumulative_reduced_twist_rad,
            abs(torsional_state_profile[segment_index].cumulative_reduced_twist_rad),
        )

    estimated_surface_torsional_load_n_m = (
        torsional_state_profile[0].reduced_torsional_load_n_m if torsional_state_profile else 0.0
    )
    return TorsionalReducedResultModel(
        status=(
            "phase11-reduced-torsional-load-and-twist-state"
            if torsional_state_profile
            else "inactive"
        ),
        reduced_torque_accumulation_profile=reduced_torque_accumulation_profile,
        torsional_state_profile=torsional_state_profile,
        estimated_surface_torsional_load_n_m=estimated_surface_torsional_load_n_m,
        maximum_cumulative_reduced_twist_rad=maximum_cumulative_reduced_twist_rad,
    )
