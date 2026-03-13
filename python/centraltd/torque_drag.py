from __future__ import annotations

from dataclasses import dataclass
import math

from .mechanics import MechanicalSegmentResultModel
from .models import LoadedCase, StringSectionModel
from .torque_drag_body import evaluate_body_torque_drag_contribution
from .torque_drag_centralizer import (
    evaluate_centralizer_torque_contribution_from_details,
)


SUPPORTED_OPERATION_MODES = {"run_in", "pull_out", "rotate_in_place"}


@dataclass(slots=True)
class AxialForcePointModel:
    measured_depth_m: float
    axial_force_n: float


@dataclass(slots=True)
class TorquePointModel:
    measured_depth_m: float
    effective_contact_radius_m: float
    body_torque_increment_n_m: float
    centralizer_torque_increment_n_m: float
    local_torque_increment_n_m: float
    cumulative_torque_n_m: float


@dataclass(slots=True)
class CentralizerFrictionPointModel:
    measured_depth_m: float
    centralizer_axial_friction_n: float
    centralizer_tangential_friction_n: float


@dataclass(slots=True)
class BodyFrictionPointModel:
    measured_depth_m: float
    body_axial_friction_n: float
    body_tangential_friction_n: float


@dataclass(slots=True)
class CentralizerTangentialDirectionPointModel:
    measured_depth_m: float
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    torsional_slip_indicator: float
    torsional_tangential_demand_factor: float
    effective_radial_direction_normal: float
    effective_radial_direction_binormal: float
    tangential_direction_normal: float
    tangential_direction_binormal: float
    projected_contact_normal_n: float
    friction_interaction_scale: float
    status: str


@dataclass(slots=True)
class CentralizerTangentialVectorPointModel:
    measured_depth_m: float
    effective_radial_direction_normal: float
    effective_radial_direction_binormal: float
    tangential_direction_normal: float
    tangential_direction_binormal: float
    tangential_friction_normal_n: float
    tangential_friction_binormal_n: float
    projected_contact_normal_n: float
    friction_interaction_scale: float
    tangential_friction_magnitude_n: float
    status: str


@dataclass(slots=True)
class CentralizerPlacementTorquePointModel:
    source_name: str
    placement_measured_depth_m: float
    measured_depth_m: float
    effective_contact_radius_m: float
    axial_force_ratio: float
    tangential_force_ratio: float
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    torsional_slip_indicator: float
    torsional_tangential_demand_factor: float
    bow_resultant_normal_n: float
    bow_resultant_binormal_n: float
    bow_resultant_magnitude_n: float
    local_contact_direction_normal: float
    local_contact_direction_binormal: float
    effective_radial_direction_normal: float
    effective_radial_direction_binormal: float
    tangential_direction_normal: float
    tangential_direction_binormal: float
    tangential_friction_normal_n: float
    tangential_friction_binormal_n: float
    tangential_friction_magnitude_n: float
    local_contact_weight: float
    direction_alignment_cosine: float
    projected_contact_normal_n: float
    friction_interaction_scale: float
    axial_friction_n: float
    tangential_friction_n: float
    local_torque_increment_n_m: float
    cumulative_torque_n_m: float
    status: str


@dataclass(slots=True)
class LocalTangentialInteractionStatePointModel:
    measured_depth_m: float
    reduced_torsional_load_n_m: float
    reduced_twist_rate_rad_per_m: float
    body_effective_contact_radius_m: float
    body_torsional_slip_indicator: float
    body_tangential_mobilization: float
    body_tangential_demand_factor: float
    body_tangential_traction_indicator: float
    body_tangential_regime: str
    body_feedback_applied: bool
    centralizer_effective_contact_radius_m: float
    centralizer_torsional_slip_indicator: float
    centralizer_tangential_mobilization: float
    centralizer_tangential_demand_factor: float
    centralizer_tangential_traction_indicator: float
    centralizer_tangential_regime: str
    centralizer_feedback_applied: bool
    local_tangential_mobilization: float
    local_tangential_traction_indicator: float
    local_tangential_regime: str
    body_status: str
    centralizer_status: str
    status: str


@dataclass(slots=True)
class TorquePartitionSummaryModel:
    body_surface_torque_n_m: float
    centralizer_surface_torque_n_m: float
    total_surface_torque_n_m: float
    body_axial_friction_sum_n: float
    centralizer_axial_friction_sum_n: float
    body_tangential_friction_sum_n: float
    centralizer_tangential_friction_sum_n: float
    status: str


@dataclass(slots=True)
class TorqueDragResultModel:
    status: str
    operation_mode: str
    axial_force_run_in_profile: list[AxialForcePointModel]
    axial_force_pull_out_profile: list[AxialForcePointModel]
    torque_profile: list[TorquePointModel]
    body_axial_friction_profile: list[BodyFrictionPointModel]
    body_torque_profile: list[TorquePointModel]
    centralizer_axial_friction_profile: list[CentralizerFrictionPointModel]
    centralizer_tangential_friction_profile: list[CentralizerFrictionPointModel]
    centralizer_tangential_direction_profile: list[CentralizerTangentialDirectionPointModel]
    centralizer_tangential_friction_vector_profile: list[CentralizerTangentialVectorPointModel]
    centralizer_torque_profile: list[TorquePointModel]
    centralizer_torque_breakdown_profile: list[list[CentralizerPlacementTorquePointModel]]
    local_tangential_interaction_state: list[LocalTangentialInteractionStatePointModel]
    torque_partition_summary: TorquePartitionSummaryModel
    hookload_run_in_n: float
    hookload_pull_out_n: float
    drag_run_in_n: float
    drag_pull_out_n: float
    estimated_surface_torque_n_m: float | None


def is_supported_operation_mode(operation_mode: str) -> bool:
    return operation_mode in SUPPORTED_OPERATION_MODES


def _section_for_name(loaded_case: LoadedCase, section_name: str) -> StringSectionModel:
    for section in loaded_case.string.sections:
        if section.name == section_name:
            return section
    raise ValueError("Torque and drag propagation could not resolve the string section name.")


def run_torque_drag_baseline(
    loaded_case: LoadedCase,
    mechanical_profile: list[MechanicalSegmentResultModel],
    *,
    reference_buoyant_hookload_n: float,
    reduced_torsional_load_profile_n_m: list[float] | None = None,
    reduced_twist_rate_profile_rad_per_m: list[float] | None = None,
) -> TorqueDragResultModel:
    if not is_supported_operation_mode(loaded_case.operation_mode):
        raise ValueError("Unsupported operation mode for reduced torque and drag.")
    if (
        reduced_torsional_load_profile_n_m is not None
        and len(reduced_torsional_load_profile_n_m) != len(mechanical_profile)
    ):
        raise ValueError(
            "Reduced torsional-load feedback profile must match the mechanical profile size."
        )
    if (
        reduced_twist_rate_profile_rad_per_m is not None
        and len(reduced_twist_rate_profile_rad_per_m) != len(mechanical_profile)
    ):
        raise ValueError(
            "Reduced twist-rate feedback profile must match the mechanical profile size."
        )

    axial_force_run_in_profile: list[AxialForcePointModel] = [
        AxialForcePointModel(measured_depth_m=segment.measured_depth_center_m, axial_force_n=0.0)
        for segment in mechanical_profile
    ]
    axial_force_pull_out_profile: list[AxialForcePointModel] = [
        AxialForcePointModel(measured_depth_m=segment.measured_depth_center_m, axial_force_n=0.0)
        for segment in mechanical_profile
    ]
    torque_profile: list[TorquePointModel] = [
        TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=0.0,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=0.0,
            cumulative_torque_n_m=0.0,
        )
        for segment in mechanical_profile
    ]
    body_axial_friction_profile: list[BodyFrictionPointModel] = [
        BodyFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            body_axial_friction_n=0.0,
            body_tangential_friction_n=0.0,
        )
        for segment in mechanical_profile
    ]
    body_torque_profile: list[TorquePointModel] = [
        TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=0.0,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=0.0,
            cumulative_torque_n_m=0.0,
        )
        for segment in mechanical_profile
    ]
    centralizer_axial_friction_profile: list[CentralizerFrictionPointModel] = [
        CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=0.0,
            centralizer_tangential_friction_n=0.0,
        )
        for segment in mechanical_profile
    ]
    centralizer_tangential_friction_profile: list[CentralizerFrictionPointModel] = [
        CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=0.0,
            centralizer_tangential_friction_n=0.0,
        )
        for segment in mechanical_profile
    ]
    centralizer_tangential_direction_profile: list[
        CentralizerTangentialDirectionPointModel
    ] = [
        CentralizerTangentialDirectionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            reduced_torsional_load_n_m=0.0,
            reduced_twist_rate_rad_per_m=0.0,
            torsional_slip_indicator=0.0,
            torsional_tangential_demand_factor=1.0,
            effective_radial_direction_normal=0.0,
            effective_radial_direction_binormal=0.0,
            tangential_direction_normal=0.0,
            tangential_direction_binormal=0.0,
            projected_contact_normal_n=0.0,
            friction_interaction_scale=1.0,
            status="inactive",
        )
        for segment in mechanical_profile
    ]
    centralizer_tangential_friction_vector_profile: list[
        CentralizerTangentialVectorPointModel
    ] = [
        CentralizerTangentialVectorPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_radial_direction_normal=0.0,
            effective_radial_direction_binormal=0.0,
            tangential_direction_normal=0.0,
            tangential_direction_binormal=0.0,
            tangential_friction_normal_n=0.0,
            tangential_friction_binormal_n=0.0,
            projected_contact_normal_n=0.0,
            friction_interaction_scale=1.0,
            tangential_friction_magnitude_n=0.0,
            status="inactive",
        )
        for segment in mechanical_profile
    ]
    centralizer_torque_profile: list[TorquePointModel] = [
        TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=0.0,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=0.0,
            cumulative_torque_n_m=0.0,
        )
        for segment in mechanical_profile
    ]
    centralizer_torque_breakdown_profile: list[list[CentralizerPlacementTorquePointModel]] = [
        [] for _ in mechanical_profile
    ]
    local_tangential_interaction_state: list[LocalTangentialInteractionStatePointModel] = [
        LocalTangentialInteractionStatePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            reduced_torsional_load_n_m=0.0,
            reduced_twist_rate_rad_per_m=0.0,
            body_effective_contact_radius_m=0.0,
            body_torsional_slip_indicator=0.0,
            body_tangential_mobilization=0.0,
            body_tangential_demand_factor=1.0,
            body_tangential_traction_indicator=0.0,
            body_tangential_regime="inactive",
            body_feedback_applied=False,
            centralizer_effective_contact_radius_m=0.0,
            centralizer_torsional_slip_indicator=0.0,
            centralizer_tangential_mobilization=0.0,
            centralizer_tangential_demand_factor=1.0,
            centralizer_tangential_traction_indicator=0.0,
            centralizer_tangential_regime="inactive",
            centralizer_feedback_applied=False,
            local_tangential_mobilization=0.0,
            local_tangential_traction_indicator=0.0,
            local_tangential_regime="inactive",
            body_status="inactive",
            centralizer_status="inactive",
            status="inactive",
        )
        for segment in mechanical_profile
    ]

    axial_force_run_in_below_n = 0.0
    axial_force_pull_out_below_n = 0.0
    cumulative_surface_torque_n_m = 0.0
    cumulative_body_surface_torque_n_m = 0.0
    cumulative_centralizer_surface_torque_n_m = 0.0
    body_axial_friction_sum_n = 0.0
    centralizer_axial_friction_sum_n = 0.0
    body_tangential_friction_sum_n = 0.0
    centralizer_tangential_friction_sum_n = 0.0
    cumulative_placement_torque_n_m: dict[tuple[str, float], float] = {}

    for segment_index in range(len(mechanical_profile) - 1, -1, -1):
        segment = mechanical_profile[segment_index]
        section = _section_for_name(loaded_case, segment.section_name)
        reduced_torsional_load_n_m = (
            0.0
            if reduced_torsional_load_profile_n_m is None
            else max(0.0, reduced_torsional_load_profile_n_m[segment_index])
        )
        reduced_twist_rate_rad_per_m = (
            0.0
            if reduced_twist_rate_profile_rad_per_m is None
            else abs(reduced_twist_rate_profile_rad_per_m[segment_index])
        )

        tangential_weight_n = (
            segment.effective_line_weight_n_per_m
            * math.cos(segment.inclination_rad)
            * segment.segment_length_m
        )
        # `normal_reaction_estimate_n` is the resultant segment reaction [N], not
        # a distributed load [N/m]. This keeps `mu * N` in [N] for axial drag
        # and `mu * N * r` in [N.m] for torque.
        body_contribution = evaluate_body_torque_drag_contribution(
            segment,
            section,
            reduced_torsional_load_n_m=reduced_torsional_load_n_m,
            reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
        )
        centralizer_contribution = evaluate_centralizer_torque_contribution_from_details(
            list(segment.centralizer_torque_details),
            reduced_torsional_load_n_m=reduced_torsional_load_n_m,
            reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
        )
        body_contact_active = segment.body_normal_reaction_estimate_n > 0.0
        body_friction_force_n = body_contribution.axial_friction_n
        centralizer_axial_friction_n = max(0.0, centralizer_contribution.axial_friction_n)
        # Sign convention:
        # - run_in/slackoff: downward motion, friction acts upward, reducing
        #   the local hookload increment.
        # - pull_out/pickup: upward motion, friction acts downward, increasing
        #   the local hookload increment.
        delta_run_in_n = tangential_weight_n - body_friction_force_n - centralizer_axial_friction_n
        delta_pull_out_n = tangential_weight_n + body_friction_force_n + centralizer_axial_friction_n

        axial_force_run_in_profile[segment_index] = AxialForcePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            axial_force_n=axial_force_run_in_below_n + (0.5 * delta_run_in_n),
        )
        axial_force_pull_out_profile[segment_index] = AxialForcePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            axial_force_n=axial_force_pull_out_below_n + (0.5 * delta_pull_out_n),
        )
        axial_force_run_in_below_n += delta_run_in_n
        axial_force_pull_out_below_n += delta_pull_out_n

        body_axial_friction_profile[segment_index] = BodyFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            body_axial_friction_n=body_contribution.axial_friction_n,
            body_tangential_friction_n=body_contribution.tangential_friction_n,
        )
        centralizer_axial_friction_profile[segment_index] = CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=centralizer_axial_friction_n,
            centralizer_tangential_friction_n=max(
                0.0,
                centralizer_contribution.tangential_friction_n,
            ),
        )
        centralizer_tangential_friction_profile[segment_index] = CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=centralizer_axial_friction_n,
            centralizer_tangential_friction_n=max(
                0.0,
                centralizer_contribution.tangential_friction_n,
            ),
        )
        centralizer_tangential_direction_profile[segment_index] = (
            CentralizerTangentialDirectionPointModel(
                measured_depth_m=segment.measured_depth_center_m,
                reduced_torsional_load_n_m=centralizer_contribution.reduced_torsional_load_n_m,
                reduced_twist_rate_rad_per_m=(
                    centralizer_contribution.reduced_twist_rate_rad_per_m
                ),
                torsional_slip_indicator=centralizer_contribution.torsional_slip_indicator,
                torsional_tangential_demand_factor=(
                    centralizer_contribution.torsional_tangential_demand_factor
                ),
                effective_radial_direction_normal=(
                    centralizer_contribution.effective_radial_direction_n_b[0]
                ),
                effective_radial_direction_binormal=(
                    centralizer_contribution.effective_radial_direction_n_b[1]
                ),
                tangential_direction_normal=centralizer_contribution.tangential_direction_n_b[0],
                tangential_direction_binormal=centralizer_contribution.tangential_direction_n_b[1],
                projected_contact_normal_n=centralizer_contribution.projected_contact_normal_n,
                friction_interaction_scale=centralizer_contribution.friction_interaction_scale,
                status=centralizer_contribution.status,
            )
        )
        centralizer_tangential_friction_vector_profile[segment_index] = (
            CentralizerTangentialVectorPointModel(
                measured_depth_m=segment.measured_depth_center_m,
                effective_radial_direction_normal=(
                    centralizer_contribution.effective_radial_direction_n_b[0]
                ),
                effective_radial_direction_binormal=(
                    centralizer_contribution.effective_radial_direction_n_b[1]
                ),
                tangential_direction_normal=centralizer_contribution.tangential_direction_n_b[0],
                tangential_direction_binormal=centralizer_contribution.tangential_direction_n_b[1],
                tangential_friction_normal_n=(
                    centralizer_contribution.tangential_friction_vector_n_b[0]
                ),
                tangential_friction_binormal_n=(
                    centralizer_contribution.tangential_friction_vector_n_b[1]
                ),
                projected_contact_normal_n=centralizer_contribution.projected_contact_normal_n,
                friction_interaction_scale=centralizer_contribution.friction_interaction_scale,
                tangential_friction_magnitude_n=(
                    centralizer_contribution.tangential_friction_vector_magnitude_n
                ),
                status=centralizer_contribution.status,
            )
        )
        local_tangential_interaction_state[segment_index] = (
            LocalTangentialInteractionStatePointModel(
                measured_depth_m=segment.measured_depth_center_m,
                reduced_torsional_load_n_m=reduced_torsional_load_n_m,
                reduced_twist_rate_rad_per_m=reduced_twist_rate_rad_per_m,
                body_effective_contact_radius_m=body_contribution.contact_radius_m,
                body_torsional_slip_indicator=body_contribution.torsional_slip_indicator,
                body_tangential_mobilization=body_contribution.tangential_mobilization,
                body_tangential_demand_factor=body_contribution.tangential_demand_factor,
                body_tangential_traction_indicator=(
                    body_contribution.tangential_traction_indicator
                ),
                body_tangential_regime=body_contribution.tangential_regime,
                body_feedback_applied=body_contribution.feedback_applied,
                centralizer_effective_contact_radius_m=(
                    centralizer_contribution.effective_contact_radius_m
                ),
                centralizer_torsional_slip_indicator=(
                    centralizer_contribution.torsional_slip_indicator
                ),
                centralizer_tangential_mobilization=(
                    centralizer_contribution.tangential_mobilization
                ),
                centralizer_tangential_demand_factor=(
                    centralizer_contribution.torsional_tangential_demand_factor
                ),
                centralizer_tangential_traction_indicator=(
                    centralizer_contribution.tangential_traction_indicator
                ),
                centralizer_tangential_regime=(
                    centralizer_contribution.tangential_regime
                ),
                centralizer_feedback_applied=(
                    centralizer_contribution.active
                    and centralizer_contribution.torsional_slip_indicator > 0.0
                ),
                local_tangential_mobilization=max(
                    body_contribution.tangential_mobilization,
                    centralizer_contribution.tangential_mobilization,
                ),
                local_tangential_traction_indicator=max(
                    body_contribution.tangential_traction_indicator,
                    centralizer_contribution.tangential_traction_indicator,
                ),
                local_tangential_regime=(
                    "inactive"
                    if not (body_contact_active or centralizer_contribution.active)
                    else "reduced-partial-adhesion-proxy"
                    if max(
                        body_contribution.tangential_traction_indicator,
                        centralizer_contribution.tangential_traction_indicator,
                    )
                    < 0.25
                    else "reduced-mobilizing-traction"
                    if max(
                        body_contribution.tangential_traction_indicator,
                        centralizer_contribution.tangential_traction_indicator,
                    )
                    < 0.95
                    else "reduced-slip-limited-traction"
                ),
                body_status=body_contribution.status,
                centralizer_status=(
                    centralizer_contribution.status
                    if centralizer_contribution.active
                    else "inactive"
                ),
                status=(
                    "phase14-reduced-local-body-centralizer-tangential-interaction-state"
                    if body_contact_active
                    or centralizer_contribution.active
                    else "inactive"
                ),
            )
        )

        body_torque_increment_n_m = body_contribution.torque_increment_n_m
        centralizer_torque_increment_n_m = max(
            0.0,
            centralizer_contribution.torque_increment_n_m,
        )
        local_torque_increment_n_m = body_torque_increment_n_m + centralizer_torque_increment_n_m
        body_torque_profile[segment_index] = TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=body_contribution.contact_radius_m,
            body_torque_increment_n_m=body_torque_increment_n_m,
            centralizer_torque_increment_n_m=0.0,
            local_torque_increment_n_m=body_torque_increment_n_m,
            cumulative_torque_n_m=(
                cumulative_body_surface_torque_n_m + (0.5 * body_torque_increment_n_m)
            ),
        )
        centralizer_torque_profile[segment_index] = TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=centralizer_contribution.effective_contact_radius_m,
            body_torque_increment_n_m=0.0,
            centralizer_torque_increment_n_m=centralizer_torque_increment_n_m,
            local_torque_increment_n_m=centralizer_torque_increment_n_m,
            cumulative_torque_n_m=(
                cumulative_centralizer_surface_torque_n_m + (0.5 * centralizer_torque_increment_n_m)
            ),
        )
        torque_profile[segment_index] = TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=max(
                body_contribution.contact_radius_m,
                centralizer_contribution.effective_contact_radius_m,
            ),
            body_torque_increment_n_m=body_torque_increment_n_m,
            centralizer_torque_increment_n_m=centralizer_torque_increment_n_m,
            local_torque_increment_n_m=local_torque_increment_n_m,
            cumulative_torque_n_m=cumulative_surface_torque_n_m + (0.5 * local_torque_increment_n_m),
        )
        breakdown_points: list[CentralizerPlacementTorquePointModel] = []
        for detail in centralizer_contribution.placement_contributions:
            placement_key = (detail.source_name, detail.placement_measured_depth_m)
            cumulative_before_n_m = cumulative_placement_torque_n_m.get(placement_key, 0.0)
            breakdown_points.append(
                CentralizerPlacementTorquePointModel(
                    source_name=detail.source_name,
                    placement_measured_depth_m=detail.placement_measured_depth_m,
                    measured_depth_m=segment.measured_depth_center_m,
                    effective_contact_radius_m=detail.effective_contact_radius_m,
                    axial_force_ratio=detail.axial_force_ratio,
                    tangential_force_ratio=detail.tangential_force_ratio,
                    reduced_torsional_load_n_m=detail.reduced_torsional_load_n_m,
                    reduced_twist_rate_rad_per_m=detail.reduced_twist_rate_rad_per_m,
                    torsional_slip_indicator=detail.torsional_slip_indicator,
                    torsional_tangential_demand_factor=(
                        detail.torsional_tangential_demand_factor
                    ),
                    bow_resultant_normal_n=detail.bow_resultant_vector_n_b[0],
                    bow_resultant_binormal_n=detail.bow_resultant_vector_n_b[1],
                    bow_resultant_magnitude_n=detail.bow_resultant_magnitude_n,
                    local_contact_direction_normal=detail.local_contact_direction_n_b[0],
                    local_contact_direction_binormal=detail.local_contact_direction_n_b[1],
                    effective_radial_direction_normal=detail.effective_radial_direction_n_b[0],
                    effective_radial_direction_binormal=detail.effective_radial_direction_n_b[1],
                    tangential_direction_normal=detail.tangential_direction_n_b[0],
                    tangential_direction_binormal=detail.tangential_direction_n_b[1],
                    tangential_friction_normal_n=detail.tangential_friction_vector_n_b[0],
                    tangential_friction_binormal_n=detail.tangential_friction_vector_n_b[1],
                    tangential_friction_magnitude_n=detail.tangential_friction_vector_magnitude_n,
                    local_contact_weight=detail.local_contact_weight,
                    direction_alignment_cosine=detail.direction_alignment_cosine,
                    projected_contact_normal_n=detail.projected_contact_normal_n,
                    friction_interaction_scale=detail.friction_interaction_scale,
                    axial_friction_n=detail.axial_friction_n,
                    tangential_friction_n=detail.tangential_friction_n,
                    local_torque_increment_n_m=detail.torque_increment_n_m,
                    cumulative_torque_n_m=(
                        cumulative_before_n_m + (0.5 * detail.torque_increment_n_m)
                    ),
                    status=detail.status,
                )
            )
            cumulative_placement_torque_n_m[placement_key] = (
                cumulative_before_n_m + detail.torque_increment_n_m
            )
        centralizer_torque_breakdown_profile[segment_index] = breakdown_points
        cumulative_surface_torque_n_m += local_torque_increment_n_m
        cumulative_body_surface_torque_n_m += body_torque_increment_n_m
        cumulative_centralizer_surface_torque_n_m += centralizer_torque_increment_n_m
        body_axial_friction_sum_n += body_contribution.axial_friction_n
        centralizer_axial_friction_sum_n += centralizer_axial_friction_n
        body_tangential_friction_sum_n += body_contribution.tangential_friction_n
        centralizer_tangential_friction_sum_n += max(
            0.0,
            centralizer_contribution.tangential_friction_n,
        )

    hookload_run_in_n = max(0.0, axial_force_run_in_below_n)
    hookload_pull_out_n = max(0.0, axial_force_pull_out_below_n)
    return TorqueDragResultModel(
        status="phase14-reduced-unified-local-tangential-torque-baseline",
        operation_mode=loaded_case.operation_mode,
        axial_force_run_in_profile=axial_force_run_in_profile,
        axial_force_pull_out_profile=axial_force_pull_out_profile,
        torque_profile=torque_profile,
        body_axial_friction_profile=body_axial_friction_profile,
        body_torque_profile=body_torque_profile,
        centralizer_axial_friction_profile=centralizer_axial_friction_profile,
        centralizer_tangential_friction_profile=centralizer_tangential_friction_profile,
        centralizer_tangential_direction_profile=centralizer_tangential_direction_profile,
        centralizer_tangential_friction_vector_profile=(
            centralizer_tangential_friction_vector_profile
        ),
        centralizer_torque_profile=centralizer_torque_profile,
        centralizer_torque_breakdown_profile=centralizer_torque_breakdown_profile,
        local_tangential_interaction_state=local_tangential_interaction_state,
        torque_partition_summary=TorquePartitionSummaryModel(
            body_surface_torque_n_m=cumulative_body_surface_torque_n_m,
            centralizer_surface_torque_n_m=cumulative_centralizer_surface_torque_n_m,
            total_surface_torque_n_m=cumulative_surface_torque_n_m,
            body_axial_friction_sum_n=body_axial_friction_sum_n,
            centralizer_axial_friction_sum_n=centralizer_axial_friction_sum_n,
            body_tangential_friction_sum_n=body_tangential_friction_sum_n,
            centralizer_tangential_friction_sum_n=centralizer_tangential_friction_sum_n,
            status="phase14-reduced-body-centralizer-torque-partition",
        ),
        hookload_run_in_n=hookload_run_in_n,
        hookload_pull_out_n=hookload_pull_out_n,
        drag_run_in_n=max(0.0, reference_buoyant_hookload_n - hookload_run_in_n),
        drag_pull_out_n=max(0.0, hookload_pull_out_n - reference_buoyant_hookload_n),
        estimated_surface_torque_n_m=cumulative_surface_torque_n_m,
    )
