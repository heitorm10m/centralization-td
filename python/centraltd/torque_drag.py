from __future__ import annotations

from dataclasses import dataclass
import math

from .mechanics import MechanicalSegmentResultModel
from .models import LoadedCase, StringSectionModel


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
class TorqueDragResultModel:
    status: str
    operation_mode: str
    axial_force_run_in_profile: list[AxialForcePointModel]
    axial_force_pull_out_profile: list[AxialForcePointModel]
    torque_profile: list[TorquePointModel]
    centralizer_axial_friction_profile: list[CentralizerFrictionPointModel]
    centralizer_tangential_friction_profile: list[CentralizerFrictionPointModel]
    centralizer_torque_profile: list[TorquePointModel]
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
) -> TorqueDragResultModel:
    if not is_supported_operation_mode(loaded_case.operation_mode):
        raise ValueError("Unsupported operation mode for reduced torque and drag.")

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

    axial_force_run_in_below_n = 0.0
    axial_force_pull_out_below_n = 0.0
    cumulative_surface_torque_n_m = 0.0
    cumulative_centralizer_surface_torque_n_m = 0.0

    for segment_index in range(len(mechanical_profile) - 1, -1, -1):
        segment = mechanical_profile[segment_index]
        section = _section_for_name(loaded_case, segment.section_name)

        tangential_weight_n = (
            segment.effective_line_weight_n_per_m
            * math.cos(segment.inclination_rad)
            * segment.segment_length_m
        )
        # `normal_reaction_estimate_n` is the resultant segment reaction [N], not
        # a distributed load [N/m]. This keeps `mu * N` in [N] for axial drag
        # and `mu * N * r` in [N.m] for torque.
        body_friction_force_n = (
            max(0.0, section.friction_coefficient)
            * max(0.0, segment.body_normal_reaction_estimate_n)
        )
        centralizer_axial_friction_n = max(0.0, segment.centralizer_axial_friction_n)
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

        centralizer_axial_friction_profile[segment_index] = CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=centralizer_axial_friction_n,
            centralizer_tangential_friction_n=max(0.0, segment.centralizer_tangential_friction_n),
        )
        centralizer_tangential_friction_profile[segment_index] = CentralizerFrictionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            centralizer_axial_friction_n=centralizer_axial_friction_n,
            centralizer_tangential_friction_n=max(0.0, segment.centralizer_tangential_friction_n),
        )

        body_contact_radius_m = max(0.0, 0.5 * section.outer_diameter_m)
        body_torque_increment_n_m = body_friction_force_n * body_contact_radius_m
        centralizer_torque_increment_n_m = max(0.0, segment.centralizer_torque_increment_n_m)
        local_torque_increment_n_m = body_torque_increment_n_m + centralizer_torque_increment_n_m
        centralizer_torque_profile[segment_index] = TorquePointModel(
            measured_depth_m=segment.measured_depth_center_m,
            effective_contact_radius_m=segment.centralizer_effective_contact_radius_m,
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
                body_contact_radius_m,
                segment.centralizer_effective_contact_radius_m,
            ),
            body_torque_increment_n_m=body_torque_increment_n_m,
            centralizer_torque_increment_n_m=centralizer_torque_increment_n_m,
            local_torque_increment_n_m=local_torque_increment_n_m,
            cumulative_torque_n_m=cumulative_surface_torque_n_m + (0.5 * local_torque_increment_n_m),
        )
        cumulative_surface_torque_n_m += local_torque_increment_n_m
        cumulative_centralizer_surface_torque_n_m += centralizer_torque_increment_n_m

    hookload_run_in_n = max(0.0, axial_force_run_in_below_n)
    hookload_pull_out_n = max(0.0, axial_force_pull_out_below_n)
    return TorqueDragResultModel(
        status="phase9-reduced-bow-spring-td-baseline",
        operation_mode=loaded_case.operation_mode,
        axial_force_run_in_profile=axial_force_run_in_profile,
        axial_force_pull_out_profile=axial_force_pull_out_profile,
        torque_profile=torque_profile,
        centralizer_axial_friction_profile=centralizer_axial_friction_profile,
        centralizer_tangential_friction_profile=centralizer_tangential_friction_profile,
        centralizer_torque_profile=centralizer_torque_profile,
        hookload_run_in_n=hookload_run_in_n,
        hookload_pull_out_n=hookload_pull_out_n,
        drag_run_in_n=max(0.0, reference_buoyant_hookload_n - hookload_run_in_n),
        drag_pull_out_n=max(0.0, hookload_pull_out_n - reference_buoyant_hookload_n),
        estimated_surface_torque_n_m=cumulative_surface_torque_n_m,
    )
