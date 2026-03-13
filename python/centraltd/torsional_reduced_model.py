from __future__ import annotations

from dataclasses import dataclass

from .mechanics import MechanicalSegmentResultModel
from .models import LoadedCase, StringSectionModel
from .torque_drag import TorquePointModel


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
        reduced_twist_increment_rad = (
            reduced_twist_rate_rad_per_m * segment.segment_length_m
        )
        status = (
            "phase11-reduced-torsional-load-and-twist-state"
            if carried_torsional_load_n_m > 0.0 or raw_cumulative_torque_n_m > 0.0
            else "inactive"
        )
        reduced_torque_accumulation_profile[segment_index] = (
            ReducedTorqueAccumulationPointModel(
                measured_depth_m=segment.measured_depth_center_m,
                body_torque_increment_n_m=torque_point.body_torque_increment_n_m,
                centralizer_torque_increment_n_m=torque_point.centralizer_torque_increment_n_m,
                local_torque_increment_n_m=torque_point.local_torque_increment_n_m,
                raw_cumulative_torque_n_m=raw_cumulative_torque_n_m,
                carried_torsional_load_n_m=carried_torsional_load_n_m,
                status=status,
            )
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
        torsional_state_profile[0].reduced_torsional_load_n_m
        if torsional_state_profile
        else 0.0
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
