from __future__ import annotations

from pathlib import Path
import math
from typing import Any

from .physics.bow_spring import (
    bow_force_magnitude_n,
    bow_reference_deflection_m,
    centralizer_axial_force_ratio,
    centralizer_effective_contact_diameter_m,
    centralizer_running_force_ratio,
    centralizer_tangential_force_ratio,
    equivalent_bow_support_stiffness_n_per_m,
    resolved_blade_power_law_k,
)
from .io.io import load_case_bundle, write_json
from .solver.coupling import run_coupled_global_baseline
from .solver.mechanics import (
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
)
from .models import (
    CentralizerSummaryModel,
    LoadedCase,
    Nodal6DOFOutputModel,
    StringSummaryModel,
    TrajectorySummaryModel,
)
from .torque_drag import (
    AxialForcePointModel,
    TorquePointModel,
)


PHASE14_WARNINGS = [
    "Phase 14 trajectory coordinates are still approximated by balanced-tangent integration of MD, inclination, and azimuth.",
    "The local trajectory frame now uses the Dao et al. Eq. 11 construction with a vertical-section eX/eY fallback for continuity.",
    "The runtime now exports a 6-DOF nodal state [u, v, w, theta_x, theta_y, theta_z] for the active mechanical path, while several downstream contact and torque/drag closures still retain reduced-model assumptions.",
    "The bending term still uses the simply supported beam equivalence delta_max = 5 q L^4 / (384 E I), so the 384/5 factor is a reduced structural hypothesis.",
    "Contact still uses reduced vector active-set penalty iterations in the annulus, but detailed centralizer support loads are now post-processed bow by bow in the local frame.",
    "Each bow is distributed uniformly in angle from the centralizer angular reference and uses the reduced deflection law delta_i = max(0, e . r_i - (c_support + c_inner)).",
    "The bow constitutive law is reduced to F_i = k_blade * delta_i^p, with optional direct k/p input or fallback calibration from nominal restoring force at a reference deflection.",
    "Bow resultants are reported in the local normal/binormal frame, while transformed trajectory coordinates remain stored in north/east/TVD for geometry reporting.",
    "Pipe-body axial drag still uses mu * N_body, while both pipe-body and centralizer tangential torque terms now pass through a shared reduced local tangential-state model with slip indicator, mobilization, traction indicator, and regime fields before their body- or centralizer-specific laws are applied.",
    "Centralizer axial friction and tangential torque now use separate reduced force-ratio parameters, each of which may be explicit in YAML or fall back to nominal_running_force_n / nominal_restoring_force_n when no dedicated ratio is supplied.",
    "The reduced centralizer tangential magnitude now comes from the bow-resultant projection onto an effective local radial/contact direction, not only from the raw resultant magnitude.",
    "Centralizer axial and tangential reduced demands now share a combined friction-budget cap, so stronger tangential demand can reduce the remaining axial centralizer friction capacity.",
    "A reduced torsional-load profile is now carried through the coupling loop and converted into a GJ-based twist indicator; that carried state now feeds a shared reduced local tangential-state model through bounded |twist_rate| * r indicators for both pipe body and centralizer, but this is still not a full torsional beam solve or a 6-DOF rotational formulation.",
    "Run in/slackoff subtracts body plus centralizer axial friction from the local hookload increment because friction opposes downward motion; pull out/pickup adds them because friction opposes upward motion.",
    "Pipe-body and centralizer contributions are exported separately for contact, axial friction, tangential friction, and torque accumulation.",
    "The reduced coupling loop now requires axial, torque, and carried torsional-load updates to fall below their tolerances, and the local torsional feedback now updates a shared reduced tangential-state layer that then feeds both pipe-body and centralizer laws while only the centralizer tangential demand still feeds back into the axial profile through the reduced axial-tangential friction budget.",
    "Detailed bows are modeled individually, but the structural solve is still not a full 6-DOF beam/contact formulation and the global friction law remains reduced.",
]

PHASE11_TODOS = [
    "TODO: evolve the reduced 2-DOF transverse model toward a fuller 3D beam formulation with rotations and torsion.",
    "TODO: refine vector contact with stronger nonlinear iteration and wall-reaction handling.",
    "TODO: calibrate the bow-spring power-law parameters against manufacturer/API data.",
    "TODO: calibrate tangential centralizer torque factors independently from the axial running-force proxy.",
    "TODO: refine axial drag with stronger bidirectional coupling between contact state and friction propagation.",
    "TODO: extend the reduced torsional state toward fuller bow-resolved nonlinear workflows, torsional/contact feedback, and eventually a stronger structural torsion solve.",
]

PHASE10_VALIDATION_STATUS = "phase10-benchmark-calibration-infrastructure"
GLOBAL_SOLVER_UPDATE_TOLERANCE_M = 1.0e-8


def load_case(case_path: str | Path) -> LoadedCase:
    return load_case_bundle(case_path)


def _trajectory_summary_to_dict(summary: TrajectorySummaryModel | Any) -> dict[str, Any]:
    return {
        "point_count": summary.point_count,
        "start_measured_depth_m": summary.start_measured_depth_m,
        "final_measured_depth_m": summary.final_measured_depth_m,
        "total_course_length_m": summary.total_course_length_m,
        "vertical_depth_m": summary.vertical_depth_m,
        "lateral_displacement_m": summary.lateral_displacement_m,
        "max_inclination_rad": summary.max_inclination_rad,
        "max_curvature_rad_per_m": summary.max_curvature_rad_per_m,
        "coordinates_are_approximate": summary.coordinates_are_approximate,
    }


def _string_summary_to_dict(summary: StringSummaryModel | Any) -> dict[str, Any]:
    return {
        "section_count": summary.section_count,
        "total_length_m": summary.total_length_m,
        "total_weight_n": summary.total_weight_n,
        "total_effective_weight_n": summary.total_effective_weight_n,
        "max_outer_diameter_m": summary.max_outer_diameter_m,
        "min_inner_diameter_m": summary.min_inner_diameter_m,
        "average_friction_coefficient": summary.average_friction_coefficient,
        "average_density_kg_per_m3": summary.average_density_kg_per_m3,
    }


def _centralizer_summary_to_dict(summary: CentralizerSummaryModel | Any) -> dict[str, Any]:
    return {
        "spec_count": summary.spec_count,
        "explicit_installation_count": summary.explicit_installation_count,
        "count_hint_total": summary.count_hint_total,
        "spacing_based_installation_estimate": summary.spacing_based_installation_estimate,
        "expanded_installation_count": summary.expanded_installation_count,
        "max_outer_diameter_m": summary.max_outer_diameter_m,
        "min_nominal_radial_clearance_m": summary.min_nominal_radial_clearance_m,
    }


def _mechanical_summary_to_dict(summary: MechanicalSummaryModel | Any) -> dict[str, Any]:
    return {
        "segment_count": summary.segment_count,
        "target_segment_length_m": summary.target_segment_length_m,
        "global_solver_iteration_count": summary.global_solver_iteration_count,
        "global_solver_final_update_norm_m": summary.global_solver_final_update_norm_m,
        "top_effective_axial_load_n": summary.top_effective_axial_load_n,
        "minimum_effective_axial_load_n": summary.minimum_effective_axial_load_n,
        "maximum_effective_axial_load_n": summary.maximum_effective_axial_load_n,
        "maximum_bending_moment_n_m": summary.maximum_bending_moment_n_m,
        "maximum_bending_stress_pa": summary.maximum_bending_stress_pa,
        "maximum_bending_strain_estimate": summary.maximum_bending_strain_estimate,
        "maximum_equivalent_lateral_load_n_per_m": summary.maximum_equivalent_lateral_load_n_per_m,
        "maximum_eccentricity_estimate_m": summary.maximum_eccentricity_estimate_m,
        "maximum_eccentricity_ratio": summary.maximum_eccentricity_ratio,
        "minimum_standoff_estimate": summary.minimum_standoff_estimate,
        "maximum_normal_reaction_estimate_n": summary.maximum_normal_reaction_estimate_n,
        "contact_segment_count": summary.contact_segment_count,
        "support_contact_segment_count": summary.support_contact_segment_count,
        "pipe_body_contact_segment_count": summary.pipe_body_contact_segment_count,
    }


def _section_summaries_to_dict(section_summaries: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "name": section_summary.name,
            "md_start_m": section_summary.md_start_m,
            "md_end_m": section_summary.md_end_m,
            "length_m": section_summary.length_m,
            "outer_diameter_m": section_summary.outer_diameter_m,
            "inner_diameter_m": section_summary.inner_diameter_m,
            "linear_weight_n_per_m": section_summary.linear_weight_n_per_m,
            "friction_coefficient": section_summary.friction_coefficient,
            "nominal_radial_clearance_m": section_summary.nominal_radial_clearance_m,
        }
        for section_summary in section_summaries
    ]


def _mechanical_profile_to_dict(profile: list[MechanicalSegmentResultModel] | list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_start_m": segment.measured_depth_start_m,
            "measured_depth_end_m": segment.measured_depth_end_m,
            "measured_depth_center_m": segment.measured_depth_center_m,
            "segment_length_m": segment.segment_length_m,
            "section_name": segment.section_name,
            "inclination_rad": segment.inclination_rad,
            "curvature_rad_per_m": segment.curvature_rad_per_m,
            "curvature_normal_component_rad_per_m": segment.curvature_normal_component_rad_per_m,
            "curvature_binormal_component_rad_per_m": segment.curvature_binormal_component_rad_per_m,
            "frame_rotation_change_rad": segment.frame_rotation_change_rad,
            "tangent_north_east_tvd": list(segment.tangent_north_east_tvd),
            "normal_north_east_tvd": list(segment.normal_north_east_tvd),
            "binormal_north_east_tvd": list(segment.binormal_north_east_tvd),
            "effective_line_weight_n_per_m": segment.effective_line_weight_n_per_m,
            "effective_axial_load_n": segment.effective_axial_load_n,
            "bending_stiffness_n_m2": segment.bending_stiffness_n_m2,
            "bending_moment_n_m": segment.bending_moment_n_m,
            "bending_stress_pa": segment.bending_stress_pa,
            "bending_strain_estimate": segment.bending_strain_estimate,
            "bending_severity_estimate": segment.bending_severity_estimate,
            "gravity_lateral_load_n_per_m": segment.gravity_lateral_load_n_per_m,
            "curvature_lateral_load_n_per_m": segment.curvature_lateral_load_n_per_m,
            "equivalent_lateral_load_n_per_m": segment.equivalent_lateral_load_n_per_m,
            "equivalent_lateral_force_n": segment.equivalent_lateral_force_n,
            "gravity_lateral_load_normal_n_per_m": segment.gravity_lateral_load_normal_n_per_m,
            "gravity_lateral_load_binormal_n_per_m": segment.gravity_lateral_load_binormal_n_per_m,
            "curvature_lateral_load_normal_n_per_m": segment.curvature_lateral_load_normal_n_per_m,
            "curvature_lateral_load_binormal_n_per_m": segment.curvature_lateral_load_binormal_n_per_m,
            "equivalent_lateral_load_normal_n_per_m": segment.equivalent_lateral_load_normal_n_per_m,
            "equivalent_lateral_load_binormal_n_per_m": segment.equivalent_lateral_load_binormal_n_per_m,
            "bending_lateral_stiffness_n_per_m": segment.bending_lateral_stiffness_n_per_m,
            "axial_tension_lateral_stiffness_n_per_m": segment.axial_tension_lateral_stiffness_n_per_m,
            "structural_lateral_stiffness_n_per_m": segment.structural_lateral_stiffness_n_per_m,
            "centralizer_centering_stiffness_n_per_m": segment.centralizer_centering_stiffness_n_per_m,
            "support_contact_penalty_n_per_m": segment.support_contact_penalty_n_per_m,
            "body_contact_penalty_n_per_m": segment.body_contact_penalty_n_per_m,
            "support_outer_diameter_m": segment.support_outer_diameter_m,
            "pipe_body_clearance_m": segment.pipe_body_clearance_m,
            "support_contact_clearance_m": segment.support_contact_clearance_m,
            "bow_force_details": [
                {
                    "source_name": bow_force.source_name,
                    "placement_measured_depth_m": bow_force.placement_measured_depth_m,
                    "bow_index": bow_force.bow_index,
                    "angle_rad": bow_force.angle_rad,
                    "direction_n_b": list(bow_force.direction_n_b),
                    "deflection_m": bow_force.deflection_m,
                    "force_magnitude_n": bow_force.force_magnitude_n,
                    "force_vector_n_b": list(bow_force.force_vector_n_b),
                }
                for bow_force in segment.bow_force_details
            ],
            "free_nodal_output_6dof": _nodal_output_6dof_to_dict(segment.free_nodal_output_6dof),
            "solved_nodal_output_6dof": _nodal_output_6dof_to_dict(segment.solved_nodal_output_6dof),
            "free_eccentricity_estimate_m": segment.free_eccentricity_estimate_m,
            "free_lateral_displacement_normal_m": segment.free_lateral_displacement_normal_m,
            "free_lateral_displacement_binormal_m": segment.free_lateral_displacement_binormal_m,
            "lateral_displacement_normal_m": segment.lateral_displacement_normal_m,
            "lateral_displacement_binormal_m": segment.lateral_displacement_binormal_m,
            "eccentricity_normal_m": segment.eccentricity_normal_m,
            "eccentricity_binormal_m": segment.eccentricity_binormal_m,
            "eccentricity_estimate_m": segment.eccentricity_estimate_m,
            "eccentricity_ratio": segment.eccentricity_ratio,
            "standoff_estimate": segment.standoff_estimate,
            "contact_direction_normal": segment.contact_direction_normal,
            "contact_direction_binormal": segment.contact_direction_binormal,
            "support_normal_reaction_normal_n": segment.support_normal_reaction_normal_n,
            "support_normal_reaction_binormal_n": segment.support_normal_reaction_binormal_n,
            "body_normal_reaction_normal_n": segment.body_normal_reaction_normal_n,
            "body_normal_reaction_binormal_n": segment.body_normal_reaction_binormal_n,
            "normal_reaction_normal_n": segment.normal_reaction_normal_n,
            "normal_reaction_binormal_n": segment.normal_reaction_binormal_n,
            "support_normal_reaction_estimate_n": segment.support_normal_reaction_estimate_n,
            "body_normal_reaction_estimate_n": segment.body_normal_reaction_estimate_n,
            "normal_reaction_estimate_n": segment.normal_reaction_estimate_n,
            "normal_reaction_estimate_n_per_m": segment.normal_reaction_estimate_n_per_m,
            "bow_resultant_normal_n": segment.bow_resultant_normal_n,
            "bow_resultant_binormal_n": segment.bow_resultant_binormal_n,
            "bow_resultant_magnitude_n": segment.bow_resultant_magnitude_n,
            "centralizer_effective_radial_direction_normal": (
                segment.centralizer_effective_radial_direction_normal
            ),
            "centralizer_effective_radial_direction_binormal": (
                segment.centralizer_effective_radial_direction_binormal
            ),
            "centralizer_tangential_direction_normal": segment.centralizer_tangential_direction_normal,
            "centralizer_tangential_direction_binormal": segment.centralizer_tangential_direction_binormal,
            "centralizer_tangential_friction_normal_n": segment.centralizer_tangential_friction_normal_n,
            "centralizer_tangential_friction_binormal_n": segment.centralizer_tangential_friction_binormal_n,
            "centralizer_tangential_friction_vector_magnitude_n": (
                segment.centralizer_tangential_friction_vector_magnitude_n
            ),
            "centralizer_projected_contact_normal_n": (
                segment.centralizer_projected_contact_normal_n
            ),
            "centralizer_friction_interaction_scale": (
                segment.centralizer_friction_interaction_scale
            ),
            "centralizer_axial_friction_n": segment.centralizer_axial_friction_n,
            "centralizer_tangential_friction_n": segment.centralizer_tangential_friction_n,
            "centralizer_torque_increment_n_m": segment.centralizer_torque_increment_n_m,
            "centralizer_effective_contact_radius_m": segment.centralizer_effective_contact_radius_m,
            "centralizer_torque_details": [
                {
                    "source_name": detail.source_name,
                    "placement_measured_depth_m": detail.placement_measured_depth_m,
                    "effective_contact_radius_m": detail.effective_contact_radius_m,
                    "axial_force_ratio": detail.axial_force_ratio,
                    "tangential_force_ratio": detail.tangential_force_ratio,
                    "reduced_torsional_load_n_m": detail.reduced_torsional_load_n_m,
                    "reduced_twist_rate_rad_per_m": detail.reduced_twist_rate_rad_per_m,
                    "torsional_slip_indicator": detail.torsional_slip_indicator,
                    "torsional_tangential_demand_factor": (
                        detail.torsional_tangential_demand_factor
                    ),
                    "bow_resultant_vector_n_b": list(detail.bow_resultant_vector_n_b),
                    "bow_resultant_magnitude_n": detail.bow_resultant_magnitude_n,
                    "radial_direction_n_b": list(detail.radial_direction_n_b),
                    "local_contact_direction_n_b": list(detail.local_contact_direction_n_b),
                    "effective_radial_direction_n_b": list(detail.effective_radial_direction_n_b),
                    "tangential_direction_n_b": list(detail.tangential_direction_n_b),
                    "tangential_friction_vector_n_b": list(detail.tangential_friction_vector_n_b),
                    "tangential_friction_vector_magnitude_n": detail.tangential_friction_vector_magnitude_n,
                    "local_contact_weight": detail.local_contact_weight,
                    "direction_alignment_cosine": detail.direction_alignment_cosine,
                    "projected_contact_normal_n": detail.projected_contact_normal_n,
                    "friction_interaction_scale": detail.friction_interaction_scale,
                    "axial_friction_n": detail.axial_friction_n,
                    "tangential_friction_n": detail.tangential_friction_n,
                    "torque_increment_n_m": detail.torque_increment_n_m,
                    "active": detail.active,
                    "status": detail.status,
                }
                for detail in segment.centralizer_torque_details
            ],
            "centralizer_torque_status": segment.centralizer_torque_status,
            "nearby_centralizer_count": segment.nearby_centralizer_count,
            "contact_iteration_count": segment.contact_iteration_count,
            "contact_state": segment.contact_state,
            "support_in_contact": segment.support_in_contact,
            "pipe_body_in_contact": segment.pipe_body_in_contact,
        }
        for segment in profile
    ]


def _nodal_output_6dof_to_dict(nodal_output: Nodal6DOFOutputModel | Any) -> dict[str, float]:
    if not isinstance(nodal_output, Nodal6DOFOutputModel):
        raise ValueError(
            "Mechanical profile export requires Nodal6DOFOutputModel with [u, v, w, theta_x, theta_y, theta_z]."
        )
    return {
        "u_m": nodal_output.u_m,
        "v_m": nodal_output.v_m,
        "w_m": nodal_output.w_m,
        "theta_x_rad": nodal_output.theta_x_rad,
        "theta_y_rad": nodal_output.theta_y_rad,
        "theta_z_rad": nodal_output.theta_z_rad,
    }


def _axial_force_profile_to_dict(profile: list[AxialForcePointModel] | list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "axial_force_n": point.axial_force_n,
        }
        for point in profile
    ]


def _torque_profile_to_dict(profile: list[TorquePointModel] | list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "effective_contact_radius_m": point.effective_contact_radius_m,
            "body_torque_increment_n_m": point.body_torque_increment_n_m,
            "centralizer_torque_increment_n_m": point.centralizer_torque_increment_n_m,
            "local_torque_increment_n_m": point.local_torque_increment_n_m,
            "cumulative_torque_n_m": point.cumulative_torque_n_m,
        }
        for point in profile
    ]


def _reduced_torque_accumulation_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "body_torque_increment_n_m": point.body_torque_increment_n_m,
            "centralizer_torque_increment_n_m": point.centralizer_torque_increment_n_m,
            "local_torque_increment_n_m": point.local_torque_increment_n_m,
            "raw_cumulative_torque_n_m": point.raw_cumulative_torque_n_m,
            "carried_torsional_load_n_m": point.carried_torsional_load_n_m,
            "status": point.status,
        }
        for point in profile
    ]


def _torsional_state_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "segment_length_m": point.segment_length_m,
            "section_name": point.section_name,
            "body_torque_increment_n_m": point.body_torque_increment_n_m,
            "centralizer_torque_increment_n_m": point.centralizer_torque_increment_n_m,
            "local_torque_increment_n_m": point.local_torque_increment_n_m,
            "reduced_torsional_load_n_m": point.reduced_torsional_load_n_m,
            "section_torsional_stiffness_n_m2": point.section_torsional_stiffness_n_m2,
            "reduced_twist_rate_rad_per_m": point.reduced_twist_rate_rad_per_m,
            "reduced_twist_increment_rad": point.reduced_twist_increment_rad,
            "cumulative_reduced_twist_rad": point.cumulative_reduced_twist_rad,
            "status": point.status,
        }
        for point in profile
    ]


def _centralizer_friction_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "centralizer_axial_friction_n": point.centralizer_axial_friction_n,
            "centralizer_tangential_friction_n": point.centralizer_tangential_friction_n,
        }
        for point in profile
    ]


def _body_friction_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "body_axial_friction_n": point.body_axial_friction_n,
            "body_tangential_friction_n": point.body_tangential_friction_n,
        }
        for point in profile
    ]


def _centralizer_tangential_vector_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "effective_radial_direction_normal": point.effective_radial_direction_normal,
            "effective_radial_direction_binormal": point.effective_radial_direction_binormal,
            "tangential_direction_normal": point.tangential_direction_normal,
            "tangential_direction_binormal": point.tangential_direction_binormal,
            "tangential_friction_normal_n": point.tangential_friction_normal_n,
            "tangential_friction_binormal_n": point.tangential_friction_binormal_n,
            "projected_contact_normal_n": point.projected_contact_normal_n,
            "friction_interaction_scale": point.friction_interaction_scale,
            "tangential_friction_magnitude_n": point.tangential_friction_magnitude_n,
            "status": point.status,
        }
        for point in profile
    ]


def _centralizer_tangential_direction_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "reduced_torsional_load_n_m": point.reduced_torsional_load_n_m,
            "reduced_twist_rate_rad_per_m": point.reduced_twist_rate_rad_per_m,
            "torsional_slip_indicator": point.torsional_slip_indicator,
            "torsional_tangential_demand_factor": point.torsional_tangential_demand_factor,
            "effective_radial_direction_normal": point.effective_radial_direction_normal,
            "effective_radial_direction_binormal": point.effective_radial_direction_binormal,
            "tangential_direction_normal": point.tangential_direction_normal,
            "tangential_direction_binormal": point.tangential_direction_binormal,
            "projected_contact_normal_n": point.projected_contact_normal_n,
            "friction_interaction_scale": point.friction_interaction_scale,
            "status": point.status,
        }
        for point in profile
    ]


def _local_tangential_interaction_state_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "reduced_torsional_load_n_m": point.reduced_torsional_load_n_m,
            "reduced_twist_rate_rad_per_m": point.reduced_twist_rate_rad_per_m,
            "body_effective_contact_radius_m": point.body_effective_contact_radius_m,
            "body_torsional_slip_indicator": point.body_torsional_slip_indicator,
            "body_tangential_mobilization": point.body_tangential_mobilization,
            "body_tangential_demand_factor": point.body_tangential_demand_factor,
            "body_tangential_traction_indicator": point.body_tangential_traction_indicator,
            "body_tangential_regime": point.body_tangential_regime,
            "body_feedback_applied": point.body_feedback_applied,
            "centralizer_effective_contact_radius_m": point.centralizer_effective_contact_radius_m,
            "centralizer_torsional_slip_indicator": point.centralizer_torsional_slip_indicator,
            "centralizer_tangential_mobilization": point.centralizer_tangential_mobilization,
            "centralizer_tangential_demand_factor": point.centralizer_tangential_demand_factor,
            "centralizer_tangential_traction_indicator": point.centralizer_tangential_traction_indicator,
            "centralizer_tangential_regime": point.centralizer_tangential_regime,
            "centralizer_feedback_applied": point.centralizer_feedback_applied,
            "local_tangential_mobilization": point.local_tangential_mobilization,
            "local_tangential_traction_indicator": point.local_tangential_traction_indicator,
            "local_tangential_regime": point.local_tangential_regime,
            "body_status": point.body_status,
            "centralizer_status": point.centralizer_status,
            "status": point.status,
        }
        for point in profile
    ]


def _local_body_tangential_interaction_state_from_payload(
    payload: dict[str, Any],
) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": interaction["measured_depth_m"],
            "reduced_torsional_load_n_m": interaction["reduced_torsional_load_n_m"],
            "reduced_twist_rate_rad_per_m": interaction["reduced_twist_rate_rad_per_m"],
            "effective_contact_radius_m": interaction["body_effective_contact_radius_m"],
            "torsional_slip_indicator": interaction["body_torsional_slip_indicator"],
            "tangential_mobilization": interaction["body_tangential_mobilization"],
            "tangential_demand_factor": interaction["body_tangential_demand_factor"],
            "tangential_traction_indicator": interaction["body_tangential_traction_indicator"],
            "tangential_regime": interaction["body_tangential_regime"],
            "axial_friction_n": body_friction["body_axial_friction_n"],
            "updated_tangential_friction_n": body_friction["body_tangential_friction_n"],
            "local_torque_increment_n_m": body_torque["local_torque_increment_n_m"],
            "feedback_applied": interaction["body_feedback_applied"],
            "status": interaction["body_status"],
        }
        for interaction, body_friction, body_torque in zip(
            payload.get("local_tangential_interaction_state", []),
            payload.get("body_axial_friction_profile", []),
            payload.get("updated_body_torque_profile", []),
            strict=True,
        )
    ]


def _local_centralizer_tangential_interaction_state_from_payload(
    payload: dict[str, Any],
) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": interaction["measured_depth_m"],
            "reduced_torsional_load_n_m": interaction["reduced_torsional_load_n_m"],
            "reduced_twist_rate_rad_per_m": interaction["reduced_twist_rate_rad_per_m"],
            "effective_contact_radius_m": interaction["centralizer_effective_contact_radius_m"],
            "torsional_slip_indicator": interaction["centralizer_torsional_slip_indicator"],
            "tangential_mobilization": interaction["centralizer_tangential_mobilization"],
            "tangential_demand_factor": interaction["centralizer_tangential_demand_factor"],
            "tangential_traction_indicator": interaction[
                "centralizer_tangential_traction_indicator"
            ],
            "tangential_regime": interaction["centralizer_tangential_regime"],
            "projected_contact_normal_n": centralizer_direction["projected_contact_normal_n"],
            "friction_interaction_scale": centralizer_direction["friction_interaction_scale"],
            "axial_friction_n": centralizer_friction["centralizer_axial_friction_n"],
            "updated_tangential_friction_n": centralizer_friction[
                "centralizer_tangential_friction_n"
            ],
            "local_torque_increment_n_m": centralizer_torque["local_torque_increment_n_m"],
            "feedback_applied": interaction["centralizer_feedback_applied"],
            "status": interaction["centralizer_status"],
        }
        for interaction, centralizer_direction, centralizer_friction, centralizer_torque in zip(
            payload.get("local_tangential_interaction_state", []),
            payload.get("centralizer_tangential_direction_profile", []),
            payload.get("centralizer_tangential_friction_profile", []),
            payload.get("updated_centralizer_torque_profile", []),
            strict=True,
        )
    ]


def _local_tangential_state_from_payload(payload: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point["measured_depth_m"],
            "reduced_torsional_load_n_m": point["reduced_torsional_load_n_m"],
            "reduced_twist_rate_rad_per_m": point["reduced_twist_rate_rad_per_m"],
            "local_tangential_mobilization": point["local_tangential_mobilization"],
            "local_tangential_traction_indicator": point[
                "local_tangential_traction_indicator"
            ],
            "local_tangential_regime": point["local_tangential_regime"],
            "body_tangential_regime": point["body_tangential_regime"],
            "centralizer_tangential_regime": point["centralizer_tangential_regime"],
            "status": point["status"],
        }
        for point in payload.get("local_tangential_interaction_state", [])
    ]


def _local_tangential_mobilization_profile_from_payload(
    payload: dict[str, Any],
) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point["measured_depth_m"],
            "body_tangential_mobilization": point["body_tangential_mobilization"],
            "centralizer_tangential_mobilization": point[
                "centralizer_tangential_mobilization"
            ],
            "local_tangential_mobilization": point["local_tangential_mobilization"],
            "body_tangential_traction_indicator": point[
                "body_tangential_traction_indicator"
            ],
            "centralizer_tangential_traction_indicator": point[
                "centralizer_tangential_traction_indicator"
            ],
            "local_tangential_traction_indicator": point[
                "local_tangential_traction_indicator"
            ],
            "local_tangential_regime": point["local_tangential_regime"],
            "status": point["status"],
        }
        for point in payload.get("local_tangential_interaction_state", [])
    ]


def _centralizer_torque_breakdown_profile_to_dict(
    profile: list[list[Any]],
) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": (
                0.0 if not segment_points else segment_points[0].measured_depth_m
            ),
            "placements": [
                {
                    "source_name": point.source_name,
                    "placement_measured_depth_m": point.placement_measured_depth_m,
                    "effective_contact_radius_m": point.effective_contact_radius_m,
                    "axial_force_ratio": point.axial_force_ratio,
                    "tangential_force_ratio": point.tangential_force_ratio,
                    "reduced_torsional_load_n_m": point.reduced_torsional_load_n_m,
                    "reduced_twist_rate_rad_per_m": point.reduced_twist_rate_rad_per_m,
                    "torsional_slip_indicator": point.torsional_slip_indicator,
                    "torsional_tangential_demand_factor": point.torsional_tangential_demand_factor,
                    "bow_resultant_normal_n": point.bow_resultant_normal_n,
                    "bow_resultant_binormal_n": point.bow_resultant_binormal_n,
                    "bow_resultant_magnitude_n": point.bow_resultant_magnitude_n,
                    "local_contact_direction_normal": point.local_contact_direction_normal,
                    "local_contact_direction_binormal": point.local_contact_direction_binormal,
                    "effective_radial_direction_normal": point.effective_radial_direction_normal,
                    "effective_radial_direction_binormal": point.effective_radial_direction_binormal,
                    "tangential_direction_normal": point.tangential_direction_normal,
                    "tangential_direction_binormal": point.tangential_direction_binormal,
                    "tangential_friction_normal_n": point.tangential_friction_normal_n,
                    "tangential_friction_binormal_n": point.tangential_friction_binormal_n,
                    "tangential_friction_magnitude_n": point.tangential_friction_magnitude_n,
                    "local_contact_weight": point.local_contact_weight,
                    "direction_alignment_cosine": point.direction_alignment_cosine,
                    "projected_contact_normal_n": point.projected_contact_normal_n,
                    "friction_interaction_scale": point.friction_interaction_scale,
                    "axial_friction_n": point.axial_friction_n,
                    "tangential_friction_n": point.tangential_friction_n,
                    "local_torque_increment_n_m": point.local_torque_increment_n_m,
                    "cumulative_torque_n_m": point.cumulative_torque_n_m,
                    "status": point.status,
                }
                for point in segment_points
            ],
        }
        for segment_points in profile
    ]


def _torque_partition_summary_to_dict(summary: Any) -> dict[str, Any]:
    return {
        "body_surface_torque_n_m": summary.body_surface_torque_n_m,
        "centralizer_surface_torque_n_m": summary.centralizer_surface_torque_n_m,
        "total_surface_torque_n_m": summary.total_surface_torque_n_m,
        "body_axial_friction_sum_n": summary.body_axial_friction_sum_n,
        "centralizer_axial_friction_sum_n": summary.centralizer_axial_friction_sum_n,
        "body_tangential_friction_sum_n": summary.body_tangential_friction_sum_n,
        "centralizer_tangential_friction_sum_n": summary.centralizer_tangential_friction_sum_n,
        "status": summary.status,
    }


def _updated_surface_torque_from_result(result: dict[str, Any]) -> float | None:
    torsional_state_profile = result.get("torsional_state_profile", [])
    if torsional_state_profile:
        return float(torsional_state_profile[0]["reduced_torsional_load_n_m"])
    torque_partition_summary = result.get("torque_partition_summary")
    if isinstance(torque_partition_summary, dict):
        return float(torque_partition_summary.get("total_surface_torque_n_m", 0.0))
    estimated_surface_torque_n_m = result.get("estimated_surface_torque_n_m")
    if estimated_surface_torque_n_m is None:
        return None
    return float(estimated_surface_torque_n_m)


def _normal_reaction_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "support_normal_reaction_estimate_n": point.support_normal_reaction_estimate_n,
            "body_normal_reaction_estimate_n": point.body_normal_reaction_estimate_n,
            "normal_reaction_estimate_n": point.normal_reaction_estimate_n,
        }
        for point in profile
    ]


def _centralizer_traceability_to_dict(loaded_case: LoadedCase) -> list[dict[str, Any]]:
    hole_radius_m = 0.5 * loaded_case.reference_hole_diameter_m
    traceability: list[dict[str, Any]] = []
    for spec in loaded_case.centralizers.centralizers:
        resolved_k = resolved_blade_power_law_k(spec, hole_radius_m)
        reference_deflection_m = bow_reference_deflection_m(spec, hole_radius_m)
        reference_loaded_bow_count = max(1.0, math.ceil(0.5 * float(spec.number_of_bows)))
        traceability.append(
            {
                "name": spec.name,
                "type": spec.type,
                "number_of_bows": spec.number_of_bows,
                "angular_orientation_reference_deg": spec.angular_orientation_reference_deg,
                "inner_clearance_to_pipe_m": spec.inner_clearance_to_pipe_m,
                "nominal_restoring_force_n": spec.nominal_restoring_force_n,
                "nominal_running_force_n": spec.nominal_running_force_n,
                "running_to_restoring_force_ratio": centralizer_running_force_ratio(spec),
                "axial_friction_force_ratio": centralizer_axial_force_ratio(spec),
                "tangential_torque_force_ratio": centralizer_tangential_force_ratio(spec),
                "axial_friction_parameter_status": (
                    "explicit_axial_force_ratio"
                    if spec.axial_force_ratio is not None
                    else "derived_from_nominal_running_to_restoring_ratio"
                ),
                "tangential_torque_parameter_status": (
                    "explicit_tangential_force_ratio"
                    if spec.tangential_force_ratio is not None
                    else "derived_from_nominal_running_to_restoring_ratio"
                ),
                "support_outer_diameter_m": spec.support_outer_diameter_m,
                "effective_contact_diameter_m": centralizer_effective_contact_diameter_m(spec),
                "reference_deflection_m": reference_deflection_m,
                "reference_loaded_bow_count": reference_loaded_bow_count,
                "blade_power_law_k_input_n_per_m_pow_p": spec.blade_power_law_k,
                "blade_power_law_p_input": spec.blade_power_law_p,
                "resolved_blade_power_law_k_n_per_m_pow_p": resolved_k,
                "resolved_blade_power_law_p": spec.blade_power_law_p,
                "equivalent_reference_support_stiffness_n_per_m": (
                    equivalent_bow_support_stiffness_n_per_m(spec, hole_radius_m)
                ),
                "stiffness_parameter_role": "bow-restoring-force-law",
                "torque_parameter_role": "reduced-centralizer-tangential-force-ratio",
                "force_per_bow_at_5mm_n": bow_force_magnitude_n(spec, hole_radius_m, 0.005),
                "force_per_bow_at_10mm_n": bow_force_magnitude_n(spec, hole_radius_m, 0.010),
                "calibration_status": (
                    "explicit_power_law_input"
                    if spec.blade_power_law_k is not None
                    else "nominal_restoring_force_fallback"
                ),
            }
        )
    return traceability


def _traceability_to_dict(loaded_case: LoadedCase, result: dict[str, Any]) -> dict[str, Any]:
    return {
        "convergence": {
            "global_solver": {
                "max_iterations": loaded_case.global_solver_max_iterations,
                "iteration_count": result["mechanical_summary"]["global_solver_iteration_count"],
                "final_update_norm_m": result["mechanical_summary"]["global_solver_final_update_norm_m"],
                "update_tolerance_m": GLOBAL_SOLVER_UPDATE_TOLERANCE_M,
            },
            "coupling": {
                "max_iterations": loaded_case.coupling_max_iterations,
                "iteration_count": result["coupling_iterations"],
                "tolerance_n": loaded_case.coupling_tolerance_n,
                "torque_tolerance_n_m": loaded_case.coupling_torque_tolerance_n_m,
                "relaxation_factor": loaded_case.relaxation_factor,
                "final_max_profile_update_n": result["coupling_final_max_profile_update_n"],
                "final_max_torque_update_n_m": result["coupling_final_max_torque_update_n_m"],
                "final_max_torsional_load_update_n_m": result[
                    "coupling_final_max_torsional_load_update_n_m"
                ],
                "converged": result["coupling_converged"],
                "status": result["coupling_status"],
                "torque_feedback_mode": result["torque_feedback_mode"],
                "torsional_feedback_status": result["torsional_feedback_status"],
            },
        },
        "centralizer_parameters": _centralizer_traceability_to_dict(loaded_case),
    }


def _convergence_metadata_to_dict(loaded_case: LoadedCase, result: dict[str, Any]) -> dict[str, Any]:
    return {
        "global_solver": {
            "reference_frame": "local-trajectory-frame",
            "transformed_coordinates": "north-east-tvd",
            "max_iterations": loaded_case.global_solver_max_iterations,
            "iteration_count": result["mechanical_summary"]["global_solver_iteration_count"],
            "final_update_norm_m": result["mechanical_summary"]["global_solver_final_update_norm_m"],
            "update_tolerance_m": GLOBAL_SOLVER_UPDATE_TOLERANCE_M,
        },
        "coupling": {
            "status": result["coupling_status"],
            "converged": result["coupling_converged"],
            "max_iterations": loaded_case.coupling_max_iterations,
            "iteration_count": result["coupling_iterations"],
            "tolerance_n": loaded_case.coupling_tolerance_n,
            "torque_tolerance_n_m": loaded_case.coupling_torque_tolerance_n_m,
            "relaxation_factor": loaded_case.relaxation_factor,
            "final_max_profile_update_n": result["coupling_final_max_profile_update_n"],
            "final_max_torque_update_n_m": result["coupling_final_max_torque_update_n_m"],
            "final_max_torsional_load_update_n_m": result[
                "coupling_final_max_torsional_load_update_n_m"
            ],
            "torque_feedback_mode": result["torque_feedback_mode"],
            "torsional_feedback_status": result["torsional_feedback_status"],
        },
    }


def _derived_profiles(mechanical_profile: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    return {
        "lateral_displacement_n_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "lateral_displacement_normal_m": segment["lateral_displacement_normal_m"],
            }
            for segment in mechanical_profile
        ],
        "lateral_displacement_b_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "lateral_displacement_binormal_m": segment["lateral_displacement_binormal_m"],
            }
            for segment in mechanical_profile
        ],
        "eccentricity_vector_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "eccentricity_normal_m": segment["eccentricity_normal_m"],
                "eccentricity_binormal_m": segment["eccentricity_binormal_m"],
            }
            for segment in mechanical_profile
        ],
        "eccentricity_magnitude_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "eccentricity_estimate_m": segment["eccentricity_estimate_m"],
            }
            for segment in mechanical_profile
        ],
        "contact_direction_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "contact_direction_normal": segment["contact_direction_normal"],
                "contact_direction_binormal": segment["contact_direction_binormal"],
            }
            for segment in mechanical_profile
        ],
        "global_eccentricity_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "eccentricity_estimate_m": segment["eccentricity_estimate_m"],
            }
            for segment in mechanical_profile
        ],
        "standoff_estimate_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "standoff_estimate": segment["standoff_estimate"],
            }
            for segment in mechanical_profile
        ],
        "contact_state_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "contact_state": segment["contact_state"],
            }
            for segment in mechanical_profile
        ],
        "normal_reaction_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "support_normal_reaction_estimate_n": segment["support_normal_reaction_estimate_n"],
                "body_normal_reaction_estimate_n": segment["body_normal_reaction_estimate_n"],
                "normal_reaction_estimate_n": segment["normal_reaction_estimate_n"],
            }
            for segment in mechanical_profile
        ],
        "normal_reaction_vector_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "normal_reaction_normal_n": segment["normal_reaction_normal_n"],
                "normal_reaction_binormal_n": segment["normal_reaction_binormal_n"],
            }
            for segment in mechanical_profile
        ],
        "normal_reaction_magnitude_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "normal_reaction_estimate_n": segment["normal_reaction_estimate_n"],
            }
            for segment in mechanical_profile
        ],
        "bending_strain_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bending_strain_estimate": segment["bending_strain_estimate"],
            }
            for segment in mechanical_profile
        ],
        "bending_severity_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bending_severity_estimate": segment["bending_severity_estimate"],
            }
            for segment in mechanical_profile
        ],
        "bow_force_vectors": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "vectors_n_b": [
                    list(bow_force["force_vector_n_b"])
                    for bow_force in segment["bow_force_details"]
                ],
            }
            for segment in mechanical_profile
        ],
        "bow_force_magnitudes": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "magnitudes_n": [
                    bow_force["force_magnitude_n"]
                    for bow_force in segment["bow_force_details"]
                ],
            }
            for segment in mechanical_profile
        ],
        "bow_resultant_vector_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bow_resultant_normal_n": segment["bow_resultant_normal_n"],
                "bow_resultant_binormal_n": segment["bow_resultant_binormal_n"],
            }
            for segment in mechanical_profile
        ],
        "bow_resultant_magnitude_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bow_resultant_magnitude_n": segment["bow_resultant_magnitude_n"],
            }
            for segment in mechanical_profile
        ],
        "bending_moment_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bending_moment_n_m": segment["bending_moment_n_m"],
            }
            for segment in mechanical_profile
        ],
        "axial_load_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "effective_axial_load_n": segment["effective_axial_load_n"],
            }
            for segment in mechanical_profile
        ],
    }


def format_case_summary(loaded_case: LoadedCase) -> str:
    trajectory_summary = loaded_case.trajectory_summary()
    string_summary = loaded_case.string_summary()
    centralizer_summary = loaded_case.centralizer_summary()

    lines = [
        f"Case: {loaded_case.definition.name}",
        f"Well: {loaded_case.well.name}",
        f"Trajectory points: {trajectory_summary.point_count}",
        f"Final measured depth [m]: {trajectory_summary.final_measured_depth_m:.2f}",
        f"Approx vertical depth [m]: {trajectory_summary.vertical_depth_m:.2f}",
        f"Approx lateral displacement [m]: {trajectory_summary.lateral_displacement_m:.2f}",
        f"Max inclination [rad]: {trajectory_summary.max_inclination_rad:.4f}",
        f"Max discrete curvature [rad/m]: {trajectory_summary.max_curvature_rad_per_m:.6f}",
        f"Reference hole diameter [m]: {loaded_case.reference_hole_diameter_m:.4f}",
        f"Fluid density [kg/m3]: {loaded_case.fluid_density_kg_per_m3:.1f}",
        f"Operation mode: {loaded_case.operation_mode}",
        f"Discretization step [m]: {loaded_case.discretization_step_m:.2f}",
        f"Global solver max iterations [-]: {loaded_case.global_solver_max_iterations}",
        f"Contact penalty scale [-]: {loaded_case.contact_penalty_scale:.2f}",
        f"Coupling max iterations [-]: {loaded_case.coupling_max_iterations}",
        f"Coupling tolerance [N]: {loaded_case.coupling_tolerance_n:.2f}",
        f"Coupling torque tolerance [N.m]: {loaded_case.coupling_torque_tolerance_n_m:.2f}",
        f"Relaxation factor [-]: {loaded_case.relaxation_factor:.2f}",
        f"Frame method: {loaded_case.frame_method}",
        f"String: {loaded_case.string.name}",
        f"String sections: {string_summary.section_count}",
        f"Total string length [m]: {string_summary.total_length_m:.2f}",
        f"Total string dry weight [N]: {string_summary.total_weight_n:.2f}",
        f"Total string effective weight [N]: {string_summary.total_effective_weight_n:.2f}",
        f"Max string OD [m]: {string_summary.max_outer_diameter_m:.4f}",
        f"Average friction coefficient [-]: {string_summary.average_friction_coefficient:.3f}",
        f"Centralizer specs: {centralizer_summary.spec_count}",
        f"Explicit installation count: {centralizer_summary.explicit_installation_count}",
        f"Expanded installation count: {centralizer_summary.expanded_installation_count}",
        f"Min nominal radial clearance [m]: {loaded_case.minimum_nominal_radial_clearance_m():.4f}",
    ]
    return "\n".join(lines)


def _mechanical_result(loaded_case: LoadedCase) -> dict[str, Any]:
    trajectory_summary = loaded_case.trajectory_summary()
    string_summary = loaded_case.string_summary()
    section_summaries = loaded_case.section_summaries()
    coupling_result = run_coupled_global_baseline(loaded_case)
    centralizer_summary = loaded_case.centralizer_summary()
    centralizer_summary.expanded_installation_count = len(coupling_result.placements)
    warnings = list(PHASE14_WARNINGS)
    if loaded_case.reference_hole_diameter_m <= 0.0:
        warnings.append(
            "Reference hole diameter is absent, so annular contact and standoff are reported as undefined-safe defaults."
        )

    return {
        "backend": "python-numpy",
        "status": "phase14-vector-local-tangential-torque-coupled-baseline",
        "message": (
            "Phase 14 reduced vector-frame torque and drag baseline with detailed bow-spring "
            "centralizer modeling. The active runtime path now exports a 6-DOF nodal state "
            "[u, v, w, theta_x, theta_y, theta_z] through the isolated Newton-Raphson wiring, "
            "while support reactions and centralizer torque still distinguish pipe-body and "
            "bow-spring contributions, use a contact-informed reduced tangential law tied to the "
            "local contact direction plus bow resultant, propagate a reduced torsional-load/twist "
            "state with GJ-based twist indicators, and let the carried reduced torsional state "
            "feed a shared reduced local tangential-state model with slip indicator, "
            "mobilization, traction indicator, and regime fields before applying the body and "
            "centralizer tangential laws. The updated local centralizer demand still affects the "
            "axial coupling loop, while both local tangential laws affect the carried torsional "
            "state without claiming a full torsional structural solve. This remains a hybrid "
            "engineering baseline, not a fully validated commercial stiff-string/contact solver."
        ),
        "operation_mode": loaded_case.operation_mode,
        "geometry_is_approximate": True,
        "trajectory_summary": _trajectory_summary_to_dict(trajectory_summary),
        "string_summary": _string_summary_to_dict(string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(centralizer_summary),
        "mechanical_summary": _mechanical_summary_to_dict(coupling_result.mechanical_summary),
        "section_summaries": _section_summaries_to_dict(section_summaries),
        "mechanical_profile": _mechanical_profile_to_dict(coupling_result.mechanical_profile),
        "estimated_hookload_n": (
            coupling_result.torque_drag_result.hookload_pull_out_n
            if loaded_case.operation_mode == "pull_out"
            else coupling_result.mechanical_summary.top_effective_axial_load_n
            if loaded_case.operation_mode == "rotate_in_place"
            else coupling_result.torque_drag_result.hookload_run_in_n
        ),
        "hookload_run_in_n": coupling_result.torque_drag_result.hookload_run_in_n,
        "hookload_pull_out_n": coupling_result.torque_drag_result.hookload_pull_out_n,
        "drag_run_in_n": coupling_result.torque_drag_result.drag_run_in_n,
        "drag_pull_out_n": coupling_result.torque_drag_result.drag_pull_out_n,
        "axial_force_run_in_profile": _axial_force_profile_to_dict(
            coupling_result.torque_drag_result.axial_force_run_in_profile
        ),
        "axial_force_pull_out_profile": _axial_force_profile_to_dict(
            coupling_result.torque_drag_result.axial_force_pull_out_profile
        ),
        "torque_profile": _torque_profile_to_dict(coupling_result.torque_drag_result.torque_profile),
        "body_axial_friction_profile": _body_friction_profile_to_dict(
            coupling_result.torque_drag_result.body_axial_friction_profile
        ),
        "body_torque_profile": _torque_profile_to_dict(
            coupling_result.torque_drag_result.body_torque_profile
        ),
        "centralizer_axial_friction_profile": _centralizer_friction_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_axial_friction_profile
        ),
        "centralizer_tangential_friction_profile": _centralizer_friction_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_tangential_friction_profile
        ),
        "centralizer_tangential_direction_profile": _centralizer_tangential_direction_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_tangential_direction_profile
        ),
        "centralizer_tangential_friction_vector_profile": _centralizer_tangential_vector_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_tangential_friction_vector_profile
        ),
        "centralizer_torque_profile": _torque_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_torque_profile
        ),
        "centralizer_torque_breakdown_profile": _centralizer_torque_breakdown_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_torque_breakdown_profile
        ),
        "local_tangential_interaction_state": _local_tangential_interaction_state_to_dict(
            coupling_result.torque_drag_result.local_tangential_interaction_state
        ),
        "updated_body_torque_profile": _torque_profile_to_dict(
            coupling_result.torque_drag_result.body_torque_profile
        ),
        "updated_centralizer_torque_profile": _torque_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_torque_profile
        ),
        "reduced_torque_accumulation_profile": _reduced_torque_accumulation_profile_to_dict(
            coupling_result.reduced_torque_accumulation_profile
        ),
        "torsional_state_profile": _torsional_state_profile_to_dict(
            coupling_result.torsional_state_profile
        ),
        "coupling_status": coupling_result.status,
        "coupling_iterations": coupling_result.iteration_count,
        "coupling_final_max_profile_update_n": coupling_result.maximum_profile_update_n,
        "coupling_final_max_torque_update_n_m": coupling_result.maximum_torque_update_n_m,
        "coupling_final_max_torsional_load_update_n_m": (
            coupling_result.maximum_torsional_load_update_n_m
        ),
        "coupling_converged": coupling_result.converged,
        "torque_feedback_mode": coupling_result.torque_feedback_mode,
        "torsional_feedback_status": coupling_result.torsional_feedback_status,
        "converged_axial_profile": _axial_force_profile_to_dict(
            coupling_result.converged_axial_profile
        ),
        "converged_normal_reaction_profile": _normal_reaction_profile_to_dict(
            coupling_result.converged_normal_reaction_profile
        ),
        "converged_torque_profile": _torque_profile_to_dict(
            coupling_result.converged_torque_profile
        ),
        "estimated_surface_torque_n_m": coupling_result.torque_drag_result.estimated_surface_torque_n_m,
        "minimum_standoff_estimate": coupling_result.mechanical_summary.minimum_standoff_estimate,
        "minimum_nominal_radial_clearance_m": loaded_case.minimum_nominal_radial_clearance_m(),
        "contact_nodes": coupling_result.mechanical_summary.contact_segment_count,
        "torque_partition_summary": _torque_partition_summary_to_dict(
            coupling_result.torque_drag_result.torque_partition_summary
        ),
        "centralizer_model_status": "phase14-detailed-bow-spring-local-tangential-vector-torque",
        "torque_and_drag_real_implemented": False,
        "torque_and_drag_status": coupling_result.torque_drag_result.status,
        "torque_drag_status": coupling_result.torque_drag_result.status,
        "warnings": warnings,
        "todos": list(PHASE11_TODOS),
    }


def _resolve_output_path(loaded_case: LoadedCase, output: str | Path | None) -> Path:
    if output is not None:
        return Path(output).resolve()
    if loaded_case.definition.output_json:
        return (loaded_case.case_path.parent / loaded_case.definition.output_json).resolve()
    return loaded_case.case_path.with_suffix(".stub.json")


def run_stub_case(
    case_path: str | Path,
    output: str | Path | None = None,
) -> tuple[LoadedCase, dict[str, Any], Path]:
    loaded_case = load_case_bundle(case_path)
    result = _mechanical_result(loaded_case)
    derived_profiles = _derived_profiles(result["mechanical_profile"])

    payload = {
        "case": loaded_case.definition.name,
        "well": loaded_case.well.name,
        "string": loaded_case.string.name,
        "centralizer_config": loaded_case.centralizers.name,
        "validation_status": PHASE10_VALIDATION_STATUS,
        "inputs": {
            "case_path": str(loaded_case.case_path),
            "well_path": str(loaded_case.well_path),
            "string_path": str(loaded_case.string_path),
            "centralizers_path": str(loaded_case.centralizers_path),
            "reference_hole_diameter_m": loaded_case.reference_hole_diameter_m,
            "fluid_density_kg_per_m3": loaded_case.fluid_density_kg_per_m3,
            "operation_mode": loaded_case.operation_mode,
            "discretization_step_m": loaded_case.discretization_step_m,
            "global_solver_max_iterations": loaded_case.global_solver_max_iterations,
            "contact_penalty_scale": loaded_case.contact_penalty_scale,
            "coupling_max_iterations": loaded_case.coupling_max_iterations,
            "coupling_tolerance_n": loaded_case.coupling_tolerance_n,
            "coupling_torque_tolerance_n_m": loaded_case.coupling_torque_tolerance_n_m,
            "relaxation_factor": loaded_case.relaxation_factor,
            "frame_method": loaded_case.frame_method,
        },
        **result,
        **derived_profiles,
        "convergence_metadata": _convergence_metadata_to_dict(loaded_case, result),
        "traceability": _traceability_to_dict(loaded_case, result),
        "updated_estimated_surface_torque_n_m": _updated_surface_torque_from_result(result),
    }
    payload["tangential_friction_vector_profile"] = list(
        payload["centralizer_tangential_friction_vector_profile"]
    )
    payload["local_body_tangential_interaction_state"] = (
        _local_body_tangential_interaction_state_from_payload(payload)
    )
    payload["local_centralizer_tangential_interaction_state"] = (
        _local_centralizer_tangential_interaction_state_from_payload(payload)
    )
    payload["local_tangential_state"] = _local_tangential_state_from_payload(payload)
    payload["local_tangential_mobilization_profile"] = (
        _local_tangential_mobilization_profile_from_payload(payload)
    )

    output_path = _resolve_output_path(loaded_case, output)
    write_json(output_path, payload)
    return loaded_case, payload, output_path
