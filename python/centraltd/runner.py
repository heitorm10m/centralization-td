from __future__ import annotations

from pathlib import Path
import math
from typing import Any

from .bow_spring import (
    bow_force_magnitude_n,
    bow_reference_deflection_m,
    centralizer_effective_contact_diameter_m,
    centralizer_running_force_ratio,
    equivalent_bow_support_stiffness_n_per_m,
    resolved_blade_power_law_k,
)
from .io import load_case_bundle, write_json
from .coupling import run_coupled_global_baseline
from .mechanics import (
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
)
from .models import (
    CentralizerSummaryModel,
    LoadedCase,
    StringSummaryModel,
    TrajectorySummaryModel,
)
from .torque_drag import (
    AxialForcePointModel,
    TorquePointModel,
)

try:
    from . import _core as cpp_core  # type: ignore[attr-defined]
except ImportError:
    cpp_core = None


PHASE9_WARNINGS = [
    "Phase 9 trajectory coordinates are still approximated by balanced-tangent integration of MD, inclination, and azimuth.",
    "The local trajectory frame uses a reduced parallel-transport construction, not a full differential-geometry reference implementation.",
    "The lateral equilibrium is now solved with two transverse displacement components in the local normal/binormal plane, but it is still not a full 6-DOF beam formulation.",
    "The bending term still uses the simply supported beam equivalence delta_max = 5 q L^4 / (384 E I), so the 384/5 factor is a reduced structural hypothesis.",
    "Contact still uses reduced vector active-set penalty iterations in the annulus, but detailed centralizer support loads are now post-processed bow by bow in the local frame.",
    "Each bow is distributed uniformly in angle from the centralizer angular reference and uses the reduced deflection law delta_i = max(0, e . r_i - (c_support + c_inner)).",
    "The bow constitutive law is reduced to F_i = k_blade * delta_i^p, with optional direct k/p input or fallback calibration from nominal restoring force at a reference deflection.",
    "In the reduced torque-drag model, pipe-body drag still uses mu * N_body, while detailed centralizer drag/torque use the bow-resultant magnitude scaled by the nominal running/restoring-force ratio.",
    "Run in/slackoff subtracts body plus centralizer axial friction from the local hookload increment because friction opposes downward motion; pull out/pickup adds them because friction opposes upward motion.",
    "Rotational torque is still reduced: body torque uses a pipe-body contact radius and centralizer torque uses the bow-resultant tangential contribution times an effective centralizer contact radius.",
    "Only the axial profile for the selected operation mode is iterated inside the reduced coupling loop; the other operational profiles remain reduced post-processing against the converged normal-reaction field.",
    "Detailed bows are modeled individually, but the structural solve is still not a full 6-DOF beam/contact formulation and the global friction law remains reduced.",
]

PHASE9_TODOS = [
    "TODO: evolve the reduced 2-DOF transverse model toward a fuller 3D beam formulation with rotations and torsion.",
    "TODO: refine vector contact with stronger nonlinear iteration and wall-reaction handling.",
    "TODO: calibrate the bow-spring power-law parameters against manufacturer/API data.",
    "TODO: refine axial drag with stronger bidirectional coupling between contact state and friction propagation.",
    "TODO: extend reduced torque and drag toward fuller bow-resolved nonlinear workflows and vector tangential contact.",
    "TODO: implement design-space optimization workflow.",
]

PHASE10_VALIDATION_STATUS = "phase10-benchmark-calibration-infrastructure"
GLOBAL_SOLVER_UPDATE_TOLERANCE_M = 1.0e-8


def _core_supports_phase9() -> bool:
    if cpp_core is None:
        return False
    try:
        solver_input = cpp_core.SolverStubInput()
        result = cpp_core.SolverStubResult()
        summary = cpp_core.MechanicalSummary()
        settings = cpp_core.DiscretizationSettings()
        segment = cpp_core.MechanicalSegmentResult()
    except Exception:
        return False
    return all(
        hasattr(instance, attribute)
        for instance, attribute in (
            (solver_input, "fluid_density_kg_per_m3"),
            (solver_input, "discretization_settings"),
            (solver_input, "operation_mode"),
            (solver_input, "reference_hole_diameter_m"),
            (result, "minimum_standoff_estimate"),
            (result, "hookload_run_in_n"),
            (result, "torque_profile"),
            (result, "centralizer_model_status"),
            (result, "centralizer_torque_profile"),
            (result, "coupling_status"),
            (result, "coupling_final_max_profile_update_n"),
            (result, "converged_axial_profile"),
            (summary, "maximum_normal_reaction_estimate_n"),
            (summary, "global_solver_iteration_count"),
            (segment, "contact_state"),
            (segment, "bow_force_details"),
            (segment, "centralizer_torque_increment_n_m"),
            (settings, "target_segment_length_m"),
            (settings, "global_solver_max_iterations"),
            (settings, "contact_penalty_scale"),
            (settings, "coupling_max_iterations"),
            (settings, "coupling_tolerance_n"),
            (settings, "relaxation_factor"),
            (settings, "frame_method"),
        )
    )


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
            "centralizer_axial_friction_n": segment.centralizer_axial_friction_n,
            "centralizer_tangential_friction_n": segment.centralizer_tangential_friction_n,
            "centralizer_torque_increment_n_m": segment.centralizer_torque_increment_n_m,
            "centralizer_effective_contact_radius_m": segment.centralizer_effective_contact_radius_m,
            "nearby_centralizer_count": segment.nearby_centralizer_count,
            "contact_iteration_count": segment.contact_iteration_count,
            "contact_state": segment.contact_state,
            "support_in_contact": segment.support_in_contact,
            "pipe_body_in_contact": segment.pipe_body_in_contact,
        }
        for segment in profile
    ]


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


def _centralizer_friction_profile_to_dict(profile: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "measured_depth_m": point.measured_depth_m,
            "centralizer_axial_friction_n": point.centralizer_axial_friction_n,
            "centralizer_tangential_friction_n": point.centralizer_tangential_friction_n,
        }
        for point in profile
    ]


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
                "relaxation_factor": loaded_case.relaxation_factor,
                "final_max_profile_update_n": result["coupling_final_max_profile_update_n"],
                "converged": result["coupling_converged"],
                "status": result["coupling_status"],
            },
        },
        "centralizer_parameters": _centralizer_traceability_to_dict(loaded_case),
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


def _build_cpp_input(loaded_case: LoadedCase) -> Any:
    points = []
    for node in loaded_case.trajectory_nodes():
        cpp_point = cpp_core.WellTrajectoryPoint()
        cpp_point.measured_depth_m = node.measured_depth_m
        cpp_point.inclination_rad = node.inclination_rad
        cpp_point.azimuth_rad = node.azimuth_rad
        cpp_point.tvd_m = node.tvd_m
        cpp_point.northing_m = node.northing_m
        cpp_point.easting_m = node.easting_m
        points.append(cpp_point)

    well = cpp_core.WellTrajectory(points)

    sections = []
    for section in loaded_case.string.sections:
        cpp_section = cpp_core.StringSection()
        cpp_section.name = section.name
        cpp_section.md_start_m = section.md_start_m
        cpp_section.md_end_m = section.md_end_m
        cpp_section.outer_diameter_m = section.outer_diameter_m
        cpp_section.inner_diameter_m = section.inner_diameter_m
        cpp_section.linear_weight_n_per_m = section.linear_weight_n_per_m
        cpp_section.young_modulus_pa = section.young_modulus_pa
        cpp_section.shear_modulus_pa = section.shear_modulus_pa
        cpp_section.density_kg_per_m3 = section.density_kg_per_m3
        cpp_section.friction_coefficient = section.friction_coefficient
        sections.append(cpp_section)

    centralizers = []
    for spec in loaded_case.centralizers.centralizers:
        cpp_spec = cpp_core.CentralizerSpec()
        cpp_spec.name = spec.name
        cpp_spec.type = spec.type
        cpp_spec.outer_diameter_m = spec.outer_diameter_m
        cpp_spec.support_outer_diameter_m = spec.support_outer_diameter_m
        cpp_spec.number_of_bows = spec.number_of_bows
        cpp_spec.angular_orientation_reference_deg = spec.angular_orientation_reference_deg
        cpp_spec.inner_clearance_to_pipe_m = spec.inner_clearance_to_pipe_m
        cpp_spec.nominal_restoring_force_n = spec.nominal_restoring_force_n
        cpp_spec.nominal_running_force_n = spec.nominal_running_force_n
        cpp_spec.blade_power_law_k = spec.blade_power_law_k
        cpp_spec.blade_power_law_p = spec.blade_power_law_p
        cpp_spec.min_contact_diameter_m = spec.min_contact_diameter_m
        cpp_spec.max_contact_diameter_m = spec.max_contact_diameter_m
        cpp_spec.spacing_hint_m = spec.spacing_hint_m
        cpp_spec.count_hint = spec.count_hint
        cpp_spec.installation_md_m = list(spec.installation_md_m or [])
        centralizers.append(cpp_spec)

    settings = cpp_core.DiscretizationSettings()
    settings.target_segment_length_m = loaded_case.discretization_step_m
    settings.global_solver_max_iterations = loaded_case.global_solver_max_iterations
    settings.contact_penalty_scale = loaded_case.contact_penalty_scale
    settings.coupling_max_iterations = loaded_case.coupling_max_iterations
    settings.coupling_tolerance_n = loaded_case.coupling_tolerance_n
    settings.relaxation_factor = loaded_case.relaxation_factor
    settings.frame_method = loaded_case.frame_method

    payload = cpp_core.SolverStubInput()
    payload.well = well
    payload.reference_hole_diameter_m = loaded_case.reference_hole_diameter_m
    payload.fluid_density_kg_per_m3 = loaded_case.fluid_density_kg_per_m3
    payload.discretization_settings = settings
    payload.operation_mode = loaded_case.operation_mode
    payload.string_sections = sections
    payload.centralizers = centralizers
    return payload


def _python_mechanical_result(loaded_case: LoadedCase) -> dict[str, Any]:
    trajectory_summary = loaded_case.trajectory_summary()
    string_summary = loaded_case.string_summary()
    section_summaries = loaded_case.section_summaries()
    coupling_result = run_coupled_global_baseline(loaded_case)
    centralizer_summary = loaded_case.centralizer_summary()
    centralizer_summary.expanded_installation_count = len(coupling_result.placements)
    warnings = list(PHASE9_WARNINGS)
    if loaded_case.reference_hole_diameter_m <= 0.0:
        warnings.append(
            "Reference hole diameter is absent, so annular contact and standoff are reported as undefined-safe defaults."
        )

    return {
        "backend": "python-fallback",
        "status": "phase9-vector-bow-spring-td-baseline",
        "message": (
            "Phase 9 reduced vector-frame torque and drag baseline with detailed bow-spring "
            "centralizer modeling. The lateral/contact solve still uses two transverse "
            "displacement components in the local trajectory frame, but support reactions and "
            "centralizer torque now come from bow-by-bow reduced resultants. This remains a "
            "reduced engineering baseline, not a full commercial stiff-string/contact solver."
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
        "centralizer_axial_friction_profile": _centralizer_friction_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_axial_friction_profile
        ),
        "centralizer_tangential_friction_profile": _centralizer_friction_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_tangential_friction_profile
        ),
        "centralizer_torque_profile": _torque_profile_to_dict(
            coupling_result.torque_drag_result.centralizer_torque_profile
        ),
        "coupling_status": coupling_result.status,
        "coupling_iterations": coupling_result.iteration_count,
        "coupling_final_max_profile_update_n": coupling_result.maximum_profile_update_n,
        "coupling_converged": coupling_result.converged,
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
        "centralizer_model_status": "phase9-detailed-bow-spring",
        "torque_and_drag_real_implemented": False,
        "torque_and_drag_status": coupling_result.torque_drag_result.status,
        "torque_drag_status": coupling_result.torque_drag_result.status,
        "warnings": warnings,
        "todos": list(PHASE9_TODOS),
    }


def _cpp_mechanical_result(loaded_case: LoadedCase) -> dict[str, Any]:
    result = cpp_core.run_solver_stub(_build_cpp_input(loaded_case))
    return {
        "backend": "cpp",
        "status": result.status,
        "message": result.message,
        "operation_mode": result.operation_mode,
        "geometry_is_approximate": result.geometry_is_approximate,
        "trajectory_summary": _trajectory_summary_to_dict(result.trajectory_summary),
        "string_summary": _string_summary_to_dict(result.string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(result.centralizer_summary),
        "mechanical_summary": _mechanical_summary_to_dict(result.mechanical_summary),
        "section_summaries": _section_summaries_to_dict(list(result.section_summaries)),
        "mechanical_profile": _mechanical_profile_to_dict(list(result.mechanical_profile)),
        "estimated_hookload_n": result.estimated_hookload_n,
        "hookload_run_in_n": result.hookload_run_in_n,
        "hookload_pull_out_n": result.hookload_pull_out_n,
        "drag_run_in_n": result.drag_run_in_n,
        "drag_pull_out_n": result.drag_pull_out_n,
        "axial_force_run_in_profile": _axial_force_profile_to_dict(list(result.axial_force_run_in_profile)),
        "axial_force_pull_out_profile": _axial_force_profile_to_dict(list(result.axial_force_pull_out_profile)),
        "torque_profile": _torque_profile_to_dict(list(result.torque_profile)),
        "centralizer_axial_friction_profile": _centralizer_friction_profile_to_dict(
            list(result.centralizer_axial_friction_profile)
        ),
        "centralizer_tangential_friction_profile": _centralizer_friction_profile_to_dict(
            list(result.centralizer_tangential_friction_profile)
        ),
        "centralizer_torque_profile": _torque_profile_to_dict(list(result.centralizer_torque_profile)),
        "coupling_status": result.coupling_status,
        "coupling_iterations": result.coupling_iterations,
        "coupling_final_max_profile_update_n": result.coupling_final_max_profile_update_n,
        "coupling_converged": result.coupling_converged,
        "converged_axial_profile": _axial_force_profile_to_dict(list(result.converged_axial_profile)),
        "converged_normal_reaction_profile": _normal_reaction_profile_to_dict(
            list(result.converged_normal_reaction_profile)
        ),
        "converged_torque_profile": _torque_profile_to_dict(list(result.converged_torque_profile)),
        "estimated_surface_torque_n_m": result.estimated_surface_torque_n_m,
        "minimum_standoff_estimate": result.minimum_standoff_estimate,
        "minimum_nominal_radial_clearance_m": result.minimum_nominal_radial_clearance_m,
        "contact_nodes": result.contact_nodes,
        "centralizer_model_status": result.centralizer_model_status,
        "torque_and_drag_real_implemented": result.torque_and_drag_real_implemented,
        "torque_and_drag_status": result.torque_and_drag_status,
        "torque_drag_status": result.torque_drag_status,
        "warnings": list(result.warnings),
        "todos": list(result.todos),
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
    result = _cpp_mechanical_result(loaded_case) if _core_supports_phase9() else _python_mechanical_result(loaded_case)
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
            "relaxation_factor": loaded_case.relaxation_factor,
            "frame_method": loaded_case.frame_method,
        },
        **result,
        **derived_profiles,
        "traceability": _traceability_to_dict(loaded_case, result),
        "updated_estimated_surface_torque_n_m": result["estimated_surface_torque_n_m"],
    }

    output_path = _resolve_output_path(loaded_case, output)
    write_json(output_path, payload)
    return loaded_case, payload, output_path
