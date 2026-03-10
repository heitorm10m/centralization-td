from __future__ import annotations

from pathlib import Path
from typing import Any

from .io import load_case_bundle, write_json
from .mechanics import (
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
    run_mechanical_baseline,
)
from .models import (
    CentralizerSummaryModel,
    LoadedCase,
    StringSummaryModel,
    TrajectorySummaryModel,
)

try:
    from . import _core as cpp_core  # type: ignore[attr-defined]
except ImportError:
    cpp_core = None


PHASE5_WARNINGS = [
    "Phase 5 trajectory coordinates are still approximated by balanced-tangent integration of MD, inclination, and azimuth.",
    "The lateral equilibrium is solved with a reduced global scalar displacement model along MD, not with a full 3D stiff-string formulation.",
    "The bending term still uses the simply supported beam equivalence delta_max = 5 q L^4 / (384 E I), so the 384/5 factor is a reduced structural hypothesis.",
    "Contact uses simple global active-set penalty iterations against annular clearance and nominal centralizer support, not a fully nonlinear contact/friction algorithm.",
    "Centralizers are represented as nominal centering/support effects only and do not use a detailed bow-spring constitutive model.",
    "Torque and drag remain intentionally unimplemented in this phase.",
]

PHASE5_TODOS = [
    "TODO: refine the reduced global solver toward fuller stiff-string equilibrium with stronger kinematic coupling.",
    "TODO: refine contact with stronger nonlinear iteration and wall-reaction handling.",
    "TODO: implement real side-force prediction with friction coupling.",
    "TODO: refine centralizer behavior beyond nominal support stiffness.",
    "TODO: implement real torque and drag calculations.",
    "TODO: implement design-space optimization workflow.",
]


def _core_supports_phase5() -> bool:
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
            (result, "minimum_standoff_estimate"),
            (summary, "maximum_normal_reaction_estimate_n"),
            (summary, "global_solver_iteration_count"),
            (segment, "contact_state"),
            (settings, "target_segment_length_m"),
            (settings, "global_solver_max_iterations"),
            (settings, "contact_penalty_scale"),
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
            "effective_line_weight_n_per_m": segment.effective_line_weight_n_per_m,
            "effective_axial_load_n": segment.effective_axial_load_n,
            "bending_stiffness_n_m2": segment.bending_stiffness_n_m2,
            "bending_moment_n_m": segment.bending_moment_n_m,
            "bending_stress_pa": segment.bending_stress_pa,
            "bending_strain_estimate": segment.bending_strain_estimate,
            "gravity_lateral_load_n_per_m": segment.gravity_lateral_load_n_per_m,
            "curvature_lateral_load_n_per_m": segment.curvature_lateral_load_n_per_m,
            "equivalent_lateral_load_n_per_m": segment.equivalent_lateral_load_n_per_m,
            "equivalent_lateral_force_n": segment.equivalent_lateral_force_n,
            "bending_lateral_stiffness_n_per_m": segment.bending_lateral_stiffness_n_per_m,
            "axial_tension_lateral_stiffness_n_per_m": segment.axial_tension_lateral_stiffness_n_per_m,
            "structural_lateral_stiffness_n_per_m": segment.structural_lateral_stiffness_n_per_m,
            "centralizer_centering_stiffness_n_per_m": segment.centralizer_centering_stiffness_n_per_m,
            "support_contact_penalty_n_per_m": segment.support_contact_penalty_n_per_m,
            "body_contact_penalty_n_per_m": segment.body_contact_penalty_n_per_m,
            "support_outer_diameter_m": segment.support_outer_diameter_m,
            "pipe_body_clearance_m": segment.pipe_body_clearance_m,
            "support_contact_clearance_m": segment.support_contact_clearance_m,
            "free_eccentricity_estimate_m": segment.free_eccentricity_estimate_m,
            "eccentricity_estimate_m": segment.eccentricity_estimate_m,
            "eccentricity_ratio": segment.eccentricity_ratio,
            "standoff_estimate": segment.standoff_estimate,
            "support_normal_reaction_estimate_n": segment.support_normal_reaction_estimate_n,
            "body_normal_reaction_estimate_n": segment.body_normal_reaction_estimate_n,
            "normal_reaction_estimate_n": segment.normal_reaction_estimate_n,
            "normal_reaction_estimate_n_per_m": segment.normal_reaction_estimate_n_per_m,
            "nearby_centralizer_count": segment.nearby_centralizer_count,
            "contact_iteration_count": segment.contact_iteration_count,
            "contact_state": segment.contact_state,
            "support_in_contact": segment.support_in_contact,
            "pipe_body_in_contact": segment.pipe_body_in_contact,
        }
        for segment in profile
    ]


def _derived_profiles(mechanical_profile: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    return {
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
        "bending_strain_profile": [
            {
                "measured_depth_m": segment["measured_depth_center_m"],
                "bending_strain_estimate": segment["bending_strain_estimate"],
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
        f"Discretization step [m]: {loaded_case.discretization_step_m:.2f}",
        f"Global solver max iterations [-]: {loaded_case.global_solver_max_iterations}",
        f"Contact penalty scale [-]: {loaded_case.contact_penalty_scale:.2f}",
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
        cpp_spec.nominal_restoring_force_n = spec.nominal_restoring_force_n
        cpp_spec.nominal_running_force_n = spec.nominal_running_force_n
        cpp_spec.spacing_hint_m = spec.spacing_hint_m
        cpp_spec.count_hint = spec.count_hint
        cpp_spec.installation_md_m = list(spec.installation_md_m or [])
        centralizers.append(cpp_spec)

    settings = cpp_core.DiscretizationSettings()
    settings.target_segment_length_m = loaded_case.discretization_step_m
    settings.global_solver_max_iterations = loaded_case.global_solver_max_iterations
    settings.contact_penalty_scale = loaded_case.contact_penalty_scale

    payload = cpp_core.SolverStubInput()
    payload.well = well
    payload.reference_hole_diameter_m = loaded_case.reference_hole_diameter_m
    payload.fluid_density_kg_per_m3 = loaded_case.fluid_density_kg_per_m3
    payload.discretization_settings = settings
    payload.string_sections = sections
    payload.centralizers = centralizers
    return payload


def _python_mechanical_result(loaded_case: LoadedCase) -> dict[str, Any]:
    trajectory_summary = loaded_case.trajectory_summary()
    string_summary = loaded_case.string_summary()
    section_summaries = loaded_case.section_summaries()
    mechanical_summary, mechanical_profile, placements = run_mechanical_baseline(loaded_case)
    centralizer_summary = loaded_case.centralizer_summary()
    centralizer_summary.expanded_installation_count = len(placements)
    warnings = list(PHASE5_WARNINGS)
    if loaded_case.reference_hole_diameter_m <= 0.0:
        warnings.append(
            "Reference hole diameter is absent, so annular contact and standoff are reported as undefined-safe defaults."
        )

    return {
        "backend": "python-fallback",
        "status": "phase5-global-stiff-string-baseline",
        "message": (
            "Phase 5 global lateral-equilibrium baseline. The column is discretized along MD and "
            "solved with a reduced global scalar bending-plus-tension model, nominal centralizer "
            "support, and simple annular contact iteration. This is still not a full 3D stiff-string "
            "or torque and drag solver."
        ),
        "geometry_is_approximate": True,
        "trajectory_summary": _trajectory_summary_to_dict(trajectory_summary),
        "string_summary": _string_summary_to_dict(string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(centralizer_summary),
        "mechanical_summary": _mechanical_summary_to_dict(mechanical_summary),
        "section_summaries": _section_summaries_to_dict(section_summaries),
        "mechanical_profile": _mechanical_profile_to_dict(mechanical_profile),
        "estimated_hookload_n": mechanical_summary.top_effective_axial_load_n,
        "estimated_surface_torque_n_m": None,
        "minimum_standoff_estimate": mechanical_summary.minimum_standoff_estimate,
        "minimum_nominal_radial_clearance_m": loaded_case.minimum_nominal_radial_clearance_m(),
        "contact_nodes": mechanical_summary.contact_segment_count,
        "torque_and_drag_real_implemented": False,
        "torque_and_drag_status": "not-implemented-yet",
        "warnings": warnings,
        "todos": list(PHASE5_TODOS),
    }


def _cpp_mechanical_result(loaded_case: LoadedCase) -> dict[str, Any]:
    result = cpp_core.run_solver_stub(_build_cpp_input(loaded_case))
    return {
        "backend": "cpp",
        "status": result.status,
        "message": result.message,
        "geometry_is_approximate": result.geometry_is_approximate,
        "trajectory_summary": _trajectory_summary_to_dict(result.trajectory_summary),
        "string_summary": _string_summary_to_dict(result.string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(result.centralizer_summary),
        "mechanical_summary": _mechanical_summary_to_dict(result.mechanical_summary),
        "section_summaries": _section_summaries_to_dict(list(result.section_summaries)),
        "mechanical_profile": _mechanical_profile_to_dict(list(result.mechanical_profile)),
        "estimated_hookload_n": result.estimated_hookload_n,
        "estimated_surface_torque_n_m": result.estimated_surface_torque_n_m,
        "minimum_standoff_estimate": result.minimum_standoff_estimate,
        "minimum_nominal_radial_clearance_m": result.minimum_nominal_radial_clearance_m,
        "contact_nodes": result.contact_nodes,
        "torque_and_drag_real_implemented": result.torque_and_drag_real_implemented,
        "torque_and_drag_status": result.torque_and_drag_status,
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
    result = _cpp_mechanical_result(loaded_case) if _core_supports_phase5() else _python_mechanical_result(loaded_case)
    derived_profiles = _derived_profiles(result["mechanical_profile"])

    payload = {
        "case": loaded_case.definition.name,
        "well": loaded_case.well.name,
        "string": loaded_case.string.name,
        "centralizer_config": loaded_case.centralizers.name,
        "inputs": {
            "case_path": str(loaded_case.case_path),
            "well_path": str(loaded_case.well_path),
            "string_path": str(loaded_case.string_path),
            "centralizers_path": str(loaded_case.centralizers_path),
            "reference_hole_diameter_m": loaded_case.reference_hole_diameter_m,
            "fluid_density_kg_per_m3": loaded_case.fluid_density_kg_per_m3,
            "discretization_step_m": loaded_case.discretization_step_m,
            "global_solver_max_iterations": loaded_case.global_solver_max_iterations,
            "contact_penalty_scale": loaded_case.contact_penalty_scale,
        },
        **result,
        **derived_profiles,
    }

    output_path = _resolve_output_path(loaded_case, output)
    write_json(output_path, payload)
    return loaded_case, payload, output_path
