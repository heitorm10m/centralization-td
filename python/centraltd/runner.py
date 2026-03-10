from __future__ import annotations

from pathlib import Path
from typing import Any

from .io import load_case_bundle, write_json
from .models import (
    LoadedCase,
    TrajectorySummaryModel,
    StringSummaryModel,
    CentralizerSummaryModel,
)

try:
    from . import _core as cpp_core  # type: ignore[attr-defined]
except ImportError:
    cpp_core = None


PHASE2_WARNINGS = [
    "Phase 2 trajectory coordinates are approximated by balanced-tangent integration of MD, inclination, and azimuth.",
    "Nominal radial clearance is purely geometric and does not represent real contact or standoff.",
    "contact_nodes is a curvature-based risk counter, not real contact detection.",
    "estimated_surface_torque_n_m is a geometry-derived placeholder, not a torque and drag calculation.",
]

PHASE3_TODOS = [
    "TODO: implement stiff-string formulation.",
    "TODO: implement contact detection and reaction forces.",
    "TODO: implement real side force prediction.",
    "TODO: implement real standoff evaluation from contact geometry.",
    "TODO: implement real torque and drag calculations.",
    "TODO: implement design-space optimization workflow.",
]


def _core_supports_phase2() -> bool:
    if cpp_core is None:
        return False
    try:
        section = cpp_core.StringSection()
        centralizer = cpp_core.CentralizerSpec()
        solver_input = cpp_core.SolverStubInput()
    except Exception:
        return False
    return all(
        hasattr(instance, attribute)
        for instance, attribute in (
            (section, "md_start_m"),
            (section, "linear_weight_n_per_m"),
            (centralizer, "nominal_restoring_force_n"),
            (solver_input, "reference_hole_diameter_m"),
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
        "max_outer_diameter_m": summary.max_outer_diameter_m,
        "min_nominal_radial_clearance_m": summary.min_nominal_radial_clearance_m,
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
        f"String: {loaded_case.string.name}",
        f"String sections: {string_summary.section_count}",
        f"Total string length [m]: {string_summary.total_length_m:.2f}",
        f"Total string weight placeholder [N]: {string_summary.total_weight_n:.2f}",
        f"Max string OD [m]: {string_summary.max_outer_diameter_m:.4f}",
        f"Average friction coefficient [-]: {string_summary.average_friction_coefficient:.3f}",
        f"Centralizer specs: {centralizer_summary.spec_count}",
        f"Explicit installation count: {centralizer_summary.explicit_installation_count}",
        f"Count-hint total: {centralizer_summary.count_hint_total}",
        f"Spacing-based installation estimate: {centralizer_summary.spacing_based_installation_estimate}",
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

    payload = cpp_core.SolverStubInput()
    payload.well = well
    payload.reference_hole_diameter_m = loaded_case.reference_hole_diameter_m
    payload.string_sections = sections
    payload.centralizers = centralizers
    return payload


def _python_stub_result(loaded_case: LoadedCase) -> dict[str, Any]:
    trajectory_summary = loaded_case.trajectory_summary()
    string_summary = loaded_case.string_summary()
    centralizer_summary = loaded_case.centralizer_summary()
    section_summaries = loaded_case.section_summaries()
    minimum_nominal_radial_clearance_m = loaded_case.minimum_nominal_radial_clearance_m()
    curvature_factor = 1.0 + (250.0 * trajectory_summary.max_curvature_rad_per_m)
    lateral_factor = (
        1.0
        if trajectory_summary.final_measured_depth_m <= 0.0
        else 1.0 + (0.5 * trajectory_summary.lateral_displacement_m / trajectory_summary.final_measured_depth_m)
    )
    reference_radius_m = 0.5 * max(string_summary.max_outer_diameter_m, 0.0)
    contact_nodes = sum(
        1 for node in loaded_case.trajectory_nodes() if node.discrete_curvature_rad_per_m >= 3.5e-4
    )
    warnings = list(PHASE2_WARNINGS)
    if loaded_case.reference_hole_diameter_m <= 0.0:
        warnings.append(
            "Reference hole diameter is absent, so nominal clearance metrics are reported as zero."
        )

    return {
        "backend": "python-fallback",
        "status": "phase2-baseline",
        "message": (
            "Phase 2 geometry baseline. Outputs derive from survey and section geometry only, "
            "not from a stiff-string or torque and drag solver."
        ),
        "geometry_is_approximate": True,
        "trajectory_summary": _trajectory_summary_to_dict(trajectory_summary),
        "string_summary": _string_summary_to_dict(string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(centralizer_summary),
        "section_summaries": _section_summaries_to_dict(section_summaries),
        "estimated_hookload_n": string_summary.total_weight_n,
        "estimated_surface_torque_n_m": (
            string_summary.total_weight_n
            * string_summary.average_friction_coefficient
            * curvature_factor
            * lateral_factor
            * reference_radius_m
        ),
        "minimum_standoff_ratio": (
            0.0
            if loaded_case.reference_hole_diameter_m <= 0.0 or centralizer_summary.max_outer_diameter_m <= 0.0
            else min(1.0, centralizer_summary.max_outer_diameter_m / loaded_case.reference_hole_diameter_m)
        ),
        "minimum_nominal_radial_clearance_m": minimum_nominal_radial_clearance_m,
        "contact_nodes": contact_nodes,
        "warnings": warnings,
        "todos": list(PHASE3_TODOS),
    }


def _cpp_stub_result(loaded_case: LoadedCase) -> dict[str, Any]:
    result = cpp_core.run_solver_stub(_build_cpp_input(loaded_case))
    return {
        "backend": "cpp",
        "status": result.status,
        "message": result.message,
        "geometry_is_approximate": result.geometry_is_approximate,
        "trajectory_summary": _trajectory_summary_to_dict(result.trajectory_summary),
        "string_summary": _string_summary_to_dict(result.string_summary),
        "centralizer_summary": _centralizer_summary_to_dict(result.centralizer_summary),
        "section_summaries": _section_summaries_to_dict(list(result.section_summaries)),
        "estimated_hookload_n": result.estimated_hookload_n,
        "estimated_surface_torque_n_m": result.estimated_surface_torque_n_m,
        "minimum_standoff_ratio": result.minimum_standoff_ratio,
        "minimum_nominal_radial_clearance_m": result.minimum_nominal_radial_clearance_m,
        "contact_nodes": result.contact_nodes,
        "warnings": list(result.warnings),
        "todos": list(result.todos),
    }


def _resolve_output_path(loaded_case: LoadedCase, output: str | Path | None) -> Path:
    if output is not None:
        return Path(output).resolve()
    if loaded_case.definition.output_json:
        return (loaded_case.case_path.parent / loaded_case.definition.output_json).resolve()
    return loaded_case.case_path.with_suffix(".stub.json")


def run_stub_case(case_path: str | Path, output: str | Path | None = None) -> tuple[LoadedCase, dict[str, Any], Path]:
    loaded_case = load_case_bundle(case_path)
    result = _cpp_stub_result(loaded_case) if _core_supports_phase2() else _python_stub_result(loaded_case)

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
        },
        **result,
    }

    output_path = _resolve_output_path(loaded_case, output)
    write_json(output_path, payload)
    return loaded_case, payload, output_path
