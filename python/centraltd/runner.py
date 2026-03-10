from __future__ import annotations

from pathlib import Path
from typing import Any

from .io import load_case_bundle, write_json
from .models import LoadedCase

try:
    from . import _core as cpp_core  # type: ignore[attr-defined]
except ImportError:
    cpp_core = None


def load_case(case_path: str | Path) -> LoadedCase:
    return load_case_bundle(case_path)


def format_case_summary(loaded_case: LoadedCase) -> str:
    lines = [
        f"Case: {loaded_case.definition.name}",
        f"Well: {loaded_case.well.name}",
        f"Trajectory points: {len(loaded_case.well.trajectory)}",
        f"Final measured depth [m]: {loaded_case.well.final_measured_depth_m:.2f}",
        f"String: {loaded_case.string.name}",
        f"String sections: {len(loaded_case.string.sections)}",
        f"Total string length [m]: {loaded_case.total_string_length_m():.2f}",
        f"Total string weight placeholder [N]: {loaded_case.total_string_weight_n():.2f}",
        f"Centralizer sets: {len(loaded_case.centralizers.centralizers)}",
        f"Total centralizers: {loaded_case.total_centralizer_count()}",
    ]
    return "\n".join(lines)


def _build_cpp_input(loaded_case: LoadedCase) -> Any:
    points = []
    for point in loaded_case.well.trajectory:
        cpp_point = cpp_core.WellTrajectoryPoint()
        cpp_point.measured_depth_m = point.measured_depth_m
        cpp_point.inclination_rad = point.inclination_rad
        cpp_point.azimuth_rad = point.azimuth_rad
        cpp_point.tvd_m = point.tvd_m
        cpp_point.northing_m = point.northing_m
        cpp_point.easting_m = point.easting_m
        points.append(cpp_point)

    well = cpp_core.WellTrajectory(points)

    sections = []
    for section in loaded_case.string.sections:
        cpp_section = cpp_core.StringSection()
        cpp_section.name = section.name
        cpp_section.length_m = section.length_m
        cpp_section.outer_diameter_m = section.outer_diameter_m
        cpp_section.inner_diameter_m = section.inner_diameter_m
        cpp_section.unit_weight_n_per_m = section.unit_weight_n_per_m
        cpp_section.grade = section.grade
        sections.append(cpp_section)

    centralizers = []
    for spec in loaded_case.centralizers.centralizers:
        cpp_spec = cpp_core.CentralizerSpec()
        cpp_spec.name = spec.name
        cpp_spec.outer_diameter_m = spec.outer_diameter_m
        cpp_spec.start_md_m = spec.start_md_m
        cpp_spec.spacing_m = spec.spacing_m
        cpp_spec.count = spec.count
        cpp_spec.type = spec.type
        centralizers.append(cpp_spec)

    payload = cpp_core.SolverStubInput()
    payload.well = well
    payload.string_sections = sections
    payload.centralizers = centralizers
    return payload


def _python_stub_result(loaded_case: LoadedCase) -> dict[str, Any]:
    hookload_n = loaded_case.total_string_weight_n()
    string_length_m = loaded_case.total_string_length_m()
    final_md_m = loaded_case.well.final_measured_depth_m
    centralizer_count = loaded_case.total_centralizer_count()
    coverage_ratio = min(1.0, (centralizer_count * 15.0) / string_length_m) if string_length_m > 0.0 else 0.0

    return {
        "backend": "python-fallback",
        "status": "stub",
        "message": "Phase 1 placeholder output. Values are deterministic scaffolding, not calibrated physics.",
        "estimated_hookload_n": hookload_n,
        "estimated_surface_torque_n_m": 0.02 * hookload_n * (1.0 + (final_md_m / 1000.0)),
        "minimum_standoff_ratio": 0.0 if centralizer_count == 0 else min(0.95, 0.10 + coverage_ratio),
        "contact_nodes": max(0, len(loaded_case.well.trajectory) - 1),
        "todos": [
            "TODO: implement stiff-string formulation.",
            "TODO: implement contact detection and reaction forces.",
            "TODO: implement standoff evaluation from contact geometry.",
            "TODO: implement detailed bow-spring centralizer model.",
            "TODO: implement torque and drag calculations.",
            "TODO: implement design-space optimization workflow.",
        ],
    }


def _cpp_stub_result(loaded_case: LoadedCase) -> dict[str, Any]:
    result = cpp_core.run_solver_stub(_build_cpp_input(loaded_case))
    return {
        "backend": "cpp",
        "status": result.status,
        "message": result.message,
        "estimated_hookload_n": result.estimated_hookload_n,
        "estimated_surface_torque_n_m": result.estimated_surface_torque_n_m,
        "minimum_standoff_ratio": result.minimum_standoff_ratio,
        "contact_nodes": result.contact_nodes,
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
    result = _cpp_stub_result(loaded_case) if cpp_core is not None else _python_stub_result(loaded_case)

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
        },
        **result,
    }

    output_path = _resolve_output_path(loaded_case, output)
    write_json(output_path, payload)
    return loaded_case, payload, output_path

