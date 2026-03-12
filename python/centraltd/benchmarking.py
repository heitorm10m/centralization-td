from __future__ import annotations

from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any

from .io import read_yaml, write_json
from .models import ConfigError
from .runner import run_stub_case


def _expect_mapping(raw: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(raw, Mapping):
        raise ConfigError(f"{context} must be a mapping.")
    return raw


def _expect_sequence(raw: Any, context: str) -> Sequence[Any]:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
        raise ConfigError(f"{context} must be a sequence.")
    return raw


def _require_text(raw: Mapping[str, Any], key: str, context: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{context}.{key} must be a non-empty string.")
    return value.strip()


def _optional_text(raw: Mapping[str, Any], key: str, context: str) -> str | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, str):
        raise ConfigError(f"{context}.{key} must be a string when provided.")
    return value.strip()


def _optional_float(raw: Mapping[str, Any], key: str, context: str) -> float | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be numeric when provided.")
    return float(value)


def _resolve_suite_output_dir(suite_path: Path, output_dir: str | Path | None) -> Path:
    if output_dir is not None:
        return Path(output_dir).resolve()
    return (suite_path.parent / f"{suite_path.stem}_outputs").resolve()


def _nested_get(payload: Mapping[str, Any], path: str) -> Any:
    current: Any = payload
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            raise ConfigError(f"Metric path not found in benchmark payload: {path}")
        current = current[part]
    return current


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _max_value(items: Sequence[Mapping[str, Any]], key: str) -> float:
    if not items:
        return 0.0
    return max(float(item[key]) for item in items)


def _derived_metrics(payload: Mapping[str, Any]) -> dict[str, float]:
    centralizer_parameters = payload.get("traceability", {}).get("centralizer_parameters", [])
    mechanical_profile = payload["mechanical_profile"]
    torque_partition_summary = payload.get("torque_partition_summary", {})
    return {
        "hookload_differential_n": float(payload["hookload_pull_out_n"]) - float(payload["hookload_run_in_n"]),
        "maximum_normal_reaction_estimate_n": float(payload["mechanical_summary"]["maximum_normal_reaction_estimate_n"]),
        "maximum_equivalent_lateral_load_n_per_m": float(
            payload["mechanical_summary"]["maximum_equivalent_lateral_load_n_per_m"]
        ),
        "maximum_eccentricity_estimate_m": float(payload["mechanical_summary"]["maximum_eccentricity_estimate_m"]),
        "minimum_standoff_estimate": float(payload["mechanical_summary"]["minimum_standoff_estimate"]),
        "maximum_bending_stress_pa": float(payload["mechanical_summary"]["maximum_bending_stress_pa"]),
        "maximum_bow_resultant_magnitude_n": _max_value(mechanical_profile, "bow_resultant_magnitude_n"),
        "maximum_centralizer_centering_stiffness_n_per_m": _max_value(
            mechanical_profile,
            "centralizer_centering_stiffness_n_per_m",
        ),
        "maximum_centralizer_torque_increment_n_m": _max_value(
            payload["torque_profile"],
            "centralizer_torque_increment_n_m",
        ),
        "maximum_body_torque_increment_n_m": _max_value(
            payload["torque_profile"],
            "body_torque_increment_n_m",
        ),
        "maximum_centralizer_tangential_friction_magnitude_n": _max_value(
            payload.get("centralizer_tangential_friction_vector_profile", []),
            "tangential_friction_magnitude_n",
        ),
        "maximum_reference_support_stiffness_n_per_m": max(
            (
                float(item["equivalent_reference_support_stiffness_n_per_m"])
                for item in centralizer_parameters
            ),
            default=0.0,
        ),
        "maximum_force_per_bow_at_5mm_n": max(
            (float(item["force_per_bow_at_5mm_n"]) for item in centralizer_parameters),
            default=0.0,
        ),
        "maximum_force_per_bow_at_10mm_n": max(
            (float(item["force_per_bow_at_10mm_n"]) for item in centralizer_parameters),
            default=0.0,
        ),
        "coupling_iterations": float(payload["coupling_iterations"]),
        "coupling_final_max_profile_update_n": float(payload["coupling_final_max_profile_update_n"]),
        "coupling_final_max_torque_update_n_m": float(payload["coupling_final_max_torque_update_n_m"]),
        "global_solver_final_update_norm_m": float(
            payload["mechanical_summary"]["global_solver_final_update_norm_m"]
        ),
        "estimated_surface_torque_n_m": float(payload["estimated_surface_torque_n_m"] or 0.0),
        "estimated_body_surface_torque_n_m": float(
            torque_partition_summary.get("body_surface_torque_n_m", 0.0)
        ),
        "estimated_centralizer_surface_torque_n_m": float(
            torque_partition_summary.get("centralizer_surface_torque_n_m", 0.0)
        ),
        "torque_partition_residual_n_m": abs(
            float(torque_partition_summary.get("total_surface_torque_n_m", 0.0))
            - float(payload["estimated_surface_torque_n_m"] or 0.0)
        ),
    }


def _generic_case_checks(payload: Mapping[str, Any], derived_metrics: Mapping[str, float]) -> list[dict[str, Any]]:
    checks = [
        {
            "id": "sign-coherence",
            "passed": (
                float(payload["hookload_pull_out_n"]) >= float(payload["hookload_run_in_n"])
                and float(payload["drag_pull_out_n"]) >= 0.0
                and float(payload["drag_run_in_n"]) >= 0.0
                and float(payload["estimated_surface_torque_n_m"] or 0.0) >= 0.0
            ),
            "details": {
                "hookload_run_in_n": payload["hookload_run_in_n"],
                "hookload_pull_out_n": payload["hookload_pull_out_n"],
                "drag_run_in_n": payload["drag_run_in_n"],
                "drag_pull_out_n": payload["drag_pull_out_n"],
                "estimated_surface_torque_n_m": payload["estimated_surface_torque_n_m"],
            },
        },
        {
            "id": "standoff-range",
            "passed": 0.0 <= float(payload["mechanical_summary"]["minimum_standoff_estimate"]) <= 1.0,
            "details": {
                "minimum_standoff_estimate": payload["mechanical_summary"]["minimum_standoff_estimate"],
            },
        },
        {
            "id": "nonnegative-reactions",
            "passed": (
                float(payload["mechanical_summary"]["maximum_normal_reaction_estimate_n"]) >= 0.0
            ),
            "details": {
                "maximum_normal_reaction_estimate_n": payload["mechanical_summary"]["maximum_normal_reaction_estimate_n"],
            },
        },
        {
            "id": "iteration-reporting",
            "passed": (
                int(payload["mechanical_summary"]["global_solver_iteration_count"]) >= 1
                and int(payload["coupling_iterations"]) >= 1
            ),
            "details": {
                "global_solver_iteration_count": payload["mechanical_summary"]["global_solver_iteration_count"],
                "coupling_iterations": payload["coupling_iterations"],
            },
        },
        {
            "id": "torque-partition-balance",
            "passed": (
                abs(
                    float(payload["torque_partition_summary"]["total_surface_torque_n_m"])
                    - (
                        float(payload["torque_partition_summary"]["body_surface_torque_n_m"])
                        + float(payload["torque_partition_summary"]["centralizer_surface_torque_n_m"])
                    )
                )
                <= 1.0e-9
                and abs(
                    float(payload["updated_estimated_surface_torque_n_m"])
                    - float(payload["torque_partition_summary"]["total_surface_torque_n_m"])
                )
                <= 1.0e-9
            ),
            "details": dict(payload["torque_partition_summary"]),
        },
        {
            "id": "partition-profile-lengths",
            "passed": (
                len(payload["body_torque_profile"]) == len(payload["torque_profile"])
                and len(payload["body_axial_friction_profile"]) == len(payload["torque_profile"])
                and len(payload["centralizer_torque_profile"]) == len(payload["torque_profile"])
            ),
            "details": {
                "torque_profile": len(payload["torque_profile"]),
                "body_torque_profile": len(payload["body_torque_profile"]),
                "body_axial_friction_profile": len(payload["body_axial_friction_profile"]),
                "centralizer_torque_profile": len(payload["centralizer_torque_profile"]),
            },
        },
        {
            "id": "finite-derived-metrics",
            "passed": all(_is_finite_number(value) for value in derived_metrics.values()),
            "details": dict(derived_metrics),
        },
    ]
    return checks


def _bounds_checks(case_definition: Mapping[str, Any], case_payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for index, raw_bound in enumerate(case_definition.get("expected_bounds", [])):
        bound = _expect_mapping(raw_bound, f"cases[{case_definition['id']}].expected_bounds[{index}]")
        metric = _require_text(bound, "metric", "expected_bounds")
        actual_value = float(_nested_get(case_payload, metric))
        lower = _optional_float(bound, "lower", "expected_bounds")
        upper = _optional_float(bound, "upper", "expected_bounds")
        passed = True
        if lower is not None:
            passed = passed and actual_value >= lower
        if upper is not None:
            passed = passed and actual_value <= upper
        checks.append(
            {
                "id": f"bound-{case_definition['id']}-{index}",
                "passed": passed,
                "metric": metric,
                "actual_value": actual_value,
                "lower": lower,
                "upper": upper,
                "rationale": _optional_text(bound, "rationale", "expected_bounds"),
            }
        )
    return checks


def _evaluate_relation(left_value: float, right_value: float, relation: str, margin: float) -> bool:
    if relation == "gt":
        return right_value > left_value + margin
    if relation == "ge":
        return right_value >= left_value - margin
    if relation == "lt":
        return right_value < left_value - margin
    if relation == "le":
        return right_value <= left_value + margin
    raise ConfigError("benchmark validation relation must be one of: gt, ge, lt, le.")


def _evaluate_threshold(actual_value: float, threshold: float, relation: str) -> bool:
    if relation == "gt":
        return actual_value > threshold
    if relation == "ge":
        return actual_value >= threshold
    if relation == "lt":
        return actual_value < threshold
    if relation == "le":
        return actual_value <= threshold
    raise ConfigError("benchmark validation relation must be one of: gt, ge, lt, le.")


def _suite_validations(
    validations: Sequence[Any],
    case_results: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    evaluated: list[dict[str, Any]] = []
    for index, raw_validation in enumerate(validations):
        validation = _expect_mapping(raw_validation, f"validations[{index}]")
        validation_id = _require_text(validation, "id", "validation")
        metric = _require_text(validation, "metric", "validation")
        relation = _require_text(validation, "relation", "validation")
        rationale = _optional_text(validation, "rationale", "validation")
        margin = _optional_float(validation, "margin", "validation") or 0.0
        if "case" in validation:
            case_id = _require_text(validation, "case", "validation")
            if case_id not in case_results:
                raise ConfigError(f"Unknown benchmark case in validation: {case_id}")
            actual_value = float(_nested_get(case_results[case_id], metric))
            threshold = _optional_float(validation, "threshold", "validation")
            if threshold is None:
                raise ConfigError(f"validation[{validation_id}] requires threshold when 'case' is used.")
            passed = _evaluate_threshold(actual_value, threshold, relation)
            evaluated.append(
                {
                    "id": validation_id,
                    "type": "threshold",
                    "passed": passed,
                    "case": case_id,
                    "metric": metric,
                    "actual_value": actual_value,
                    "threshold": threshold,
                    "relation": relation,
                    "rationale": rationale,
                }
            )
            continue

        left_case = _require_text(validation, "left_case", "validation")
        right_case = _require_text(validation, "right_case", "validation")
        if left_case not in case_results or right_case not in case_results:
            raise ConfigError(f"Unknown benchmark cases in validation: {validation_id}")
        left_value = float(_nested_get(case_results[left_case], metric))
        right_value = float(_nested_get(case_results[right_case], metric))
        passed = _evaluate_relation(left_value, right_value, relation, margin)
        evaluated.append(
            {
                "id": validation_id,
                "type": "comparison",
                "passed": passed,
                "left_case": left_case,
                "right_case": right_case,
                "metric": metric,
                "left_value": left_value,
                "right_value": right_value,
                "relation": relation,
                "margin": margin,
                "rationale": rationale,
            }
        )
    return evaluated


def run_benchmark_suite(
    suite_path: str | Path,
    output_dir: str | Path | None = None,
) -> tuple[dict[str, Any], Path]:
    resolved_suite_path = Path(suite_path).resolve()
    raw_suite = _expect_mapping(read_yaml(resolved_suite_path), "benchmark suite")
    suite_name = _require_text(raw_suite, "name", "benchmark suite")
    suite_description = _optional_text(raw_suite, "description", "benchmark suite")
    cases_raw = _expect_sequence(raw_suite.get("cases"), "benchmark suite.cases")
    validations_raw = raw_suite.get("validations", [])
    if not isinstance(validations_raw, Sequence) or isinstance(validations_raw, (str, bytes, bytearray)):
        raise ConfigError("benchmark suite.validations must be a sequence when provided.")

    resolved_output_dir = _resolve_suite_output_dir(resolved_suite_path, output_dir)
    cases_output_dir = resolved_output_dir / "cases"
    cases_output_dir.mkdir(parents=True, exist_ok=True)

    cases_summary: list[dict[str, Any]] = []
    case_payloads_for_validation: dict[str, Mapping[str, Any]] = {}
    for index, raw_case in enumerate(cases_raw):
        case_definition = _expect_mapping(raw_case, f"cases[{index}]")
        case_id = _require_text(case_definition, "id", "benchmark case")
        case_reference = _require_text(case_definition, "case", "benchmark case")
        case_path = (resolved_suite_path.parent / case_reference).resolve()
        if not case_path.exists():
            raise ConfigError(f"Benchmark case file does not exist: {case_path}")
        _, payload, output_path = run_stub_case(case_path, cases_output_dir / f"{case_id}.json")
        derived_metrics = _derived_metrics(payload)
        case_payloads_for_validation[case_id] = {
            **payload,
            "derived_metrics": derived_metrics,
        }
        generic_checks = _generic_case_checks(payload, derived_metrics)
        bounds_checks = _bounds_checks({"id": case_id, **case_definition}, case_payloads_for_validation[case_id])
        all_checks = generic_checks + bounds_checks
        cases_summary.append(
            {
                "id": case_id,
                "case_path": str(case_path),
                "purpose": _optional_text(case_definition, "purpose", "benchmark case"),
                "tags": list(case_definition.get("tags", [])) if isinstance(case_definition.get("tags", []), Sequence) else [],
                "backend": payload["backend"],
                "solver_status": payload["status"],
                "validation_status": payload["validation_status"],
                "output_json": str(output_path),
                "derived_metrics": derived_metrics,
                "checks": all_checks,
                "passed_check_count": sum(1 for check in all_checks if check["passed"]),
                "failed_check_count": sum(1 for check in all_checks if not check["passed"]),
            }
        )

    validation_results = _suite_validations(validations_raw, case_payloads_for_validation)
    payload = {
        "name": suite_name,
        "description": suite_description,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "suite_path": str(resolved_suite_path),
        "output_dir": str(resolved_output_dir),
        "cases": cases_summary,
        "validations": validation_results,
        "summary": {
            "case_count": len(cases_summary),
            "case_check_count": sum(len(case["checks"]) for case in cases_summary),
            "case_check_failures": sum(case["failed_check_count"] for case in cases_summary),
            "validation_count": len(validation_results),
            "validation_failures": sum(1 for result in validation_results if not result["passed"]),
            "all_passed": (
                all(case["failed_check_count"] == 0 for case in cases_summary)
                and all(result["passed"] for result in validation_results)
            ),
        },
    }
    summary_path = resolved_output_dir / "suite_summary.json"
    write_json(summary_path, payload)
    return payload, summary_path


def format_benchmark_summary(payload: Mapping[str, Any]) -> str:
    summary = payload["summary"]
    lines = [
        f"Benchmark suite: {payload['name']}",
        f"Cases [-]: {summary['case_count']}",
        f"Case checks [-]: {summary['case_check_count']}",
        f"Case check failures [-]: {summary['case_check_failures']}",
        f"Cross-case validations [-]: {summary['validation_count']}",
        f"Cross-case validation failures [-]: {summary['validation_failures']}",
        f"All passed: {summary['all_passed']}",
    ]
    failing_ids = [
        validation["id"]
        for validation in payload["validations"]
        if not validation["passed"]
    ]
    if failing_ids:
        lines.append(f"Failing validations: {', '.join(failing_ids)}")
    return "\n".join(lines)
