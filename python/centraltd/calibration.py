from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from pathlib import Path
from typing import Any

from .io import read_yaml, write_json
from .models import ConfigError

try:
    from . import _core as cpp_core  # type: ignore[attr-defined]
except ImportError:
    cpp_core = None


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


def _require_float(raw: Mapping[str, Any], key: str, context: str) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be numeric.")
    return float(value)


def _optional_float(raw: Mapping[str, Any], key: str, context: str) -> float | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be numeric when provided.")
    return float(value)


def _core_supports_calibration() -> bool:
    if cpp_core is None:
        return False
    try:
        point = cpp_core.BowSpringCalibrationPoint()
        result = cpp_core.BowSpringCalibrationResult()
    except Exception:
        return False
    return all(
        hasattr(instance, attribute)
        for instance, attribute in (
            (point, "deflection_m"),
            (point, "force_n"),
            (result, "blade_power_law_k"),
            (result, "evaluation_points"),
        )
    )


def _python_calibrate_power_law(
    points: list[dict[str, float]],
    fixed_blade_power_law_p: float | None,
) -> dict[str, Any]:
    if not points:
        return {
            "status": "invalid_input",
            "blade_power_law_k": 0.0,
            "blade_power_law_p": 1.0,
            "fitted_parameter_count": 0,
            "warnings": ["At least one deflection-force point is required for calibration."],
        }
    for point in points:
        if point["deflection_m"] <= 0.0:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": 1.0,
                "fitted_parameter_count": 0,
                "warnings": ["Calibration deflections must be strictly positive."],
            }
        if point["force_n"] <= 0.0:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": 1.0,
                "fitted_parameter_count": 0,
                "warnings": ["Calibration forces must be strictly positive."],
            }

    warnings: list[str] = []
    if fixed_blade_power_law_p is not None:
        if fixed_blade_power_law_p <= 0.0:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": 1.0,
                "fitted_parameter_count": 0,
                "warnings": ["The fixed blade power-law exponent must be positive."],
            }
        resolved_p = fixed_blade_power_law_p
        numerator = 0.0
        denominator = 0.0
        for point in points:
            transformed_deflection = point["deflection_m"] ** resolved_p
            numerator += point["force_n"] * transformed_deflection
            denominator += transformed_deflection * transformed_deflection
        if denominator <= 1.0e-12:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": resolved_p,
                "fitted_parameter_count": 0,
                "warnings": ["Fixed-exponent calibration encountered a near-singular fit."],
            }
        resolved_k = numerator / denominator
        fitted_parameter_count = 1
        status = "calibrated"
    elif len(points) == 1:
        resolved_p = 1.0
        resolved_k = points[0]["force_n"] / points[0]["deflection_m"]
        fitted_parameter_count = 1
        status = "calibrated_with_assumed_p"
        warnings.append("Only one calibration point was supplied, so p was assumed to be 1.0.")
    else:
        x_values = [math.log(point["deflection_m"]) for point in points]
        y_values = [math.log(point["force_n"]) for point in points]
        x_mean = sum(x_values) / len(x_values)
        y_mean = sum(y_values) / len(y_values)
        x_variance = sum((value - x_mean) ** 2 for value in x_values)
        if x_variance <= 1.0e-12:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": 1.0,
                "fitted_parameter_count": 0,
                "warnings": [
                    "Calibration deflections must contain at least two distinct positive values."
                ],
            }
        covariance = sum(
            (x_value - x_mean) * (y_value - y_mean)
            for x_value, y_value in zip(x_values, y_values, strict=True)
        )
        resolved_p = covariance / x_variance
        resolved_k = math.exp(y_mean - (resolved_p * x_mean))
        fitted_parameter_count = 2
        status = "calibrated"

    evaluation_points: list[dict[str, float]] = []
    squared_error_sum_n2 = 0.0
    absolute_error_sum_n = 0.0
    absolute_relative_error_sum = 0.0
    for point in points:
        predicted_force_n = resolved_k * (point["deflection_m"] ** resolved_p)
        absolute_error_n = abs(predicted_force_n - point["force_n"])
        relative_error = 0.0 if point["force_n"] <= 1.0e-12 else absolute_error_n / point["force_n"]
        squared_error_sum_n2 += absolute_error_n * absolute_error_n
        absolute_error_sum_n += absolute_error_n
        absolute_relative_error_sum += relative_error
        evaluation_points.append(
            {
                "deflection_m": point["deflection_m"],
                "measured_force_n": point["force_n"],
                "predicted_force_n": predicted_force_n,
                "absolute_error_n": absolute_error_n,
                "relative_error": relative_error,
            }
        )

    point_count = len(points)
    return {
        "status": status,
        "blade_power_law_k": resolved_k,
        "blade_power_law_p": resolved_p,
        "point_count": point_count,
        "fitted_parameter_count": fitted_parameter_count,
        "rmse_force_n": math.sqrt(squared_error_sum_n2 / point_count),
        "mean_absolute_error_n": absolute_error_sum_n / point_count,
        "mean_absolute_relative_error": absolute_relative_error_sum / point_count,
        "evaluation_points": evaluation_points,
        "warnings": warnings,
    }


def _cpp_points(points: list[dict[str, float]]) -> list[Any]:
    cpp_points: list[Any] = []
    for point in points:
        cpp_point = cpp_core.BowSpringCalibrationPoint()
        cpp_point.deflection_m = point["deflection_m"]
        cpp_point.force_n = point["force_n"]
        cpp_points.append(cpp_point)
    return cpp_points


def _calibration_result_to_dict(result: Any) -> dict[str, Any]:
    return {
        "status": result["status"] if isinstance(result, dict) else result.status,
        "blade_power_law_k": (
            result["blade_power_law_k"] if isinstance(result, dict) else result.blade_power_law_k
        ),
        "blade_power_law_p": (
            result["blade_power_law_p"] if isinstance(result, dict) else result.blade_power_law_p
        ),
        "point_count": result.get("point_count", 0) if isinstance(result, dict) else result.point_count,
        "fitted_parameter_count": (
            result.get("fitted_parameter_count", 0)
            if isinstance(result, dict)
            else result.fitted_parameter_count
        ),
        "rmse_force_n": result.get("rmse_force_n", 0.0) if isinstance(result, dict) else result.rmse_force_n,
        "mean_absolute_error_n": (
            result.get("mean_absolute_error_n", 0.0)
            if isinstance(result, dict)
            else result.mean_absolute_error_n
        ),
        "mean_absolute_relative_error": (
            result.get("mean_absolute_relative_error", 0.0)
            if isinstance(result, dict)
            else result.mean_absolute_relative_error
        ),
        "evaluation_points": (
            result.get("evaluation_points", [])
            if isinstance(result, dict)
            else [
                {
                    "deflection_m": point.deflection_m,
                    "measured_force_n": point.measured_force_n,
                    "predicted_force_n": point.predicted_force_n,
                    "absolute_error_n": point.absolute_error_n,
                    "relative_error": point.relative_error,
                }
                for point in result.evaluation_points
            ]
        ),
        "warnings": list(result.get("warnings", [])) if isinstance(result, dict) else list(result.warnings),
    }


def _run_pair_calibration(
    points: list[dict[str, float]],
    fixed_blade_power_law_p: float | None,
) -> tuple[str, dict[str, Any]]:
    if _core_supports_calibration():
        result = cpp_core.calibrate_bow_spring_power_law(
            _cpp_points(points),
            fixed_blade_power_law_p,
        )
        return "cpp", _calibration_result_to_dict(result)
    return "python-fallback", _calibration_result_to_dict(
        _python_calibrate_power_law(points, fixed_blade_power_law_p)
    )


def _run_nominal_calibration(
    nominal_restoring_force_n: float,
    restoring_deflection_m: float,
    nominal_running_force_n: float | None,
    running_deflection_m: float | None,
    fixed_blade_power_law_p: float | None,
) -> tuple[str, dict[str, Any], list[dict[str, float]]]:
    points = [{"deflection_m": restoring_deflection_m, "force_n": nominal_restoring_force_n}]
    if nominal_running_force_n is not None:
        if running_deflection_m is None:
            raise ConfigError("running_deflection_m must be provided when nominal_running_force_n is used.")
        points.append({"deflection_m": running_deflection_m, "force_n": nominal_running_force_n})
    if _core_supports_calibration():
        result = cpp_core.calibrate_bow_spring_from_nominal_points(
            nominal_restoring_force_n,
            restoring_deflection_m,
            nominal_running_force_n,
            running_deflection_m,
            fixed_blade_power_law_p,
        )
        return "cpp", _calibration_result_to_dict(result), points
    result = _python_calibrate_power_law(points, fixed_blade_power_law_p)
    if nominal_running_force_n is not None:
        result["warnings"].append(
            "Running-force calibration is treated as a reduced proxy point on the same power-law curve."
        )
    elif fixed_blade_power_law_p is None:
        result["warnings"].append(
            "Only a restoring-force point was supplied, so the nominal-point fit remains underdetermined without an assumed p."
        )
    return "python-fallback", _calibration_result_to_dict(result), points


def _resolve_output_path(config_path: Path, output: str | Path | None) -> Path:
    if output is not None:
        return Path(output).resolve()
    return config_path.with_suffix(".calibration.json")


def run_bow_spring_calibration(
    config_path: str | Path,
    output: str | Path | None = None,
) -> tuple[dict[str, Any], Path]:
    resolved_config_path = Path(config_path).resolve()
    raw = _expect_mapping(read_yaml(resolved_config_path), "calibration file")
    name = _require_text(raw, "name", "calibration")
    mode = _require_text(raw, "mode", "calibration")
    fixed_blade_power_law_p = _optional_float(raw, "fixed_blade_power_law_p", "calibration")
    notes = raw.get("notes")
    if notes is not None and not isinstance(notes, str):
        raise ConfigError("calibration.notes must be a string when provided.")

    backend: str
    result: dict[str, Any]
    input_summary: dict[str, Any]

    if mode == "force_deflection_pairs":
        points_raw = _expect_sequence(raw.get("force_deflection_pairs"), "calibration.force_deflection_pairs")
        points = [
            {
                "deflection_m": _require_float(_expect_mapping(item, f"force_deflection_pairs[{index}]"), "deflection_m", f"force_deflection_pairs[{index}]"),
                "force_n": _require_float(_expect_mapping(item, f"force_deflection_pairs[{index}]"), "force_n", f"force_deflection_pairs[{index}]"),
            }
            for index, item in enumerate(points_raw)
        ]
        backend, result = _run_pair_calibration(points, fixed_blade_power_law_p)
        input_summary = {
            "fixed_blade_power_law_p": fixed_blade_power_law_p,
            "force_deflection_pairs": points,
        }
    elif mode == "nominal_force_points":
        nominal_restoring_force_n = _require_float(raw, "nominal_restoring_force_n", "calibration")
        restoring_deflection_m = _require_float(raw, "restoring_deflection_m", "calibration")
        nominal_running_force_n = _optional_float(raw, "nominal_running_force_n", "calibration")
        running_deflection_m = _optional_float(raw, "running_deflection_m", "calibration")
        backend, result, points = _run_nominal_calibration(
            nominal_restoring_force_n,
            restoring_deflection_m,
            nominal_running_force_n,
            running_deflection_m,
            fixed_blade_power_law_p,
        )
        input_summary = {
            "fixed_blade_power_law_p": fixed_blade_power_law_p,
            "nominal_restoring_force_n": nominal_restoring_force_n,
            "restoring_deflection_m": restoring_deflection_m,
            "nominal_running_force_n": nominal_running_force_n,
            "running_deflection_m": running_deflection_m,
            "resolved_force_points": points,
        }
    else:
        raise ConfigError("calibration.mode must be 'force_deflection_pairs' or 'nominal_force_points'.")

    payload = {
        "name": name,
        "model": "bow-spring-power-law",
        "calibration_mode": mode,
        "backend": backend,
        "status": result["status"],
        "input": input_summary,
        "resolved_parameters": {
            "blade_power_law_k_n_per_m_pow_p": result["blade_power_law_k"],
            "blade_power_law_p": result["blade_power_law_p"],
        },
        "fit_quality": {
            "point_count": result["point_count"],
            "fitted_parameter_count": result["fitted_parameter_count"],
            "rmse_force_n": result["rmse_force_n"],
            "mean_absolute_error_n": result["mean_absolute_error_n"],
            "mean_absolute_relative_error": result["mean_absolute_relative_error"],
        },
        "evaluation_points": result["evaluation_points"],
        "warnings": result["warnings"],
        "notes": notes,
        "yaml_snippet": {
            "blade_power_law_k": result["blade_power_law_k"],
            "blade_power_law_p": result["blade_power_law_p"],
        },
    }

    output_path = _resolve_output_path(resolved_config_path, output)
    write_json(output_path, payload)
    return payload, output_path


def format_bow_spring_calibration_summary(payload: Mapping[str, Any]) -> str:
    resolved_parameters = payload["resolved_parameters"]
    fit_quality = payload["fit_quality"]
    lines = [
        f"Calibration: {payload['name']}",
        f"Mode: {payload['calibration_mode']}",
        f"Backend: {payload['backend']}",
        f"Status: {payload['status']}",
        "Resolved blade power-law k [N/m^p]: "
        f"{resolved_parameters['blade_power_law_k_n_per_m_pow_p']:.6f}",
        f"Resolved blade power-law p [-]: {resolved_parameters['blade_power_law_p']:.6f}",
        f"Calibration points [-]: {fit_quality['point_count']}",
        f"Fitted parameters [-]: {fit_quality['fitted_parameter_count']}",
        f"RMSE force [N]: {fit_quality['rmse_force_n']:.6f}",
        f"Mean absolute error [N]: {fit_quality['mean_absolute_error_n']:.6f}",
        "Mean absolute relative error [-]: "
        f"{fit_quality['mean_absolute_relative_error']:.6f}",
    ]
    if payload["warnings"]:
        lines.append(f"Warnings: {len(payload['warnings'])}")
    return "\n".join(lines)
