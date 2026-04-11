from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from ..io.io import read_yaml, write_json
from ..models import ConfigError


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


def _numpy_calibrate_power_law(
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

    deflections_m = np.asarray([point["deflection_m"] for point in points], dtype=float)
    forces_n = np.asarray([point["force_n"] for point in points], dtype=float)

    if bool(np.any(deflections_m <= 0.0)):
        return {
            "status": "invalid_input",
            "blade_power_law_k": 0.0,
            "blade_power_law_p": 1.0,
            "fitted_parameter_count": 0,
            "warnings": ["Calibration deflections must be strictly positive."],
        }
    if bool(np.any(forces_n <= 0.0)):
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
        resolved_p = float(fixed_blade_power_law_p)
        transformed_deflection = np.power(deflections_m, resolved_p)
        denominator = float(np.dot(transformed_deflection, transformed_deflection))
        if denominator <= 1.0e-12:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": resolved_p,
                "fitted_parameter_count": 0,
                "warnings": ["Fixed-exponent calibration encountered a near-singular fit."],
            }
        resolved_k = float(np.dot(forces_n, transformed_deflection) / denominator)
        fitted_parameter_count = 1
        status = "calibrated"
    elif len(points) == 1:
        resolved_p = 1.0
        resolved_k = float(forces_n[0] / deflections_m[0])
        fitted_parameter_count = 1
        status = "calibrated_with_assumed_p"
        warnings.append("Only one calibration point was supplied, so p was assumed to be 1.0.")
    else:
        x_values = np.log(deflections_m)
        y_values = np.log(forces_n)
        if float(np.var(x_values)) <= 1.0e-12:
            return {
                "status": "invalid_input",
                "blade_power_law_k": 0.0,
                "blade_power_law_p": 1.0,
                "fitted_parameter_count": 0,
                "warnings": [
                    "Calibration deflections must contain at least two distinct positive values."
                ],
            }
        design_matrix = np.column_stack((x_values, np.ones_like(x_values)))
        try:
            coefficients, *_ = np.linalg.lstsq(design_matrix, y_values, rcond=None)
        except np.linalg.LinAlgError as exc:
            raise ValueError("NumPy least-squares calibration failed.") from exc
        resolved_p = float(coefficients[0])
        resolved_k = float(np.exp(coefficients[1]))
        fitted_parameter_count = 2
        status = "calibrated"

    predicted_forces_n = resolved_k * np.power(deflections_m, resolved_p)
    absolute_errors_n = np.abs(predicted_forces_n - forces_n)
    relative_errors = np.divide(
        absolute_errors_n,
        forces_n,
        out=np.zeros_like(absolute_errors_n),
        where=forces_n > 1.0e-12,
    )
    evaluation_points = [
        {
            "deflection_m": float(deflection_m),
            "measured_force_n": float(measured_force_n),
            "predicted_force_n": float(predicted_force_n),
            "absolute_error_n": float(absolute_error_n),
            "relative_error": float(relative_error),
        }
        for deflection_m, measured_force_n, predicted_force_n, absolute_error_n, relative_error in zip(
            deflections_m,
            forces_n,
            predicted_forces_n,
            absolute_errors_n,
            relative_errors,
            strict=True,
        )
    ]

    return {
        "status": status,
        "blade_power_law_k": resolved_k,
        "blade_power_law_p": resolved_p,
        "point_count": len(points),
        "fitted_parameter_count": fitted_parameter_count,
        "rmse_force_n": float(np.sqrt(np.mean(np.square(absolute_errors_n)))),
        "mean_absolute_error_n": float(np.mean(absolute_errors_n)),
        "mean_absolute_relative_error": float(np.mean(relative_errors)),
        "evaluation_points": evaluation_points,
        "warnings": warnings,
    }


def _calibration_result_to_dict(result: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "status": result["status"],
        "blade_power_law_k": result["blade_power_law_k"],
        "blade_power_law_p": result["blade_power_law_p"],
        "point_count": result.get("point_count", 0),
        "fitted_parameter_count": result.get("fitted_parameter_count", 0),
        "rmse_force_n": result.get("rmse_force_n", 0.0),
        "mean_absolute_error_n": result.get("mean_absolute_error_n", 0.0),
        "mean_absolute_relative_error": result.get("mean_absolute_relative_error", 0.0),
        "evaluation_points": list(result.get("evaluation_points", [])),
        "warnings": list(result.get("warnings", [])),
    }


def _run_pair_calibration(
    points: list[dict[str, float]],
    fixed_blade_power_law_p: float | None,
) -> tuple[str, dict[str, Any]]:
    return "python-numpy", _calibration_result_to_dict(
        _numpy_calibrate_power_law(points, fixed_blade_power_law_p)
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
    result = _numpy_calibrate_power_law(points, fixed_blade_power_law_p)
    if nominal_running_force_n is not None:
        result["warnings"].append(
            "Running-force calibration is treated as a reduced proxy point on the same power-law curve."
        )
    elif fixed_blade_power_law_p is None:
        result["warnings"].append(
            "Only a restoring-force point was supplied, so the nominal-point fit remains underdetermined without an assumed p."
        )
    return "python-numpy", _calibration_result_to_dict(result), points


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
            "axial_friction_force_ratio_parameter": None,
            "tangential_torque_force_ratio_parameter": None,
        },
        "parameter_roles": {
            "blade_power_law_k_n_per_m_pow_p": "bow-restoring-force-law",
            "blade_power_law_p": "bow-restoring-force-law-exponent",
            "axial_friction_force_ratio_parameter": (
                "not calibrated here; currently derived in the solver from nominal_running_force_n / nominal_restoring_force_n"
            ),
            "tangential_torque_force_ratio_parameter": (
                "not calibrated here; currently derived in the solver from nominal_running_force_n / nominal_restoring_force_n"
            ),
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
