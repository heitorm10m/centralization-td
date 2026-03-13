from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
from typer.testing import CliRunner

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.cli import app  # noqa: E402


def test_summary_command_prints_case_overview() -> None:
    runner = CliRunner()
    case_path = ROOT / "examples" / "minimal_case.yaml"

    result = runner.invoke(app, ["summary", str(case_path)])

    assert result.exit_code == 0
    assert "Case: minimal-case" in result.stdout
    assert "Trajectory points: 5" in result.stdout
    assert "String sections: 3" in result.stdout
    assert "Fluid density [kg/m3]:" in result.stdout
    assert "Discretization step [m]:" in result.stdout
    assert "Coupling max iterations [-]:" in result.stdout
    assert "Frame method:" in result.stdout
    assert "Coupling torque tolerance [N.m]:" in result.stdout
    assert "Centralizer specs: 2" in result.stdout


def test_run_stub_writes_json_output(tmp_path: Path) -> None:
    runner = CliRunner()
    case_path = ROOT / "examples" / "minimal_case.yaml"
    output_path = tmp_path / "stub-output.json"

    result = runner.invoke(app, ["run-stub", str(case_path), "--output", str(output_path)])

    assert result.exit_code == 0
    assert output_path.exists()
    assert "Status: phase14-vector-local-tangential-torque-coupled-baseline" in result.stdout
    assert "Operation mode: run_in" in result.stdout
    assert "Global solver iterations [-]: " in result.stdout
    assert "Coupling status:" in result.stdout
    assert "Coupling converged:" in result.stdout
    assert "Coupling final max profile update [N]:" in result.stdout
    assert "Coupling final max torque update [N.m]:" in result.stdout
    assert "Coupling final max torsional-load update [N.m]:" in result.stdout
    assert "Torque feedback mode:" in result.stdout
    assert "Torsional feedback status:" in result.stdout
    assert "Max local tangential mobilization [-]:" in result.stdout
    assert "Max local body tangential-demand factor [-]:" in result.stdout
    assert "Max local centralizer tangential-demand factor [-]:" in result.stdout
    assert "Hookload run in [N]:" in result.stdout
    assert "Hookload pull out [N]:" in result.stdout
    assert "Max normal reaction estimate [N]:" in result.stdout
    assert (
        "Centralizer model status: phase14-detailed-bow-spring-local-tangential-vector-torque"
        in result.stdout
    )
    assert "Validation status: phase10-benchmark-calibration-infrastructure" in result.stdout
    assert "Surface torque [N.m]:" in result.stdout
    assert "Updated surface torque with reduced torsional state [N.m]:" in result.stdout
    assert "Body surface torque [N.m]:" in result.stdout
    assert "Centralizer surface torque [N.m]:" in result.stdout

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "phase14-vector-local-tangential-torque-coupled-baseline"
    assert payload["backend"] in {"cpp", "python-fallback"}
    assert payload["operation_mode"] == "run_in"
    assert payload["validation_status"] == "phase10-benchmark-calibration-infrastructure"
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["mechanical_summary"]["segment_count"] > 0
    assert payload["mechanical_summary"]["global_solver_iteration_count"] >= 1
    assert payload["coupling_iterations"] >= 1
    assert payload["coupling_final_max_profile_update_n"] >= 0.0
    assert payload["coupling_final_max_torque_update_n_m"] >= 0.0
    assert payload["coupling_final_max_torsional_load_update_n_m"] >= 0.0
    assert payload["torque_feedback_mode"] == (
        "reduced-unified-local-tangential-state-fed-by-carried-torsional-state-plus-centralizer-axial-tangential-budget-and-convergence"
    )
    assert payload["torsional_feedback_status"] in {
        "phase11-reduced-torsional-load-and-twist-state",
        "phase14-reduced-torsional-state-fed-into-unified-local-tangential-state",
    }
    assert payload["estimated_surface_torque_n_m"] is not None
    assert payload["updated_estimated_surface_torque_n_m"] == pytest.approx(
        payload["torsional_state_profile"][0]["reduced_torsional_load_n_m"]
    )
    assert (
        payload["centralizer_model_status"]
        == "phase14-detailed-bow-spring-local-tangential-vector-torque"
    )
    assert payload["torque_and_drag_status"] == "phase14-reduced-unified-local-tangential-torque-baseline"
    assert payload["torque_drag_status"] == "phase14-reduced-unified-local-tangential-torque-baseline"
    assert len(payload["global_eccentricity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_n_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_b_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_run_in_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_pull_out_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["body_axial_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["body_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_axial_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_tangential_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_tangential_friction_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_torque_breakdown_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["local_tangential_interaction_state"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["local_tangential_state"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["local_tangential_mobilization_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["local_body_tangential_interaction_state"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["local_centralizer_tangential_interaction_state"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["updated_body_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["updated_centralizer_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["reduced_torque_accumulation_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["torsional_state_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_axial_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_normal_reaction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert any(
        0.0 <= point["body_tangential_mobilization"] <= 1.0
        for point in payload["local_tangential_interaction_state"]
    )
    assert any(
        0.0 <= point["centralizer_tangential_mobilization"] <= 1.0
        for point in payload["local_tangential_interaction_state"]
    )
    assert len(payload["bow_resultant_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_resultant_magnitude_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert payload["mechanical_profile"][0]["contact_state"] in {
        "free",
        "support-contact",
        "bow-spring-contact",
        "pipe-body-contact",
        "hole-undefined",
    }


def test_benchmark_suite_command_writes_summary(tmp_path: Path) -> None:
    runner = CliRunner()
    suite_path = ROOT / "benchmarks" / "suites" / "phase10_validation.yaml"
    output_dir = tmp_path / "benchmark-suite"

    result = runner.invoke(app, ["benchmark-suite", str(suite_path), "--output-dir", str(output_dir)])

    assert result.exit_code == 0
    assert "Benchmark suite: phase10-validation-benchmark-suite" in result.stdout
    assert "All passed: True" in result.stdout
    assert (output_dir / "suite_summary.json").exists()


def test_calibrate_bow_spring_command_writes_json(tmp_path: Path) -> None:
    runner = CliRunner()
    config_path = ROOT / "benchmarks" / "calibration" / "force_deflection_pairs.yaml"
    output_path = tmp_path / "calibration.json"

    result = runner.invoke(
        app,
        ["calibrate-bow-spring", str(config_path), "--output", str(output_path)],
    )

    assert result.exit_code == 0
    assert "Calibration: benchmark-force-deflection-fit" in result.stdout
    assert "Status: calibrated" in result.stdout
    assert output_path.exists()
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "calibrated"
    assert payload["resolved_parameters"]["blade_power_law_p"] == pytest.approx(1.25)
