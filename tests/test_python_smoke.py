from __future__ import annotations

import json
import sys
from pathlib import Path

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
    assert "Centralizer specs: 2" in result.stdout


def test_run_stub_writes_json_output(tmp_path: Path) -> None:
    runner = CliRunner()
    case_path = ROOT / "examples" / "minimal_case.yaml"
    output_path = tmp_path / "stub-output.json"

    result = runner.invoke(app, ["run-stub", str(case_path), "--output", str(output_path)])

    assert result.exit_code == 0
    assert output_path.exists()
    assert "Status: phase9-vector-bow-spring-td-baseline" in result.stdout
    assert "Operation mode: run_in" in result.stdout
    assert "Global solver iterations [-]:" in result.stdout
    assert "Coupling status:" in result.stdout
    assert "Coupling converged:" in result.stdout
    assert "Hookload run in [N]:" in result.stdout
    assert "Hookload pull out [N]:" in result.stdout
    assert "Max normal reaction estimate [N]:" in result.stdout
    assert "Centralizer model status: phase9-detailed-bow-spring" in result.stdout
    assert "Surface torque [N.m]:" in result.stdout
    assert "Updated surface torque with detailed centralizers [N.m]:" in result.stdout

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "phase9-vector-bow-spring-td-baseline"
    assert payload["backend"] in {"cpp", "python-fallback"}
    assert payload["operation_mode"] == "run_in"
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["mechanical_summary"]["segment_count"] > 0
    assert payload["mechanical_summary"]["global_solver_iteration_count"] >= 1
    assert payload["coupling_iterations"] >= 1
    assert payload["estimated_surface_torque_n_m"] is not None
    assert payload["updated_estimated_surface_torque_n_m"] == payload["estimated_surface_torque_n_m"]
    assert payload["centralizer_model_status"] == "phase9-detailed-bow-spring"
    assert payload["torque_and_drag_status"] == "phase9-reduced-bow-spring-td-baseline"
    assert payload["torque_drag_status"] == "phase9-reduced-bow-spring-td-baseline"
    assert len(payload["global_eccentricity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_n_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_b_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_run_in_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_pull_out_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_axial_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_tangential_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_axial_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_normal_reaction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_resultant_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_resultant_magnitude_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert payload["mechanical_profile"][0]["contact_state"] in {
        "free",
        "support-contact",
        "bow-spring-contact",
        "pipe-body-contact",
        "hole-undefined",
    }
