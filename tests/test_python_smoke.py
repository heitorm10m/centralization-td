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
    assert "Centralizer specs: 2" in result.stdout


def test_run_stub_writes_json_output(tmp_path: Path) -> None:
    runner = CliRunner()
    case_path = ROOT / "examples" / "minimal_case.yaml"
    output_path = tmp_path / "stub-output.json"

    result = runner.invoke(app, ["run-stub", str(case_path), "--output", str(output_path)])

    assert result.exit_code == 0
    assert output_path.exists()
    assert "Status: phase5-global-stiff-string-baseline" in result.stdout
    assert "Global solver iterations [-]:" in result.stdout
    assert "Top effective axial load [N]:" in result.stdout
    assert "Max normal reaction estimate [N]:" in result.stdout
    assert "Surface torque [N.m]: not implemented yet" in result.stdout

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "phase5-global-stiff-string-baseline"
    assert payload["backend"] in {"cpp", "python-fallback"}
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["mechanical_summary"]["segment_count"] > 0
    assert payload["mechanical_summary"]["global_solver_iteration_count"] >= 1
    assert payload["estimated_surface_torque_n_m"] is None
    assert payload["torque_and_drag_status"] == "not-implemented-yet"
    assert len(payload["global_eccentricity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert payload["mechanical_profile"][0]["contact_state"] in {
        "free",
        "support-contact",
        "pipe-body-contact",
        "hole-undefined",
    }
