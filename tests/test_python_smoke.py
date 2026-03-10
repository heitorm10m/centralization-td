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
    assert "Approx vertical depth [m]:" in result.stdout
    assert "Centralizer specs: 2" in result.stdout


def test_run_stub_writes_json_output(tmp_path: Path) -> None:
    runner = CliRunner()
    case_path = ROOT / "examples" / "minimal_case.yaml"
    output_path = tmp_path / "stub-output.json"

    result = runner.invoke(app, ["run-stub", str(case_path), "--output", str(output_path)])

    assert result.exit_code == 0
    assert output_path.exists()

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "phase2-baseline"
    assert payload["backend"] in {"cpp", "python-fallback"}
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert "warnings" in payload
    assert len(payload["todos"]) == 6
