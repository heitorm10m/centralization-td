from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.io import load_case_bundle  # noqa: E402
from centraltd.models import ConfigError, StringConfigModel, WellModel  # noqa: E402
from centraltd.runner import run_stub_case  # noqa: E402


def test_example_case_parses_phase2_inputs() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    assert loaded_case.trajectory_summary().point_count == 5
    assert loaded_case.string_summary().section_count == 3
    assert loaded_case.centralizer_summary().spec_count == 2


def test_trajectory_validation_rejects_non_monotonic_md() -> None:
    with pytest.raises(ConfigError):
        WellModel.from_dict(
            {
                "name": "invalid-well",
                "trajectory": [
                    {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
                    {"measured_depth_m": 100.0, "inclination_rad": 0.1, "azimuth_rad": 0.0},
                    {"measured_depth_m": 90.0, "inclination_rad": 0.2, "azimuth_rad": 0.0},
                ],
            }
        )


def test_interpolation_for_vertical_well_is_straightforward() -> None:
    well = WellModel.from_dict(
        {
            "name": "simple-well",
            "trajectory": [
                {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
                {"measured_depth_m": 100.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            ],
        }
    )

    node = well.interpolate(50.0)

    assert node.tvd_m == pytest.approx(50.0)
    assert node.northing_m == pytest.approx(0.0)
    assert node.easting_m == pytest.approx(0.0)
    assert node.inclination_rad == pytest.approx(0.0)


def test_string_section_validation_rejects_invalid_geometry() -> None:
    with pytest.raises(ConfigError):
        StringConfigModel.from_dict(
            {
                "name": "invalid-string",
                "sections": [
                    {
                        "name": "bad-section",
                        "md_start_m": 0.0,
                        "md_end_m": 100.0,
                        "outer_diameter_m": 0.1778,
                        "inner_diameter_m": 0.2000,
                        "linear_weight_n_per_m": 700.0,
                        "young_modulus_pa": 2.07e11,
                        "shear_modulus_pa": 8.0e10,
                        "density_kg_per_m3": 7850.0,
                        "friction_coefficient": 0.25,
                    }
                ],
            }
        )


def test_phase2_baseline_payload_contains_geometry_summaries(tmp_path: Path) -> None:
    case_path = ROOT / "examples" / "minimal_case.yaml"
    loaded_case, payload, output_path = run_stub_case(case_path, tmp_path / "phase2-output.json")

    assert output_path.exists()
    assert loaded_case.definition.name == "minimal-case"
    assert payload["status"] == "phase2-baseline"
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["centralizer_summary"]["spec_count"] == 2
    assert len(payload["section_summaries"]) == 3
    assert len(payload["warnings"]) >= 3
    assert len(payload["todos"]) == 6

