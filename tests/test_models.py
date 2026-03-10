from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.io import load_case_bundle  # noqa: E402
from centraltd.mechanics import (  # noqa: E402
    GlobalContactStateModel,
    assemble_global_linear_system,
    build_global_node_inputs,
    discretize_case,
    run_mechanical_baseline,
)
from centraltd.models import (  # noqa: E402
    CaseDefinition,
    CentralizerConfigModel,
    ConfigError,
    LoadedCase,
    StringConfigModel,
    WellModel,
)
from centraltd.runner import run_stub_case  # noqa: E402


def _build_loaded_case(
    trajectory: list[dict[str, float]],
    *,
    centralizers: list[dict[str, object]] | None = None,
) -> LoadedCase:
    return LoadedCase(
        case_path=ROOT / "examples" / "synthetic_case.yaml",
        definition=CaseDefinition(
            name="synthetic-case",
            well="well.yaml",
            string="string.yaml",
            centralizers="centralizers.yaml",
            discretization_step_m=25.0,
        ),
        well_path=ROOT / "examples" / "well.yaml",
        string_path=ROOT / "examples" / "string.yaml",
        centralizers_path=ROOT / "examples" / "centralizers.yaml",
        well=WellModel.from_dict(
            {
                "name": "synthetic-well",
                "hole_diameter_m": 0.216,
                "fluid_density_kg_per_m3": 1100.0,
                "trajectory": trajectory,
            }
        ),
        string=StringConfigModel.from_dict(
            {
                "name": "synthetic-string",
                "sections": [
                    {
                        "name": "section-a",
                        "md_start_m": 0.0,
                        "md_end_m": 100.0,
                        "outer_diameter_m": 0.1778,
                        "inner_diameter_m": 0.1524,
                        "linear_weight_n_per_m": 700.0,
                        "young_modulus_pa": 2.07e11,
                        "shear_modulus_pa": 8.0e10,
                        "density_kg_per_m3": 7850.0,
                        "friction_coefficient": 0.25,
                    }
                ],
            }
        ),
        centralizers=CentralizerConfigModel.from_dict(
            {
                "name": "synthetic-centralizers",
                "centralizers": [] if centralizers is None else centralizers,
            }
        ),
    )


def test_example_case_parses_phase5_inputs() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    assert loaded_case.trajectory_summary().point_count == 5
    assert loaded_case.string_summary().section_count == 3
    assert loaded_case.centralizer_summary().spec_count == 2
    assert loaded_case.fluid_density_kg_per_m3 == pytest.approx(1100.0)
    assert loaded_case.discretization_step_m == pytest.approx(40.0)
    assert loaded_case.global_solver_max_iterations == 8
    assert loaded_case.contact_penalty_scale == pytest.approx(25.0)


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


def test_discretization_respects_requested_step() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    segments, placements = discretize_case(loaded_case)

    assert len(segments) > 0
    assert placements
    assert all(segment.segment_length_m <= loaded_case.discretization_step_m for segment in segments)
    assert segments[0].measured_depth_start_m == pytest.approx(0.0)
    assert segments[-1].measured_depth_end_m == pytest.approx(1950.0)


def test_global_assembly_builds_consistent_square_system() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")
    _, _, nodes = build_global_node_inputs(loaded_case)

    stiffness_matrix, load_vector = assemble_global_linear_system(
        nodes,
        [GlobalContactStateModel() for _ in nodes],
    )

    assert len(stiffness_matrix) == len(nodes)
    assert all(len(row) == len(nodes) for row in stiffness_matrix)
    assert len(load_vector) == len(nodes)
    assert stiffness_matrix[0][0] == pytest.approx(1.0)
    assert stiffness_matrix[-1][-1] == pytest.approx(1.0)


def test_vertical_case_has_no_contact_or_lateral_reaction() -> None:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
        ]
    )

    summary, profile, _ = run_mechanical_baseline(loaded_case)

    assert summary.segment_count == len(profile)
    assert summary.global_solver_iteration_count >= 1
    assert summary.top_effective_axial_load_n > 0.0
    assert summary.maximum_bending_stress_pa == pytest.approx(0.0)
    assert summary.maximum_normal_reaction_estimate_n == pytest.approx(0.0)
    assert summary.minimum_standoff_estimate == pytest.approx(1.0)
    assert summary.contact_segment_count == 0
    assert all(segment.contact_state == "free" for segment in profile)


def test_higher_curvature_increases_eccentricity_and_contact() -> None:
    low_curvature_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.08, "azimuth_rad": 0.0},
        ]
    )
    high_curvature_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ]
    )

    low_summary, _, _ = run_mechanical_baseline(low_curvature_case)
    high_summary, _, _ = run_mechanical_baseline(high_curvature_case)

    assert high_summary.maximum_bending_stress_pa > low_summary.maximum_bending_stress_pa
    assert high_summary.maximum_eccentricity_estimate_m > low_summary.maximum_eccentricity_estimate_m
    assert high_summary.maximum_normal_reaction_estimate_n > low_summary.maximum_normal_reaction_estimate_n


def test_centralizer_reduces_local_eccentricity_and_improves_standoff() -> None:
    trajectory = [
        {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
        {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
        {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
    ]
    base_case = _build_loaded_case(trajectory)
    centralized_case = _build_loaded_case(
        trajectory,
        centralizers=[
            {
                "name": "test-centralizer",
                "type": "bow-spring",
                "outer_diameter_m": 0.210,
                "nominal_restoring_force_n": 2400.0,
                "nominal_running_force_n": 1000.0,
                "spacing_hint_m": 20.0,
                "installation_md_m": [50.0],
            }
        ],
    )

    _, base_profile, _ = run_mechanical_baseline(base_case)
    _, centralized_profile, _ = run_mechanical_baseline(centralized_case)
    base_mid_segment = min(base_profile, key=lambda segment: abs(segment.measured_depth_center_m - 50.0))
    centralized_mid_segment = min(
        centralized_profile,
        key=lambda segment: abs(segment.measured_depth_center_m - 50.0),
    )

    assert centralized_mid_segment.eccentricity_estimate_m <= base_mid_segment.eccentricity_estimate_m
    assert centralized_mid_segment.standoff_estimate >= base_mid_segment.standoff_estimate


def test_standoff_estimate_stays_within_physical_limits() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, profile, _ = run_mechanical_baseline(loaded_case)

    assert all(0.0 <= segment.standoff_estimate <= 1.0 for segment in profile)


def test_normal_reaction_is_non_negative_when_contact_occurs() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, profile, _ = run_mechanical_baseline(loaded_case)

    assert all(segment.normal_reaction_estimate_n >= 0.0 for segment in profile)
    assert all(segment.support_normal_reaction_estimate_n >= 0.0 for segment in profile)
    assert all(segment.body_normal_reaction_estimate_n >= 0.0 for segment in profile)


def test_phase5_baseline_payload_contains_global_contact_outputs(tmp_path: Path) -> None:
    case_path = ROOT / "examples" / "minimal_case.yaml"
    loaded_case, payload, output_path = run_stub_case(case_path, tmp_path / "phase5-output.json")

    assert output_path.exists()
    assert loaded_case.definition.name == "minimal-case"
    assert payload["status"] == "phase5-global-stiff-string-baseline"
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["centralizer_summary"]["spec_count"] == 2
    assert payload["mechanical_summary"]["segment_count"] > 0
    assert payload["mechanical_summary"]["global_solver_iteration_count"] >= 1
    assert len(payload["mechanical_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["global_eccentricity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["standoff_estimate_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["contact_state_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["normal_reaction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bending_strain_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bending_moment_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_load_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert payload["estimated_surface_torque_n_m"] is None
    assert payload["torque_and_drag_status"] == "not-implemented-yet"
    assert payload["mechanical_summary"]["maximum_normal_reaction_estimate_n"] >= 0.0
    assert len(payload["warnings"]) >= 6
    assert len(payload["todos"]) == 6
