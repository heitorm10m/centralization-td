from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.io import load_case_bundle  # noqa: E402
from centraltd.benchmarking import run_benchmark_suite  # noqa: E402
from centraltd.calibration import run_bow_spring_calibration  # noqa: E402
from centraltd.coupling import run_coupled_global_baseline  # noqa: E402
from centraltd.bow_spring import (  # noqa: E402
    bow_contact_onset_clearance_m,
    bow_deflection_m,
    bow_force_magnitude_n,
    build_bow_directions,
    evaluate_bow_spring_segment_result,
)
from centraltd.torque_drag_centralizer import (  # noqa: E402
    evaluate_centralizer_torque_contribution,
)
from centraltd.frames import build_frame_nodes  # noqa: E402
from centraltd.mechanics import (  # noqa: E402
    GlobalContactStateModel,
    assemble_global_linear_system,
    build_global_node_inputs,
    compute_buoyant_axial_load_profile,
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
from centraltd.torque_drag import run_torque_drag_baseline  # noqa: E402


def _build_loaded_case(
    trajectory: list[dict[str, float]],
    *,
    centralizers: list[dict[str, object]] | None = None,
    operation_mode: str = "run_in",
    friction_coefficient: float = 0.25,
) -> LoadedCase:
    return LoadedCase(
        case_path=ROOT / "examples" / "synthetic_case.yaml",
        definition=CaseDefinition(
            name="synthetic-case",
            well="well.yaml",
            string="string.yaml",
            centralizers="centralizers.yaml",
            operation_mode=operation_mode,
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
                        "friction_coefficient": friction_coefficient,
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


def _build_bow_placement_case() -> LoadedCase:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ],
        centralizers=[
            {
                "name": "test-centralizer",
                "centralizer_type": "bow-spring",
                "support_outer_diameter_m": 0.214,
                "number_of_bows": 4,
                "angular_orientation_reference_deg": 15.0,
                "inner_clearance_to_pipe_m": 0.001,
                "nominal_restoring_force_n": 3200.0,
                "nominal_running_force_n": 1600.0,
                "blade_power_law_p": 1.5,
                "spacing_hint_m": 40.0,
                "installation_md_m": [50.0],
            }
        ],
    )
    loaded_case.definition.discretization_step_m = 10.0
    return loaded_case


def _select_mid_segment_and_placement(loaded_case: LoadedCase) -> tuple[object, object]:
    segments, placements = discretize_case(loaded_case)
    segment = min(segments, key=lambda item: abs(item.measured_depth_center_m - 50.0))
    placement = min(placements, key=lambda item: abs(item.measured_depth_m - 50.0))
    return segment, placement


def test_example_case_parses_phase9_inputs() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    assert loaded_case.trajectory_summary().point_count == 5
    assert loaded_case.string_summary().section_count == 3
    assert loaded_case.centralizer_summary().spec_count == 2
    assert loaded_case.fluid_density_kg_per_m3 == pytest.approx(1100.0)
    assert loaded_case.discretization_step_m == pytest.approx(40.0)
    assert loaded_case.global_solver_max_iterations == 8
    assert loaded_case.contact_penalty_scale == pytest.approx(25.0)
    assert loaded_case.coupling_max_iterations == 6
    assert loaded_case.coupling_tolerance_n == pytest.approx(25.0)
    assert loaded_case.relaxation_factor == pytest.approx(0.5)
    assert loaded_case.frame_method == "parallel-transport"
    assert loaded_case.centralizers.centralizers[0].number_of_bows >= 4
    assert loaded_case.centralizers.centralizers[0].inner_clearance_to_pipe_m >= 0.0
    assert loaded_case.centralizers.centralizers[0].effective_contact_diameter_m() > 0.0


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


def test_parallel_transport_frame_is_orthonormal_for_simple_trajectory() -> None:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ]
    )

    frame_nodes = build_frame_nodes(loaded_case)

    assert frame_nodes
    for frame_node in frame_nodes:
        tangent_norm = sum(component * component for component in frame_node.tangent_north_east_tvd)
        normal_norm = sum(component * component for component in frame_node.normal_north_east_tvd)
        binormal_norm = sum(component * component for component in frame_node.binormal_north_east_tvd)
        tangent_normal = sum(
            left * right
            for left, right in zip(
                frame_node.tangent_north_east_tvd,
                frame_node.normal_north_east_tvd,
                strict=True,
            )
        )
        tangent_binormal = sum(
            left * right
            for left, right in zip(
                frame_node.tangent_north_east_tvd,
                frame_node.binormal_north_east_tvd,
                strict=True,
            )
        )
        normal_binormal = sum(
            left * right
            for left, right in zip(
                frame_node.normal_north_east_tvd,
                frame_node.binormal_north_east_tvd,
                strict=True,
            )
        )
        assert tangent_norm == pytest.approx(1.0)
        assert normal_norm == pytest.approx(1.0)
        assert binormal_norm == pytest.approx(1.0)
        assert tangent_normal == pytest.approx(0.0, abs=1.0e-10)
        assert tangent_binormal == pytest.approx(0.0, abs=1.0e-10)
        assert normal_binormal == pytest.approx(0.0, abs=1.0e-10)


def test_bow_distribution_respects_reference_angle() -> None:
    loaded_case = _build_bow_placement_case()
    _, placement = _select_mid_segment_and_placement(loaded_case)

    directions = build_bow_directions(placement)

    assert len(directions) == 4
    assert directions[0][1] == pytest.approx(math.radians(15.0))
    assert directions[1][1] == pytest.approx(math.radians(105.0))
    assert directions[2][1] == pytest.approx(math.radians(195.0))
    assert directions[3][1] == pytest.approx(math.radians(285.0))


def test_bow_without_contact_generates_zero_force() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)
    direction_n_b = build_bow_directions(placement)[0][2]
    hole_radius_m = 0.5 * segment.reference_hole_diameter_m

    deflection_m = bow_deflection_m(
        placement,
        hole_radius_m,
        (0.0, 0.0),
        direction_n_b,
    )

    assert deflection_m == pytest.approx(0.0)
    assert bow_force_magnitude_n(placement, hole_radius_m, deflection_m) == pytest.approx(0.0)


def test_bow_force_increases_monotonically_with_deflection() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)
    hole_radius_m = 0.5 * segment.reference_hole_diameter_m

    force_low_n = bow_force_magnitude_n(placement, hole_radius_m, 0.002)
    force_high_n = bow_force_magnitude_n(placement, hole_radius_m, 0.006)

    assert force_low_n > 0.0
    assert force_high_n > force_low_n


def test_symmetric_bow_layout_gives_zero_resultant_without_eccentricity() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)

    result = evaluate_bow_spring_segment_result(segment, [placement], (0.0, 0.0))

    assert result.bow_resultant_magnitude_n == pytest.approx(0.0)
    assert result.centralizer_torque_increment_n_m == pytest.approx(0.0)


def test_eccentricity_breaks_bow_symmetry_and_generates_resultant() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)
    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    onset_m = bow_contact_onset_clearance_m(placement, hole_radius_m)

    result = evaluate_bow_spring_segment_result(segment, [placement], (onset_m + 0.01, 0.0))

    assert result.bow_resultant_magnitude_n > 0.0
    assert result.bow_resultant_vector_n_b[0] > 0.0


def test_centralizer_tangential_direction_is_orthogonal_to_bow_resultant() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)
    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    onset_m = bow_contact_onset_clearance_m(placement, hole_radius_m)

    bow_result = evaluate_bow_spring_segment_result(segment, [placement], (onset_m + 0.01, 0.0))
    torque_contribution = evaluate_centralizer_torque_contribution(bow_result)

    assert torque_contribution.tangential_friction_n > 0.0
    dot_product = (
        bow_result.bow_resultant_vector_n_b[0] * torque_contribution.tangential_direction_n_b[0]
        + bow_result.bow_resultant_vector_n_b[1] * torque_contribution.tangential_direction_n_b[1]
    )
    assert dot_product == pytest.approx(0.0, abs=1.0e-10)


def test_higher_bow_resultant_increases_centralizer_torque_proxy() -> None:
    loaded_case = _build_bow_placement_case()
    segment, placement = _select_mid_segment_and_placement(loaded_case)
    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    onset_m = bow_contact_onset_clearance_m(placement, hole_radius_m)

    low_bow_result = evaluate_bow_spring_segment_result(segment, [placement], (onset_m + 0.004, 0.0))
    high_bow_result = evaluate_bow_spring_segment_result(segment, [placement], (onset_m + 0.012, 0.0))
    low_torque = evaluate_centralizer_torque_contribution(low_bow_result)
    high_torque = evaluate_centralizer_torque_contribution(high_bow_result)

    assert high_bow_result.bow_resultant_magnitude_n > low_bow_result.bow_resultant_magnitude_n
    assert high_torque.torque_increment_n_m > low_torque.torque_increment_n_m


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

    assert len(stiffness_matrix) == (2 * len(nodes))
    assert all(len(row) == (2 * len(nodes)) for row in stiffness_matrix)
    assert len(load_vector) == (2 * len(nodes))
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
    assert all(abs(segment.lateral_displacement_normal_m) <= 1.0e-8 for segment in profile)
    assert all(abs(segment.lateral_displacement_binormal_m) <= 1.0e-8 for segment in profile)


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
    high_summary, high_profile, _ = run_mechanical_baseline(high_curvature_case)

    assert high_summary.maximum_bending_stress_pa > low_summary.maximum_bending_stress_pa
    assert high_summary.maximum_eccentricity_estimate_m > low_summary.maximum_eccentricity_estimate_m
    assert high_summary.maximum_normal_reaction_estimate_n > low_summary.maximum_normal_reaction_estimate_n
    assert any(abs(segment.eccentricity_binormal_m) > 0.0 for segment in high_profile)


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
                "number_of_bows": 4,
                "angular_orientation_reference_deg": 0.0,
                "inner_clearance_to_pipe_m": 0.002,
                "nominal_restoring_force_n": 2400.0,
                "nominal_running_force_n": 1000.0,
                "blade_power_law_p": 1.5,
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


def test_contact_direction_points_with_normal_reaction_vector() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, profile, _ = run_mechanical_baseline(loaded_case)

    for segment in profile:
        magnitude = math.sqrt(
            (segment.normal_reaction_normal_n ** 2) + (segment.normal_reaction_binormal_n ** 2)
        )
        assert magnitude == pytest.approx(segment.normal_reaction_estimate_n)
        if magnitude > 0.0:
            dot_product = (
                segment.contact_direction_normal * segment.normal_reaction_normal_n
                + segment.contact_direction_binormal * segment.normal_reaction_binormal_n
            )
            assert dot_product >= 0.0


def test_normal_reaction_is_non_negative_when_contact_occurs() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, profile, _ = run_mechanical_baseline(loaded_case)

    assert all(segment.normal_reaction_estimate_n >= 0.0 for segment in profile)
    assert all(segment.support_normal_reaction_estimate_n >= 0.0 for segment in profile)
    assert all(segment.body_normal_reaction_estimate_n >= 0.0 for segment in profile)


def test_normal_reaction_profile_is_dimensionally_consistent() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, profile, _ = run_mechanical_baseline(loaded_case)

    for segment in profile:
        assert segment.normal_reaction_estimate_n == pytest.approx(
            segment.normal_reaction_estimate_n_per_m * segment.segment_length_m
        )


def test_vertical_case_has_reduced_td_with_near_zero_drag_and_torque() -> None:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
        ]
    )

    mechanical_summary, mechanical_profile, _ = run_mechanical_baseline(loaded_case)
    segments, _ = discretize_case(loaded_case)
    _, buoyant_top_hookload_n = compute_buoyant_axial_load_profile(segments)
    torque_drag = run_torque_drag_baseline(
        loaded_case,
        mechanical_profile,
        reference_buoyant_hookload_n=buoyant_top_hookload_n,
    )

    assert torque_drag.hookload_run_in_n == pytest.approx(torque_drag.hookload_pull_out_n)
    assert torque_drag.hookload_run_in_n == pytest.approx(buoyant_top_hookload_n)
    assert torque_drag.estimated_surface_torque_n_m == pytest.approx(0.0)


def test_higher_normal_reaction_increases_drag_and_torque() -> None:
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

    low_summary, low_profile, _ = run_mechanical_baseline(low_curvature_case)
    high_summary, high_profile, _ = run_mechanical_baseline(high_curvature_case)
    low_segments, _ = discretize_case(low_curvature_case)
    _, low_buoyant_top_hookload_n = compute_buoyant_axial_load_profile(low_segments)
    high_segments, _ = discretize_case(high_curvature_case)
    _, high_buoyant_top_hookload_n = compute_buoyant_axial_load_profile(high_segments)
    low_td = run_torque_drag_baseline(
        low_curvature_case,
        low_profile,
        reference_buoyant_hookload_n=low_buoyant_top_hookload_n,
    )
    high_td = run_torque_drag_baseline(
        high_curvature_case,
        high_profile,
        reference_buoyant_hookload_n=high_buoyant_top_hookload_n,
    )

    assert high_td.drag_pull_out_n > low_td.drag_pull_out_n
    assert high_td.estimated_surface_torque_n_m > low_td.estimated_surface_torque_n_m


def test_detailed_centralizer_contribution_adds_torque_component() -> None:
    loaded_case = load_case_bundle(ROOT / "examples" / "minimal_case.yaml")

    _, mechanical_profile, _ = run_mechanical_baseline(loaded_case)
    segments, _ = discretize_case(loaded_case)
    _, buoyant_top_hookload_n = compute_buoyant_axial_load_profile(segments)
    torque_drag = run_torque_drag_baseline(
        loaded_case,
        mechanical_profile,
        reference_buoyant_hookload_n=buoyant_top_hookload_n,
    )

    assert max(point.centralizer_torque_increment_n_m for point in torque_drag.torque_profile) > 0.0
    assert max(point.cumulative_torque_n_m for point in torque_drag.centralizer_torque_profile) > 0.0
    assert torque_drag.torque_partition_summary.centralizer_surface_torque_n_m > 0.0
    assert torque_drag.torque_partition_summary.total_surface_torque_n_m == pytest.approx(
        torque_drag.torque_partition_summary.body_surface_torque_n_m
        + torque_drag.torque_partition_summary.centralizer_surface_torque_n_m
    )


def test_higher_friction_increases_hookload_differential() -> None:
    base_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ],
        friction_coefficient=0.15,
    )
    high_mu_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ],
        friction_coefficient=0.45,
    )

    base_summary, base_profile, _ = run_mechanical_baseline(base_case)
    high_mu_summary, high_mu_profile, _ = run_mechanical_baseline(high_mu_case)
    base_segments, _ = discretize_case(base_case)
    _, base_buoyant_top_hookload_n = compute_buoyant_axial_load_profile(base_segments)
    high_mu_segments, _ = discretize_case(high_mu_case)
    _, high_mu_buoyant_top_hookload_n = compute_buoyant_axial_load_profile(high_mu_segments)
    base_td = run_torque_drag_baseline(
        base_case,
        base_profile,
        reference_buoyant_hookload_n=base_buoyant_top_hookload_n,
    )
    high_mu_td = run_torque_drag_baseline(
        high_mu_case,
        high_mu_profile,
        reference_buoyant_hookload_n=high_mu_buoyant_top_hookload_n,
    )

    base_delta = base_td.hookload_pull_out_n - base_td.hookload_run_in_n
    high_mu_delta = high_mu_td.hookload_pull_out_n - high_mu_td.hookload_run_in_n
    assert high_mu_delta > base_delta


def test_coupled_pull_out_has_higher_curvature_lateral_load_than_run_in() -> None:
    trajectory = [
        {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
        {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
        {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
    ]
    run_in_case = _build_loaded_case(trajectory, operation_mode="run_in", friction_coefficient=0.40)
    pull_out_case = _build_loaded_case(trajectory, operation_mode="pull_out", friction_coefficient=0.40)

    run_in_result = run_coupled_global_baseline(run_in_case)
    pull_out_result = run_coupled_global_baseline(pull_out_case)

    assert pull_out_result.mechanical_summary.top_effective_axial_load_n > (
        run_in_result.mechanical_summary.top_effective_axial_load_n
    )
    assert max(
        segment.curvature_lateral_load_n_per_m for segment in pull_out_result.mechanical_profile
    ) > max(segment.curvature_lateral_load_n_per_m for segment in run_in_result.mechanical_profile)


def test_coupling_converges_for_vertical_case() -> None:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
        ]
    )

    result = run_coupled_global_baseline(loaded_case)

    assert result.converged is True
    assert result.status == "converged"
    assert result.iteration_count >= 1
    assert result.maximum_torque_update_n_m >= 0.0
    assert len(result.converged_axial_profile) == len(result.mechanical_profile)
    assert len(result.converged_normal_reaction_profile) == len(result.mechanical_profile)
    assert len(result.converged_torque_profile) == len(result.mechanical_profile)


def test_coupling_falls_back_cleanly_when_iteration_budget_is_too_small() -> None:
    loaded_case = _build_loaded_case(
        [
            {"measured_depth_m": 0.0, "inclination_rad": 0.0, "azimuth_rad": 0.0},
            {"measured_depth_m": 50.0, "inclination_rad": 0.25, "azimuth_rad": 0.0},
            {"measured_depth_m": 100.0, "inclination_rad": 0.50, "azimuth_rad": 0.4},
        ],
        friction_coefficient=0.40,
    )
    loaded_case.definition.coupling_max_iterations = 1
    loaded_case.definition.coupling_tolerance_n = 1.0e-12

    result = run_coupled_global_baseline(loaded_case)

    assert result.converged is False
    assert result.status == "max_iterations_reached"
    assert result.iteration_count == 1
    assert all(point.axial_force_n >= 0.0 for point in result.converged_axial_profile)


def test_run_in_and_pull_out_signs_are_coherent(tmp_path: Path) -> None:
    case_path = ROOT / "examples" / "minimal_case.yaml"

    _, payload, _ = run_stub_case(case_path, tmp_path / "mode-signs.json")

    assert payload["hookload_pull_out_n"] >= payload["hookload_run_in_n"]
    assert payload["drag_pull_out_n"] >= 0.0
    assert payload["drag_run_in_n"] >= 0.0
    assert payload["estimated_surface_torque_n_m"] >= 0.0


def test_phase11_payload_contains_torque_partition_outputs(tmp_path: Path) -> None:
    case_path = ROOT / "examples" / "minimal_case.yaml"
    loaded_case, payload, output_path = run_stub_case(case_path, tmp_path / "phase11-output.json")

    assert output_path.exists()
    assert loaded_case.definition.name == "minimal-case"
    assert payload["status"] == "phase11-vector-centralizer-torque-coupled-baseline"
    assert payload["operation_mode"] == "run_in"
    assert payload["trajectory_summary"]["point_count"] == 5
    assert payload["string_summary"]["section_count"] == 3
    assert payload["centralizer_summary"]["spec_count"] == 2
    assert payload["mechanical_summary"]["segment_count"] > 0
    assert payload["mechanical_summary"]["global_solver_iteration_count"] >= 1
    assert payload["coupling_iterations"] >= 1
    assert len(payload["mechanical_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["global_eccentricity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_n_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["lateral_displacement_b_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["eccentricity_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["eccentricity_magnitude_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["contact_direction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["standoff_estimate_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["contact_state_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["normal_reaction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["normal_reaction_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["normal_reaction_magnitude_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bending_strain_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bending_severity_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_force_vectors"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_force_magnitudes"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_resultant_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bow_resultant_magnitude_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["bending_moment_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_load_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_run_in_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["axial_force_pull_out_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["body_axial_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["body_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_axial_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_tangential_friction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_tangential_friction_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["tangential_friction_vector_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["centralizer_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_axial_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_normal_reaction_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert len(payload["converged_torque_profile"]) == payload["mechanical_summary"]["segment_count"]
    assert payload["estimated_surface_torque_n_m"] is not None
    assert payload["updated_estimated_surface_torque_n_m"] == pytest.approx(
        payload["torque_partition_summary"]["total_surface_torque_n_m"]
    )
    assert payload["validation_status"] == "phase10-benchmark-calibration-infrastructure"
    assert payload["centralizer_model_status"] == "phase11-detailed-bow-spring-vector-torque"
    assert payload["torque_and_drag_status"] == "phase11-reduced-vector-centralizer-torque-baseline"
    assert payload["torque_drag_status"] == "phase11-reduced-vector-centralizer-torque-baseline"
    assert payload["coupling_final_max_profile_update_n"] >= 0.0
    assert payload["coupling_final_max_torque_update_n_m"] >= 0.0
    assert payload["mechanical_summary"]["maximum_normal_reaction_estimate_n"] >= 0.0
    assert any(segment["bow_force_details"] for segment in payload["mechanical_profile"])
    assert any(
        point["centralizer_torque_increment_n_m"] > 0.0
        for point in payload["torque_profile"]
    )
    assert payload["torque_partition_summary"]["total_surface_torque_n_m"] == pytest.approx(
        payload["torque_partition_summary"]["body_surface_torque_n_m"]
        + payload["torque_partition_summary"]["centralizer_surface_torque_n_m"]
    )
    assert payload["convergence_metadata"]["coupling"]["final_max_torque_update_n_m"] == pytest.approx(
        payload["coupling_final_max_torque_update_n_m"]
    )
    assert "traceability" in payload
    assert payload["traceability"]["convergence"]["global_solver"]["update_tolerance_m"] == pytest.approx(1.0e-8)
    assert payload["traceability"]["convergence"]["coupling"]["final_max_profile_update_n"] == pytest.approx(
        payload["coupling_final_max_profile_update_n"]
    )
    assert payload["traceability"]["convergence"]["coupling"]["final_max_torque_update_n_m"] == pytest.approx(
        payload["coupling_final_max_torque_update_n_m"]
    )
    assert len(payload["traceability"]["centralizer_parameters"]) == payload["centralizer_summary"]["spec_count"]
    assert payload["traceability"]["centralizer_parameters"][0]["calibration_status"] in {
        "explicit_power_law_input",
        "nominal_restoring_force_fallback",
    }
    assert payload["traceability"]["centralizer_parameters"][0]["tangential_torque_parameter_status"] == (
        "derived_from_nominal_running_to_restoring_ratio"
    )
    assert len(payload["warnings"]) >= 11
    assert len(payload["todos"]) == 6


def test_benchmark_suite_runs_and_validations_pass(tmp_path: Path) -> None:
    suite_path = ROOT / "benchmarks" / "suites" / "phase10_validation.yaml"

    payload, output_path = run_benchmark_suite(suite_path, tmp_path / "benchmark-output")

    assert output_path.exists()
    assert payload["summary"]["case_count"] == 14
    assert payload["summary"]["validation_count"] >= 12
    assert payload["summary"]["all_passed"] is True
    assert all(case["failed_check_count"] == 0 for case in payload["cases"])
    assert all(validation["passed"] for validation in payload["validations"])


def test_force_deflection_calibration_recovers_expected_power_law(tmp_path: Path) -> None:
    config_path = ROOT / "benchmarks" / "calibration" / "force_deflection_pairs.yaml"

    payload, output_path = run_bow_spring_calibration(config_path, tmp_path / "force-fit.json")

    assert output_path.exists()
    assert payload["status"] == "calibrated"
    assert payload["resolved_parameters"]["blade_power_law_k_n_per_m_pow_p"] == pytest.approx(80000.0)
    assert payload["resolved_parameters"]["blade_power_law_p"] == pytest.approx(1.25)
    assert payload["fit_quality"]["rmse_force_n"] == pytest.approx(0.0, abs=1.0e-10)
    assert len(payload["evaluation_points"]) == 4


def test_nominal_point_calibration_produces_positive_parameters(tmp_path: Path) -> None:
    config_path = ROOT / "benchmarks" / "calibration" / "nominal_force_points.yaml"

    payload, output_path = run_bow_spring_calibration(config_path, tmp_path / "nominal-fit.json")

    assert output_path.exists()
    assert payload["status"] == "calibrated"
    assert payload["resolved_parameters"]["blade_power_law_k_n_per_m_pow_p"] > 0.0
    assert payload["resolved_parameters"]["blade_power_law_p"] > 0.0
    assert len(payload["evaluation_points"]) == 2
