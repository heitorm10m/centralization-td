from __future__ import annotations

from dataclasses import dataclass
import math

from .models import LoadedCase, StringSectionModel


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2))
    if norm <= 0.0:
        return (0.0, 0.0, 1.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _tangent_from_angles(inclination_rad: float, azimuth_rad: float) -> tuple[float, float, float]:
    return (
        math.sin(inclination_rad) * math.cos(azimuth_rad),
        math.sin(inclination_rad) * math.sin(azimuth_rad),
        math.cos(inclination_rad),
    )


def _angle_between(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    lhs = _normalize(left)
    rhs = _normalize(right)
    dot_product = (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2])
    return math.acos(_clamp(dot_product, -1.0, 1.0))


@dataclass(slots=True)
class CentralizerPlacementModel:
    source_name: str
    type: str
    measured_depth_m: float
    outer_diameter_m: float
    nominal_restoring_force_n: float
    nominal_running_force_n: float
    influence_length_m: float


@dataclass(slots=True)
class DiscretizedSegmentModel:
    segment_index: int
    section_index: int
    measured_depth_start_m: float
    measured_depth_end_m: float
    measured_depth_center_m: float
    segment_length_m: float
    inclination_rad: float
    azimuth_rad: float
    curvature_rad_per_m: float
    tvd_m: float
    northing_m: float
    easting_m: float
    reference_hole_diameter_m: float
    fluid_density_kg_per_m3: float
    effective_line_weight_n_per_m: float
    second_moment_of_area_m4: float
    bending_stiffness_n_m2: float
    section: StringSectionModel


@dataclass(slots=True)
class LateralEquilibriumStateModel:
    gravity_lateral_load_n_per_m: float
    curvature_lateral_load_n_per_m: float
    equivalent_lateral_load_n_per_m: float
    equivalent_lateral_force_n: float
    bending_lateral_stiffness_n_per_m: float
    axial_tension_lateral_stiffness_n_per_m: float
    structural_lateral_stiffness_n_per_m: float
    free_eccentricity_estimate_m: float


@dataclass(slots=True)
class CentralizerSupportEffectModel:
    pipe_body_clearance_m: float
    support_contact_clearance_m: float
    support_outer_diameter_m: float
    centering_stiffness_n_per_m: float
    support_contact_penalty_n_per_m: float
    nearby_centralizer_count: int
    support_present: bool


@dataclass(slots=True)
class GlobalNodeInputModel:
    node_index: int
    measured_depth_m: float
    segment_length_m: float
    inclination_rad: float
    curvature_rad_per_m: float
    effective_line_weight_n_per_m: float
    effective_axial_load_n: float
    bending_stiffness_n_m2: float
    bending_moment_n_m: float
    bending_stress_pa: float
    bending_strain_estimate: float
    gravity_lateral_load_n_per_m: float
    curvature_lateral_load_n_per_m: float
    equivalent_lateral_load_n_per_m: float
    equivalent_lateral_force_n: float
    bending_lateral_stiffness_n_per_m: float
    axial_tension_lateral_stiffness_n_per_m: float
    structural_lateral_stiffness_n_per_m: float
    centralizer_centering_stiffness_n_per_m: float
    support_contact_penalty_n_per_m: float
    body_contact_penalty_n_per_m: float
    support_outer_diameter_m: float
    pipe_body_clearance_m: float
    support_contact_clearance_m: float
    nearby_centralizer_count: int
    free_eccentricity_estimate_m: float
    section: StringSectionModel


@dataclass(slots=True)
class GlobalContactStateModel:
    support_contact_active: bool = False
    pipe_body_contact_active: bool = False


@dataclass(slots=True)
class GlobalNodeSolutionModel:
    measured_depth_m: float
    eccentricity_estimate_m: float
    eccentricity_ratio: float
    standoff_estimate: float
    support_normal_reaction_estimate_n: float
    body_normal_reaction_estimate_n: float
    normal_reaction_estimate_n: float
    normal_reaction_estimate_n_per_m: float
    support_in_contact: bool
    pipe_body_in_contact: bool
    contact_state: str


@dataclass(slots=True)
class GlobalSolverResultModel:
    node_solutions: list[GlobalNodeSolutionModel]
    contact_states: list[GlobalContactStateModel]
    iteration_count: int
    final_update_norm_m: float


@dataclass(slots=True)
class MechanicalSegmentResultModel:
    measured_depth_start_m: float
    measured_depth_end_m: float
    measured_depth_center_m: float
    segment_length_m: float
    section_name: str
    inclination_rad: float
    curvature_rad_per_m: float
    effective_line_weight_n_per_m: float
    effective_axial_load_n: float
    bending_stiffness_n_m2: float
    bending_moment_n_m: float
    bending_stress_pa: float
    bending_strain_estimate: float
    gravity_lateral_load_n_per_m: float
    curvature_lateral_load_n_per_m: float
    equivalent_lateral_load_n_per_m: float
    equivalent_lateral_force_n: float
    bending_lateral_stiffness_n_per_m: float
    axial_tension_lateral_stiffness_n_per_m: float
    structural_lateral_stiffness_n_per_m: float
    centralizer_centering_stiffness_n_per_m: float
    support_contact_penalty_n_per_m: float
    body_contact_penalty_n_per_m: float
    support_outer_diameter_m: float
    pipe_body_clearance_m: float
    support_contact_clearance_m: float
    free_eccentricity_estimate_m: float
    eccentricity_estimate_m: float
    eccentricity_ratio: float
    standoff_estimate: float
    support_normal_reaction_estimate_n: float
    body_normal_reaction_estimate_n: float
    normal_reaction_estimate_n: float
    normal_reaction_estimate_n_per_m: float
    nearby_centralizer_count: int
    contact_iteration_count: int
    contact_state: str
    support_in_contact: bool
    pipe_body_in_contact: bool


@dataclass(slots=True)
class MechanicalSummaryModel:
    segment_count: int
    target_segment_length_m: float
    global_solver_iteration_count: int
    global_solver_final_update_norm_m: float
    top_effective_axial_load_n: float
    minimum_effective_axial_load_n: float
    maximum_effective_axial_load_n: float
    maximum_bending_moment_n_m: float
    maximum_bending_stress_pa: float
    maximum_bending_strain_estimate: float
    maximum_equivalent_lateral_load_n_per_m: float
    maximum_eccentricity_estimate_m: float
    maximum_eccentricity_ratio: float
    minimum_standoff_estimate: float
    maximum_normal_reaction_estimate_n: float
    contact_segment_count: int
    support_contact_segment_count: int
    pipe_body_contact_segment_count: int


def _evenly_spaced_positions(start_md_m: float, end_md_m: float, count: int) -> list[float]:
    if count <= 0 or end_md_m <= start_md_m:
        return []
    spacing_m = (end_md_m - start_md_m) / float(count + 1)
    return [start_md_m + (spacing_m * float(index + 1)) for index in range(count)]


def expand_centralizer_placements(loaded_case: LoadedCase) -> list[CentralizerPlacementModel]:
    coverage_start_md_m = loaded_case.string.sections[0].md_start_m
    coverage_end_md_m = loaded_case.string.sections[-1].md_end_m
    placements: list[CentralizerPlacementModel] = []

    for spec in loaded_case.centralizers.centralizers:
        if spec.installation_md_m:
            positions = list(spec.installation_md_m)
        elif spec.count_hint is not None:
            positions = _evenly_spaced_positions(
                coverage_start_md_m,
                coverage_end_md_m,
                spec.count_hint,
            )
        else:
            positions = []
            md_m = coverage_start_md_m + (0.5 * spec.spacing_hint_m)
            while md_m < coverage_end_md_m:
                positions.append(md_m)
                md_m += spec.spacing_hint_m

        for measured_depth_m in positions:
            placements.append(
                CentralizerPlacementModel(
                    source_name=spec.name,
                    type=spec.type,
                    measured_depth_m=measured_depth_m,
                    outer_diameter_m=spec.outer_diameter_m,
                    nominal_restoring_force_n=spec.nominal_restoring_force_n,
                    nominal_running_force_n=spec.nominal_running_force_n,
                    influence_length_m=spec.spacing_hint_m,
                )
            )

    placements.sort(key=lambda item: item.measured_depth_m)
    return placements


def _section_for_md(loaded_case: LoadedCase, measured_depth_m: float) -> tuple[int, StringSectionModel]:
    for index, section in enumerate(loaded_case.string.sections):
        if section.contains_md(measured_depth_m):
            return index, section
    raise ValueError("No string section covers the requested measured depth.")


def discretize_case(loaded_case: LoadedCase) -> tuple[list[DiscretizedSegmentModel], list[CentralizerPlacementModel]]:
    loaded_case.validate()
    placements = expand_centralizer_placements(loaded_case)
    coverage_start_md_m = loaded_case.string.sections[0].md_start_m
    coverage_end_md_m = loaded_case.string.sections[-1].md_end_m
    segments: list[DiscretizedSegmentModel] = []
    segment_start_md_m = coverage_start_md_m

    while segment_start_md_m < coverage_end_md_m:
        segment_end_md_m = min(
            segment_start_md_m + loaded_case.discretization_step_m,
            coverage_end_md_m,
        )
        center_md_m = 0.5 * (segment_start_md_m + segment_end_md_m)
        segment_length_m = segment_end_md_m - segment_start_md_m

        start_point = loaded_case.well.interpolate(segment_start_md_m)
        center_point = loaded_case.well.interpolate(center_md_m)
        end_point = loaded_case.well.interpolate(segment_end_md_m)
        start_tangent = _tangent_from_angles(start_point.inclination_rad, start_point.azimuth_rad)
        end_tangent = _tangent_from_angles(end_point.inclination_rad, end_point.azimuth_rad)
        curvature_rad_per_m = (
            0.0
            if segment_length_m <= 0.0
            else _angle_between(start_tangent, end_tangent) / segment_length_m
        )
        section_index, section = _section_for_md(loaded_case, center_md_m)

        segments.append(
            DiscretizedSegmentModel(
                segment_index=len(segments),
                section_index=section_index,
                measured_depth_start_m=segment_start_md_m,
                measured_depth_end_m=segment_end_md_m,
                measured_depth_center_m=center_md_m,
                segment_length_m=segment_length_m,
                inclination_rad=center_point.inclination_rad,
                azimuth_rad=center_point.azimuth_rad,
                curvature_rad_per_m=curvature_rad_per_m,
                tvd_m=center_point.tvd_m,
                northing_m=center_point.northing_m,
                easting_m=center_point.easting_m,
                reference_hole_diameter_m=loaded_case.reference_hole_diameter_m,
                fluid_density_kg_per_m3=loaded_case.fluid_density_kg_per_m3,
                effective_line_weight_n_per_m=section.effective_line_weight_n_per_m(
                    loaded_case.fluid_density_kg_per_m3
                ),
                second_moment_of_area_m4=section.second_moment_of_area_m4,
                bending_stiffness_n_m2=section.bending_stiffness_n_m2,
                section=section,
            )
        )
        segment_start_md_m = segment_end_md_m

    return segments, placements


def _evaluate_lateral_equilibrium(
    segment: DiscretizedSegmentModel,
    effective_axial_load_n: float,
) -> LateralEquilibriumStateModel:
    segment_length_m = max(segment.segment_length_m, 1.0e-6)
    gravity_lateral_load_n_per_m = abs(
        segment.effective_line_weight_n_per_m * math.sin(segment.inclination_rad)
    )
    curvature_lateral_load_n_per_m = max(effective_axial_load_n, 0.0) * segment.curvature_rad_per_m
    equivalent_lateral_load_n_per_m = (
        gravity_lateral_load_n_per_m + curvature_lateral_load_n_per_m
    )
    equivalent_lateral_force_n = equivalent_lateral_load_n_per_m * segment.segment_length_m
    bending_lateral_stiffness_n_per_m = 0.0
    if segment.bending_stiffness_n_m2 > 0.0:
        bending_lateral_stiffness_n_per_m = (
            384.0 * segment.bending_stiffness_n_m2 / (5.0 * (segment_length_m**3))
        )
    axial_tension_lateral_stiffness_n_per_m = 0.0
    if effective_axial_load_n > 0.0:
        axial_tension_lateral_stiffness_n_per_m = 4.0 * effective_axial_load_n / segment_length_m
    structural_lateral_stiffness_n_per_m = (
        bending_lateral_stiffness_n_per_m + axial_tension_lateral_stiffness_n_per_m
    )
    free_eccentricity_estimate_m = (
        0.0
        if structural_lateral_stiffness_n_per_m <= 0.0
        else equivalent_lateral_force_n / structural_lateral_stiffness_n_per_m
    )
    return LateralEquilibriumStateModel(
        gravity_lateral_load_n_per_m=gravity_lateral_load_n_per_m,
        curvature_lateral_load_n_per_m=curvature_lateral_load_n_per_m,
        equivalent_lateral_load_n_per_m=equivalent_lateral_load_n_per_m,
        equivalent_lateral_force_n=equivalent_lateral_force_n,
        bending_lateral_stiffness_n_per_m=bending_lateral_stiffness_n_per_m,
        axial_tension_lateral_stiffness_n_per_m=axial_tension_lateral_stiffness_n_per_m,
        structural_lateral_stiffness_n_per_m=structural_lateral_stiffness_n_per_m,
        free_eccentricity_estimate_m=free_eccentricity_estimate_m,
    )


def _evaluate_support_effect(
    segment: DiscretizedSegmentModel,
    placements: list[CentralizerPlacementModel],
    structural_lateral_stiffness_n_per_m: float,
    contact_penalty_scale: float,
) -> CentralizerSupportEffectModel:
    support_outer_diameter_m = segment.section.outer_diameter_m
    if segment.reference_hole_diameter_m <= 0.0:
        return CentralizerSupportEffectModel(
            pipe_body_clearance_m=0.0,
            support_contact_clearance_m=0.0,
            support_outer_diameter_m=support_outer_diameter_m,
            centering_stiffness_n_per_m=0.0,
            support_contact_penalty_n_per_m=0.0,
            nearby_centralizer_count=0,
            support_present=False,
        )

    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    pipe_body_clearance_m = max(0.0, hole_radius_m - segment.section.outer_radius_m)
    reference_deflection_m = max(
        pipe_body_clearance_m,
        max(0.05 * segment.section.outer_diameter_m, 1.0e-4),
    )

    nearby_centralizer_count = 0
    support_present = False
    centering_stiffness_n_per_m = 0.0
    for placement in placements:
        half_influence_length_m = max(
            0.5 * placement.influence_length_m,
            0.5 * segment.segment_length_m,
        )
        distance_to_segment_center_m = abs(
            placement.measured_depth_m - segment.measured_depth_center_m
        )
        if distance_to_segment_center_m > half_influence_length_m:
            continue

        proximity_weight = max(0.0, 1.0 - (distance_to_segment_center_m / half_influence_length_m))
        nearby_centralizer_count += 1
        support_present = True
        support_outer_diameter_m = max(support_outer_diameter_m, placement.outer_diameter_m)
        centering_stiffness_n_per_m += (
            proximity_weight * placement.nominal_restoring_force_n / reference_deflection_m
        )

    support_contact_clearance_m = max(0.0, hole_radius_m - (0.5 * support_outer_diameter_m))
    support_contact_penalty_n_per_m = 0.0
    if support_present:
        support_contact_penalty_n_per_m = contact_penalty_scale * max(
            structural_lateral_stiffness_n_per_m + centering_stiffness_n_per_m,
            1.0,
        )

    return CentralizerSupportEffectModel(
        pipe_body_clearance_m=pipe_body_clearance_m,
        support_contact_clearance_m=support_contact_clearance_m,
        support_outer_diameter_m=support_outer_diameter_m,
        centering_stiffness_n_per_m=centering_stiffness_n_per_m,
        support_contact_penalty_n_per_m=support_contact_penalty_n_per_m,
        nearby_centralizer_count=nearby_centralizer_count,
        support_present=support_present,
    )


def build_global_node_inputs(
    loaded_case: LoadedCase,
) -> tuple[
    list[DiscretizedSegmentModel],
    list[CentralizerPlacementModel],
    list[GlobalNodeInputModel],
]:
    segments, placements = discretize_case(loaded_case)
    load_below_n = 0.0
    axial_loads_n = [0.0 for _ in segments]

    for segment_index in range(len(segments) - 1, -1, -1):
        segment = segments[segment_index]
        tangential_weight_n = (
            segment.effective_line_weight_n_per_m
            * math.cos(segment.inclination_rad)
            * segment.segment_length_m
        )
        axial_loads_n[segment_index] = load_below_n + (0.5 * tangential_weight_n)
        load_below_n += tangential_weight_n

    nodes: list[GlobalNodeInputModel] = []
    for segment, effective_axial_load_n in zip(segments, axial_loads_n, strict=True):
        lateral_state = _evaluate_lateral_equilibrium(segment, effective_axial_load_n)
        support_effect = _evaluate_support_effect(
            segment,
            placements,
            lateral_state.structural_lateral_stiffness_n_per_m,
            loaded_case.contact_penalty_scale,
        )
        base_lateral_stiffness_n_per_m = (
            lateral_state.structural_lateral_stiffness_n_per_m
            + support_effect.centering_stiffness_n_per_m
        )
        nodes.append(
            GlobalNodeInputModel(
                node_index=segment.segment_index,
                measured_depth_m=segment.measured_depth_center_m,
                segment_length_m=segment.segment_length_m,
                inclination_rad=segment.inclination_rad,
                curvature_rad_per_m=segment.curvature_rad_per_m,
                effective_line_weight_n_per_m=segment.effective_line_weight_n_per_m,
                effective_axial_load_n=effective_axial_load_n,
                bending_stiffness_n_m2=segment.bending_stiffness_n_m2,
                bending_moment_n_m=segment.bending_stiffness_n_m2 * segment.curvature_rad_per_m,
                bending_stress_pa=(
                    segment.section.young_modulus_pa
                    * segment.curvature_rad_per_m
                    * segment.section.outer_radius_m
                ),
                bending_strain_estimate=segment.curvature_rad_per_m * segment.section.outer_radius_m,
                gravity_lateral_load_n_per_m=lateral_state.gravity_lateral_load_n_per_m,
                curvature_lateral_load_n_per_m=lateral_state.curvature_lateral_load_n_per_m,
                equivalent_lateral_load_n_per_m=lateral_state.equivalent_lateral_load_n_per_m,
                equivalent_lateral_force_n=lateral_state.equivalent_lateral_force_n,
                bending_lateral_stiffness_n_per_m=lateral_state.bending_lateral_stiffness_n_per_m,
                axial_tension_lateral_stiffness_n_per_m=lateral_state.axial_tension_lateral_stiffness_n_per_m,
                structural_lateral_stiffness_n_per_m=lateral_state.structural_lateral_stiffness_n_per_m,
                centralizer_centering_stiffness_n_per_m=support_effect.centering_stiffness_n_per_m,
                support_contact_penalty_n_per_m=support_effect.support_contact_penalty_n_per_m,
                body_contact_penalty_n_per_m=(
                    2.0
                    * loaded_case.contact_penalty_scale
                    * max(base_lateral_stiffness_n_per_m, 1.0)
                ),
                support_outer_diameter_m=support_effect.support_outer_diameter_m,
                pipe_body_clearance_m=support_effect.pipe_body_clearance_m,
                support_contact_clearance_m=support_effect.support_contact_clearance_m,
                nearby_centralizer_count=support_effect.nearby_centralizer_count,
                free_eccentricity_estimate_m=lateral_state.free_eccentricity_estimate_m,
                section=segment.section,
            )
        )

    return segments, placements, nodes


def assemble_global_linear_system(
    nodes: list[GlobalNodeInputModel],
    contact_states: list[GlobalContactStateModel],
) -> tuple[list[list[float]], list[float]]:
    if len(nodes) != len(contact_states):
        raise ValueError("Global assembly requires one contact state per node.")

    node_count = len(nodes)
    stiffness_matrix = [[0.0 for _ in range(node_count)] for _ in range(node_count)]
    load_vector = [0.0 for _ in range(node_count)]

    for node_index, node in enumerate(nodes):
        load_vector[node_index] += node.equivalent_lateral_force_n
        stiffness_matrix[node_index][node_index] += node.centralizer_centering_stiffness_n_per_m

        if contact_states[node_index].support_contact_active:
            stiffness_matrix[node_index][node_index] += node.support_contact_penalty_n_per_m
            load_vector[node_index] += (
                node.support_contact_penalty_n_per_m * node.support_contact_clearance_m
            )

        if contact_states[node_index].pipe_body_contact_active:
            stiffness_matrix[node_index][node_index] += node.body_contact_penalty_n_per_m
            load_vector[node_index] += node.body_contact_penalty_n_per_m * node.pipe_body_clearance_m

    for node_index in range(1, node_count):
        spacing_m = max(
            nodes[node_index].measured_depth_m - nodes[node_index - 1].measured_depth_m,
            1.0e-6,
        )
        effective_axial_load_n = max(
            0.0,
            0.5
            * (
                nodes[node_index].effective_axial_load_n
                + nodes[node_index - 1].effective_axial_load_n
            ),
        )
        axial_coefficient = effective_axial_load_n / spacing_m

        stiffness_matrix[node_index - 1][node_index - 1] += axial_coefficient
        stiffness_matrix[node_index - 1][node_index] -= axial_coefficient
        stiffness_matrix[node_index][node_index - 1] -= axial_coefficient
        stiffness_matrix[node_index][node_index] += axial_coefficient

    for node_index in range(1, node_count - 1):
        spacing_left_m = max(
            nodes[node_index].measured_depth_m - nodes[node_index - 1].measured_depth_m,
            1.0e-6,
        )
        spacing_right_m = max(
            nodes[node_index + 1].measured_depth_m - nodes[node_index].measured_depth_m,
            1.0e-6,
        )
        average_spacing_m = 0.5 * (spacing_left_m + spacing_right_m)
        bending_coefficient = nodes[node_index].bending_stiffness_n_m2 / max(
            average_spacing_m**3,
            1.0e-6,
        )

        stiffness_matrix[node_index - 1][node_index - 1] += bending_coefficient
        stiffness_matrix[node_index - 1][node_index] -= 2.0 * bending_coefficient
        stiffness_matrix[node_index - 1][node_index + 1] += bending_coefficient
        stiffness_matrix[node_index][node_index - 1] -= 2.0 * bending_coefficient
        stiffness_matrix[node_index][node_index] += 4.0 * bending_coefficient
        stiffness_matrix[node_index][node_index + 1] -= 2.0 * bending_coefficient
        stiffness_matrix[node_index + 1][node_index - 1] += bending_coefficient
        stiffness_matrix[node_index + 1][node_index] -= 2.0 * bending_coefficient
        stiffness_matrix[node_index + 1][node_index + 1] += bending_coefficient

    def apply_centered_end_boundary(boundary_index: int) -> None:
        for row_index in range(node_count):
            stiffness_matrix[row_index][boundary_index] = 0.0
        for column_index in range(node_count):
            stiffness_matrix[boundary_index][column_index] = 0.0
        stiffness_matrix[boundary_index][boundary_index] = 1.0
        load_vector[boundary_index] = 0.0

    if node_count == 1:
        apply_centered_end_boundary(0)
    elif node_count > 1:
        apply_centered_end_boundary(0)
        apply_centered_end_boundary(node_count - 1)

    return stiffness_matrix, load_vector


def _solve_dense_linear_system(
    matrix: list[list[float]],
    rhs: list[float],
) -> list[float]:
    dimension = len(rhs)
    working_matrix = [row[:] for row in matrix]
    working_rhs = list(rhs)

    for pivot_index in range(dimension):
        best_row_index = pivot_index
        best_pivot_value = abs(working_matrix[pivot_index][pivot_index])
        for row_index in range(pivot_index + 1, dimension):
            candidate_value = abs(working_matrix[row_index][pivot_index])
            if candidate_value > best_pivot_value:
                best_pivot_value = candidate_value
                best_row_index = row_index

        if best_pivot_value <= 1.0e-12:
            raise ValueError("Global solver encountered a singular linear system.")

        if best_row_index != pivot_index:
            working_matrix[pivot_index], working_matrix[best_row_index] = (
                working_matrix[best_row_index],
                working_matrix[pivot_index],
            )
            working_rhs[pivot_index], working_rhs[best_row_index] = (
                working_rhs[best_row_index],
                working_rhs[pivot_index],
            )

        pivot_value = working_matrix[pivot_index][pivot_index]
        for row_index in range(pivot_index + 1, dimension):
            elimination_factor = working_matrix[row_index][pivot_index] / pivot_value
            if elimination_factor == 0.0:
                continue

            for column_index in range(pivot_index, dimension):
                working_matrix[row_index][column_index] -= (
                    elimination_factor * working_matrix[pivot_index][column_index]
                )
            working_rhs[row_index] -= elimination_factor * working_rhs[pivot_index]

    solution = [0.0 for _ in range(dimension)]
    for row_index in range(dimension - 1, -1, -1):
        back_substitution_sum = working_rhs[row_index]
        for column_index in range(row_index + 1, dimension):
            back_substitution_sum -= working_matrix[row_index][column_index] * solution[column_index]
        solution[row_index] = back_substitution_sum / working_matrix[row_index][row_index]

    return solution


def run_global_lateral_solver(
    nodes: list[GlobalNodeInputModel],
    *,
    max_iterations: int,
) -> GlobalSolverResultModel:
    contact_states = [GlobalContactStateModel() for _ in nodes]
    displacements_m = [0.0 for _ in nodes]
    final_update_norm_m = 0.0
    iteration_count = 0

    for iteration_index in range(max_iterations):
        stiffness_matrix, load_vector = assemble_global_linear_system(nodes, contact_states)
        updated_displacements_m = _solve_dense_linear_system(stiffness_matrix, load_vector)

        final_update_norm_m = 0.0
        contact_state_changed = False
        for node_index, node in enumerate(nodes):
            updated_eccentricity_m = max(0.0, updated_displacements_m[node_index])
            final_update_norm_m = max(
                final_update_norm_m,
                abs(updated_eccentricity_m - displacements_m[node_index]),
            )
            support_contact_active = (
                node.support_contact_penalty_n_per_m > 0.0
                and updated_eccentricity_m > node.support_contact_clearance_m
            )
            pipe_body_contact_active = updated_eccentricity_m > node.pipe_body_clearance_m
            if (
                support_contact_active != contact_states[node_index].support_contact_active
                or pipe_body_contact_active != contact_states[node_index].pipe_body_contact_active
            ):
                contact_state_changed = True

            contact_states[node_index].support_contact_active = support_contact_active
            contact_states[node_index].pipe_body_contact_active = pipe_body_contact_active
            displacements_m[node_index] = updated_eccentricity_m

        iteration_count = iteration_index + 1
        if not contact_state_changed and final_update_norm_m <= 1.0e-8:
            break

    node_solutions: list[GlobalNodeSolutionModel] = []
    for node_index, node in enumerate(nodes):
        eccentricity_estimate_m = max(0.0, displacements_m[node_index])
        support_normal_reaction_estimate_n = max(
            0.0,
            node.support_contact_penalty_n_per_m
            * (eccentricity_estimate_m - node.support_contact_clearance_m),
        )
        body_normal_reaction_estimate_n = max(
            0.0,
            node.body_contact_penalty_n_per_m
            * (eccentricity_estimate_m - node.pipe_body_clearance_m),
        )
        normal_reaction_estimate_n = (
            support_normal_reaction_estimate_n + body_normal_reaction_estimate_n
        )
        if node.pipe_body_clearance_m > 0.0:
            eccentricity_ratio = eccentricity_estimate_m / node.pipe_body_clearance_m
            standoff_estimate = _clamp(1.0 - eccentricity_ratio, 0.0, 1.0)
        else:
            eccentricity_ratio = 1.0
            standoff_estimate = 0.0

        support_in_contact = support_normal_reaction_estimate_n > 0.0
        pipe_body_in_contact = body_normal_reaction_estimate_n > 0.0
        if pipe_body_in_contact:
            contact_state = "pipe-body-contact"
        elif support_in_contact:
            contact_state = "support-contact"
        else:
            contact_state = "free"

        node_solutions.append(
            GlobalNodeSolutionModel(
                measured_depth_m=node.measured_depth_m,
                eccentricity_estimate_m=eccentricity_estimate_m,
                eccentricity_ratio=eccentricity_ratio,
                standoff_estimate=standoff_estimate,
                support_normal_reaction_estimate_n=support_normal_reaction_estimate_n,
                body_normal_reaction_estimate_n=body_normal_reaction_estimate_n,
                normal_reaction_estimate_n=normal_reaction_estimate_n,
                normal_reaction_estimate_n_per_m=(
                    0.0 if node.segment_length_m <= 0.0 else normal_reaction_estimate_n / node.segment_length_m
                ),
                support_in_contact=support_in_contact,
                pipe_body_in_contact=pipe_body_in_contact,
                contact_state=contact_state,
            )
        )

    return GlobalSolverResultModel(
        node_solutions=node_solutions,
        contact_states=contact_states,
        iteration_count=iteration_count,
        final_update_norm_m=final_update_norm_m,
    )


def run_mechanical_baseline(
    loaded_case: LoadedCase,
) -> tuple[MechanicalSummaryModel, list[MechanicalSegmentResultModel], list[CentralizerPlacementModel]]:
    segments, placements, nodes = build_global_node_inputs(loaded_case)
    if not nodes:
        return (
            MechanicalSummaryModel(
                segment_count=0,
                target_segment_length_m=loaded_case.discretization_step_m,
                global_solver_iteration_count=0,
                global_solver_final_update_norm_m=0.0,
                top_effective_axial_load_n=0.0,
                minimum_effective_axial_load_n=0.0,
                maximum_effective_axial_load_n=0.0,
                maximum_bending_moment_n_m=0.0,
                maximum_bending_stress_pa=0.0,
                maximum_bending_strain_estimate=0.0,
                maximum_equivalent_lateral_load_n_per_m=0.0,
                maximum_eccentricity_estimate_m=0.0,
                maximum_eccentricity_ratio=0.0,
                minimum_standoff_estimate=0.0,
                maximum_normal_reaction_estimate_n=0.0,
                contact_segment_count=0,
                support_contact_segment_count=0,
                pipe_body_contact_segment_count=0,
            ),
            [],
            placements,
        )

    global_result = run_global_lateral_solver(
        nodes,
        max_iterations=loaded_case.global_solver_max_iterations,
    )

    profile: list[MechanicalSegmentResultModel] = []
    top_effective_axial_load_n = 0.0
    minimum_effective_axial_load_n = float("inf")
    maximum_effective_axial_load_n = float("-inf")
    maximum_bending_moment_n_m = 0.0
    maximum_bending_stress_pa = 0.0
    maximum_bending_strain_estimate = 0.0
    maximum_equivalent_lateral_load_n_per_m = 0.0
    maximum_eccentricity_estimate_m = 0.0
    maximum_eccentricity_ratio = 0.0
    minimum_standoff_estimate = 1.0
    maximum_normal_reaction_estimate_n = 0.0
    contact_segment_count = 0
    support_contact_segment_count = 0
    pipe_body_contact_segment_count = 0

    for segment, node, solution in zip(segments, nodes, global_result.node_solutions, strict=True):
        tangential_weight_n = (
            segment.effective_line_weight_n_per_m
            * math.cos(segment.inclination_rad)
            * segment.segment_length_m
        )
        top_effective_axial_load_n += tangential_weight_n

        result = MechanicalSegmentResultModel(
            measured_depth_start_m=segment.measured_depth_start_m,
            measured_depth_end_m=segment.measured_depth_end_m,
            measured_depth_center_m=segment.measured_depth_center_m,
            segment_length_m=segment.segment_length_m,
            section_name=segment.section.name,
            inclination_rad=segment.inclination_rad,
            curvature_rad_per_m=segment.curvature_rad_per_m,
            effective_line_weight_n_per_m=segment.effective_line_weight_n_per_m,
            effective_axial_load_n=node.effective_axial_load_n,
            bending_stiffness_n_m2=segment.bending_stiffness_n_m2,
            bending_moment_n_m=node.bending_moment_n_m,
            bending_stress_pa=node.bending_stress_pa,
            bending_strain_estimate=node.bending_strain_estimate,
            gravity_lateral_load_n_per_m=node.gravity_lateral_load_n_per_m,
            curvature_lateral_load_n_per_m=node.curvature_lateral_load_n_per_m,
            equivalent_lateral_load_n_per_m=node.equivalent_lateral_load_n_per_m,
            equivalent_lateral_force_n=node.equivalent_lateral_force_n,
            bending_lateral_stiffness_n_per_m=node.bending_lateral_stiffness_n_per_m,
            axial_tension_lateral_stiffness_n_per_m=node.axial_tension_lateral_stiffness_n_per_m,
            structural_lateral_stiffness_n_per_m=node.structural_lateral_stiffness_n_per_m,
            centralizer_centering_stiffness_n_per_m=node.centralizer_centering_stiffness_n_per_m,
            support_contact_penalty_n_per_m=node.support_contact_penalty_n_per_m,
            body_contact_penalty_n_per_m=node.body_contact_penalty_n_per_m,
            support_outer_diameter_m=node.support_outer_diameter_m,
            pipe_body_clearance_m=node.pipe_body_clearance_m,
            support_contact_clearance_m=node.support_contact_clearance_m,
            free_eccentricity_estimate_m=node.free_eccentricity_estimate_m,
            eccentricity_estimate_m=solution.eccentricity_estimate_m,
            eccentricity_ratio=solution.eccentricity_ratio,
            standoff_estimate=solution.standoff_estimate,
            support_normal_reaction_estimate_n=solution.support_normal_reaction_estimate_n,
            body_normal_reaction_estimate_n=solution.body_normal_reaction_estimate_n,
            normal_reaction_estimate_n=solution.normal_reaction_estimate_n,
            normal_reaction_estimate_n_per_m=solution.normal_reaction_estimate_n_per_m,
            nearby_centralizer_count=node.nearby_centralizer_count,
            contact_iteration_count=global_result.iteration_count,
            contact_state=solution.contact_state,
            support_in_contact=solution.support_in_contact,
            pipe_body_in_contact=solution.pipe_body_in_contact,
        )
        profile.append(result)

        minimum_effective_axial_load_n = min(minimum_effective_axial_load_n, result.effective_axial_load_n)
        maximum_effective_axial_load_n = max(maximum_effective_axial_load_n, result.effective_axial_load_n)
        maximum_bending_moment_n_m = max(maximum_bending_moment_n_m, result.bending_moment_n_m)
        maximum_bending_stress_pa = max(maximum_bending_stress_pa, result.bending_stress_pa)
        maximum_bending_strain_estimate = max(
            maximum_bending_strain_estimate,
            result.bending_strain_estimate,
        )
        maximum_equivalent_lateral_load_n_per_m = max(
            maximum_equivalent_lateral_load_n_per_m,
            result.equivalent_lateral_load_n_per_m,
        )
        maximum_eccentricity_estimate_m = max(
            maximum_eccentricity_estimate_m,
            result.eccentricity_estimate_m,
        )
        maximum_eccentricity_ratio = max(maximum_eccentricity_ratio, result.eccentricity_ratio)
        minimum_standoff_estimate = min(minimum_standoff_estimate, result.standoff_estimate)
        maximum_normal_reaction_estimate_n = max(
            maximum_normal_reaction_estimate_n,
            result.normal_reaction_estimate_n,
        )
        contact_segment_count += int(result.support_in_contact or result.pipe_body_in_contact)
        support_contact_segment_count += int(result.support_in_contact)
        pipe_body_contact_segment_count += int(result.pipe_body_in_contact)

    summary = MechanicalSummaryModel(
        segment_count=len(profile),
        target_segment_length_m=loaded_case.discretization_step_m,
        global_solver_iteration_count=global_result.iteration_count,
        global_solver_final_update_norm_m=global_result.final_update_norm_m,
        top_effective_axial_load_n=top_effective_axial_load_n,
        minimum_effective_axial_load_n=minimum_effective_axial_load_n,
        maximum_effective_axial_load_n=maximum_effective_axial_load_n,
        maximum_bending_moment_n_m=maximum_bending_moment_n_m,
        maximum_bending_stress_pa=maximum_bending_stress_pa,
        maximum_bending_strain_estimate=maximum_bending_strain_estimate,
        maximum_equivalent_lateral_load_n_per_m=maximum_equivalent_lateral_load_n_per_m,
        maximum_eccentricity_estimate_m=maximum_eccentricity_estimate_m,
        maximum_eccentricity_ratio=maximum_eccentricity_ratio,
        minimum_standoff_estimate=minimum_standoff_estimate,
        maximum_normal_reaction_estimate_n=maximum_normal_reaction_estimate_n,
        contact_segment_count=contact_segment_count,
        support_contact_segment_count=support_contact_segment_count,
        pipe_body_contact_segment_count=pipe_body_contact_segment_count,
    )
    return summary, profile, placements
