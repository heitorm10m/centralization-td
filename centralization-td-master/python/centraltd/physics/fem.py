"""
Finite element formulation for stiff-string torque and drag model.

Reference: Dao et al. 2023, Geoenergy Science and Engineering 222, 211457

Equations implemented in this module:
  Eq. 11 — Local frame definition (eu, ev, ew) (PENDING)
  Eq. 12 — Global equilibrium K·X = Fo + Fc(X) (PENDING)
  Eq. 13 — Nodal gravity/buoyancy force transfer (IMPLEMENTED)
  Eq. 14 — Global body force vector assembly (IMPLEMENTED)
  Section 3.2 — Initial curvature force (PARTIAL: stub only; returns zero
    vector because the available extracts describe the concept but do not
    provide a complete algebraic algorithm)
  Eq. 30 — Newton-Raphson Taylor expansion (PENDING)
  Eq. 31 — Newton-Raphson iterative step (PENDING)
  Eq. 32 — Full system Jacobian (PENDING)
  Eq. 35 — Jacobian of body contact forces (PENDING)
  Eq. 36 — Jacobian of bow-spring contact forces (PENDING)

See CHANGELOG.md for implementation status of each equation.
See DECISIONS.md for decisions about this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

import numpy as np

from .bow_spring import (
    bow_contact_onset_clearance_m,
    centralizer_effective_contact_diameter_m,
    equivalent_bow_support_stiffness_n_per_m,
    placement_proximity_weight,
)
from .constants import DTYPE
from ..frames import build_frame_nodes, interpolate_frame
from ..models import LoadedCase, Nodal6DOFOutputModel, StringSectionModel

if TYPE_CHECKING:
    from .bow_spring import BowForceDetailModel
    from ..torque_drag_centralizer import CentralizerPlacementTorqueContributionModel


@dataclass(slots=True)
class ConstantForceElementModel:
    node_start_index: int
    node_end_index: int
    length_m: float
    density_kg_per_m3: float
    fluid_density_kg_per_m3: float
    cross_sectional_area_m2: float
    tangent_start_north_east_tvd: tuple[float, float, float]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _norm2(vector: tuple[float, float]) -> float:
    return math.sqrt((vector[0] ** 2) + (vector[1] ** 2))


def _normalize2(vector: tuple[float, float]) -> tuple[float, float]:
    magnitude = _norm2(vector)
    if magnitude <= 1.0e-12:
        return (1.0, 0.0)
    return (vector[0] / magnitude, vector[1] / magnitude)


def _dot3(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2])


def _cross3(
    lhs: tuple[float, float, float],
    rhs: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]),
        (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]),
        (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]),
    )


def _normalize3(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    magnitude = math.sqrt((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2))
    if magnitude <= 1.0e-12:
        return (0.0, 0.0, 0.0)
    return (vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude)


def _tangent_from_angles(inclination_rad: float, azimuth_rad: float) -> tuple[float, float, float]:
    return (
        math.sin(inclination_rad) * math.cos(azimuth_rad),
        math.sin(inclination_rad) * math.sin(azimuth_rad),
        math.cos(inclination_rad),
    )


def _angle_between(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    lhs = _normalize3(left)
    rhs = _normalize3(right)
    dot_product = _clamp(_dot3(lhs, rhs), -1.0, 1.0)
    return math.acos(dot_product)


def distributed_gravity_buoyancy_force_n_per_m(
    *,
    density_kg_per_m3: float,
    fluid_density_kg_per_m3: float,
    cross_sectional_area_m2: float,
    gravity_vector_north_east_tvd: tuple[float, float, float],
) -> np.ndarray:
    """
    Dao et al. 2023, Eq. 13.

    q = (rho - rho_f) * A * g
    """
    force_scale = (density_kg_per_m3 - fluid_density_kg_per_m3) * cross_sectional_area_m2
    return (force_scale * np.asarray(gravity_vector_north_east_tvd, dtype=DTYPE)).astype(DTYPE, copy=False)


def equivalent_gravity_buoyancy_nodal_loads(
    *,
    length_m: float,
    tangent_start_north_east_tvd: tuple[float, float, float],
    distributed_force_n_per_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Dao et al. 2023, Eq. 13.

    fg,1 = fg,2 = (L / 2) * q
    mg,1 = -mg,2 = (L^2 / 12) * (t1 x q)
    """
    tangent = _normalize3(tangent_start_north_east_tvd)
    distributed_force = np.asarray(distributed_force_n_per_m, dtype=DTYPE)
    translational_force = (0.5 * length_m * distributed_force).astype(DTYPE, copy=False)
    nodal_moment = (
        (length_m**2) / 12.0
    ) * np.asarray(_cross3(tangent, tuple(float(component) for component in distributed_force)), dtype=DTYPE)

    node_1_load = np.zeros(6, dtype=DTYPE)
    node_2_load = np.zeros(6, dtype=DTYPE)
    node_1_load[:3] = translational_force
    node_2_load[:3] = translational_force
    node_1_load[3:] = nodal_moment
    node_2_load[3:] = -nodal_moment
    return node_1_load, node_2_load


def build_gravity_buoyancy_force_vector(
    elements: list[ConstantForceElementModel],
    *,
    node_count: int,
    gravity_vector_north_east_tvd: tuple[float, float, float],
) -> np.ndarray:
    """
    Dao et al. 2023, Eq. 14.

    Assemble the constant gravity/buoyancy contribution Fo_gravity into the
    global 6-DOF nodal vector, including both translational forces and moments.
    """
    force_vector = np.zeros(6 * node_count, dtype=DTYPE)

    for element in elements:
        distributed_force = distributed_gravity_buoyancy_force_n_per_m(
            density_kg_per_m3=element.density_kg_per_m3,
            fluid_density_kg_per_m3=element.fluid_density_kg_per_m3,
            cross_sectional_area_m2=element.cross_sectional_area_m2,
            gravity_vector_north_east_tvd=gravity_vector_north_east_tvd,
        )
        node_1_load, node_2_load = equivalent_gravity_buoyancy_nodal_loads(
            length_m=element.length_m,
            tangent_start_north_east_tvd=element.tangent_start_north_east_tvd,
            distributed_force_n_per_m=distributed_force,
        )

        start_slice = slice(6 * element.node_start_index, 6 * (element.node_start_index + 1))
        end_slice = slice(6 * element.node_end_index, 6 * (element.node_end_index + 1))
        force_vector[start_slice] += node_1_load
        force_vector[end_slice] += node_2_load

    return force_vector


def build_initial_curvature_force(
    *,
    node_count: int,
) -> np.ndarray:
    # WARNING: Initial curvature force not fully implemented.
    # The paper (Dao et al. 2023, Section 3.2) describes the
    # concept but does not provide a complete algebraic algorithm
    # in the available extracts. Returning zero vector.
    # This must be revisited before final validation.
    return np.zeros(6 * node_count, dtype=DTYPE)


def build_constant_force_vector(
    elements: list[ConstantForceElementModel],
    *,
    node_count: int,
    gravity_vector_north_east_tvd: tuple[float, float, float],
) -> np.ndarray:
    """
    Dao et al. 2023, Eq. 12 with Eq. 13 and Eq. 14 contributions.
    """
    return build_gravity_buoyancy_force_vector(
        elements,
        node_count=node_count,
        gravity_vector_north_east_tvd=gravity_vector_north_east_tvd,
    ) + build_initial_curvature_force(node_count=node_count)


@dataclass(slots=True)
class CentralizerPlacementModel:
    source_name: str
    type: str
    measured_depth_m: float
    support_outer_diameter_m: float
    nominal_restoring_force_n: float
    nominal_running_force_n: float
    number_of_bows: int
    angular_orientation_reference_deg: float
    inner_clearance_to_pipe_m: float
    blade_power_law_k: float | None
    blade_power_law_p: float
    min_contact_diameter_m: float | None
    max_contact_diameter_m: float | None
    influence_length_m: float
    axial_force_ratio: float | None
    tangential_force_ratio: float | None


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
    curvature_normal_component_rad_per_m: float
    curvature_binormal_component_rad_per_m: float
    frame_rotation_change_rad: float
    tvd_m: float
    northing_m: float
    easting_m: float
    tangent_north_east_tvd: tuple[float, float, float]
    normal_north_east_tvd: tuple[float, float, float]
    binormal_north_east_tvd: tuple[float, float, float]
    reference_hole_diameter_m: float
    fluid_density_kg_per_m3: float
    effective_line_weight_n_per_m: float
    second_moment_of_area_m4: float
    bending_stiffness_n_m2: float
    section: StringSectionModel


@dataclass(slots=True)
class LateralEquilibriumStateModel:
    gravity_lateral_load_n_b_n_per_m: tuple[float, float]
    curvature_lateral_load_n_b_n_per_m: tuple[float, float]
    equivalent_lateral_load_n_b_n_per_m: tuple[float, float]
    equivalent_lateral_force_n_b: tuple[float, float]
    equivalent_lateral_load_magnitude_n_per_m: float
    equivalent_lateral_force_magnitude_n: float
    bending_lateral_stiffness_n_per_m: float
    axial_tension_lateral_stiffness_n_per_m: float
    structural_lateral_stiffness_n_per_m: float
    free_displacement_n_b_m: tuple[float, float]


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
    effective_axial_load_n: float
    bending_stiffness_n_m2: float
    bending_moment_n_m: float
    bending_stress_pa: float
    bending_strain_estimate: float
    gravity_lateral_load_n_b_n_per_m: tuple[float, float]
    curvature_lateral_load_n_b_n_per_m: tuple[float, float]
    equivalent_lateral_load_n_b_n_per_m: tuple[float, float]
    equivalent_lateral_force_n_b: tuple[float, float]
    equivalent_lateral_load_magnitude_n_per_m: float
    equivalent_lateral_force_magnitude_n: float
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
    free_displacement_n_b_m: tuple[float, float]
    section: StringSectionModel


@dataclass(slots=True)
class GlobalContactStateModel:
    support_contact_active: bool = False
    pipe_body_contact_active: bool = False
    contact_direction_n_b: tuple[float, float] = (1.0, 0.0)
@dataclass(slots=True)
class MechanicalSegmentResultModel:
    measured_depth_start_m: float
    measured_depth_end_m: float
    measured_depth_center_m: float
    segment_length_m: float
    section_name: str
    inclination_rad: float
    curvature_rad_per_m: float
    curvature_normal_component_rad_per_m: float
    curvature_binormal_component_rad_per_m: float
    frame_rotation_change_rad: float
    tangent_north_east_tvd: tuple[float, float, float]
    normal_north_east_tvd: tuple[float, float, float]
    binormal_north_east_tvd: tuple[float, float, float]
    effective_line_weight_n_per_m: float
    effective_axial_load_n: float
    bending_stiffness_n_m2: float
    bending_moment_n_m: float
    bending_stress_pa: float
    bending_strain_estimate: float
    bending_severity_estimate: float
    gravity_lateral_load_n_per_m: float
    curvature_lateral_load_n_per_m: float
    equivalent_lateral_load_n_per_m: float
    equivalent_lateral_force_n: float
    gravity_lateral_load_normal_n_per_m: float
    gravity_lateral_load_binormal_n_per_m: float
    curvature_lateral_load_normal_n_per_m: float
    curvature_lateral_load_binormal_n_per_m: float
    equivalent_lateral_load_normal_n_per_m: float
    equivalent_lateral_load_binormal_n_per_m: float
    bending_lateral_stiffness_n_per_m: float
    axial_tension_lateral_stiffness_n_per_m: float
    structural_lateral_stiffness_n_per_m: float
    centralizer_centering_stiffness_n_per_m: float
    support_contact_penalty_n_per_m: float
    body_contact_penalty_n_per_m: float
    support_outer_diameter_m: float
    pipe_body_clearance_m: float
    support_contact_clearance_m: float
    bow_force_details: list[BowForceDetailModel]
    free_nodal_output_6dof: Nodal6DOFOutputModel
    solved_nodal_output_6dof: Nodal6DOFOutputModel
    free_eccentricity_estimate_m: float
    free_lateral_displacement_normal_m: float
    free_lateral_displacement_binormal_m: float
    lateral_displacement_normal_m: float
    lateral_displacement_binormal_m: float
    eccentricity_normal_m: float
    eccentricity_binormal_m: float
    eccentricity_estimate_m: float
    eccentricity_ratio: float
    standoff_estimate: float
    contact_direction_normal: float
    contact_direction_binormal: float
    support_normal_reaction_normal_n: float
    support_normal_reaction_binormal_n: float
    body_normal_reaction_normal_n: float
    body_normal_reaction_binormal_n: float
    normal_reaction_normal_n: float
    normal_reaction_binormal_n: float
    support_normal_reaction_estimate_n: float
    body_normal_reaction_estimate_n: float
    normal_reaction_estimate_n: float
    normal_reaction_estimate_n_per_m: float
    bow_resultant_normal_n: float
    bow_resultant_binormal_n: float
    bow_resultant_magnitude_n: float
    centralizer_effective_radial_direction_normal: float
    centralizer_effective_radial_direction_binormal: float
    centralizer_tangential_direction_normal: float
    centralizer_tangential_direction_binormal: float
    centralizer_tangential_friction_normal_n: float
    centralizer_tangential_friction_binormal_n: float
    centralizer_tangential_friction_vector_magnitude_n: float
    centralizer_projected_contact_normal_n: float
    centralizer_friction_interaction_scale: float
    centralizer_axial_friction_n: float
    centralizer_tangential_friction_n: float
    centralizer_torque_increment_n_m: float
    centralizer_effective_contact_radius_m: float
    centralizer_torque_details: list[CentralizerPlacementTorqueContributionModel]
    centralizer_torque_status: str
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


@dataclass(slots=True)
class NormalReactionPointModel:
    measured_depth_m: float
    support_normal_reaction_estimate_n: float
    body_normal_reaction_estimate_n: float
    normal_reaction_estimate_n: float


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
                    support_outer_diameter_m=spec.resolved_support_outer_diameter_m(),
                    nominal_restoring_force_n=spec.nominal_restoring_force_n,
                    nominal_running_force_n=spec.nominal_running_force_n,
                    number_of_bows=spec.number_of_bows,
                    angular_orientation_reference_deg=spec.angular_orientation_reference_deg,
                    inner_clearance_to_pipe_m=spec.inner_clearance_to_pipe_m,
                    blade_power_law_k=spec.blade_power_law_k,
                    blade_power_law_p=spec.blade_power_law_p,
                    min_contact_diameter_m=spec.min_contact_diameter_m,
                    max_contact_diameter_m=spec.max_contact_diameter_m,
                    influence_length_m=spec.spacing_hint_m,
                    axial_force_ratio=spec.axial_force_ratio,
                    tangential_force_ratio=spec.tangential_force_ratio,
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
    frame_nodes = build_frame_nodes(loaded_case)
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
        start_frame = interpolate_frame(frame_nodes, segment_start_md_m)
        center_frame = interpolate_frame(frame_nodes, center_md_m)
        end_frame = interpolate_frame(frame_nodes, segment_end_md_m)
        start_tangent = _tangent_from_angles(start_point.inclination_rad, start_point.azimuth_rad)
        end_tangent = _tangent_from_angles(end_point.inclination_rad, end_point.azimuth_rad)
        curvature_rad_per_m = (
            0.0
            if segment_length_m <= 0.0
            else _angle_between(start_tangent, end_tangent) / segment_length_m
        )
        curvature_vector = (
            0.0
            if segment_length_m <= 0.0
            else (end_frame.tangent_north_east_tvd[0] - start_frame.tangent_north_east_tvd[0]) / segment_length_m,
            0.0
            if segment_length_m <= 0.0
            else (end_frame.tangent_north_east_tvd[1] - start_frame.tangent_north_east_tvd[1]) / segment_length_m,
            0.0
            if segment_length_m <= 0.0
            else (end_frame.tangent_north_east_tvd[2] - start_frame.tangent_north_east_tvd[2]) / segment_length_m,
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
                curvature_normal_component_rad_per_m=_dot3(
                    curvature_vector, center_frame.normal_north_east_tvd
                ),
                curvature_binormal_component_rad_per_m=_dot3(
                    curvature_vector, center_frame.binormal_north_east_tvd
                ),
                frame_rotation_change_rad=center_frame.frame_rotation_change_rad,
                tvd_m=center_point.tvd_m,
                northing_m=center_point.northing_m,
                easting_m=center_point.easting_m,
                tangent_north_east_tvd=center_frame.tangent_north_east_tvd,
                normal_north_east_tvd=center_frame.normal_north_east_tvd,
                binormal_north_east_tvd=center_frame.binormal_north_east_tvd,
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


def compute_buoyant_axial_load_profile(
    segments: list[DiscretizedSegmentModel],
) -> tuple[list[float], float]:
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

    return axial_loads_n, load_below_n


def _evaluate_vector_lateral_equilibrium(
    segment: DiscretizedSegmentModel,
    effective_axial_load_n: float,
) -> LateralEquilibriumStateModel:
    segment_length_m = max(segment.segment_length_m, 1.0e-6)
    gravity_vector_north_east_tvd = (0.0, 0.0, segment.effective_line_weight_n_per_m)
    gravity_lateral_load_n_b_n_per_m = (
        _dot3(gravity_vector_north_east_tvd, segment.normal_north_east_tvd),
        _dot3(gravity_vector_north_east_tvd, segment.binormal_north_east_tvd),
    )
    curvature_lateral_load_n_b_n_per_m = (
        max(effective_axial_load_n, 0.0) * segment.curvature_normal_component_rad_per_m,
        max(effective_axial_load_n, 0.0) * segment.curvature_binormal_component_rad_per_m,
    )
    equivalent_lateral_load_n_b_n_per_m = (
        gravity_lateral_load_n_b_n_per_m[0] + curvature_lateral_load_n_b_n_per_m[0],
        gravity_lateral_load_n_b_n_per_m[1] + curvature_lateral_load_n_b_n_per_m[1],
    )
    equivalent_lateral_force_n_b = (
        equivalent_lateral_load_n_b_n_per_m[0] * segment.segment_length_m,
        equivalent_lateral_load_n_b_n_per_m[1] * segment.segment_length_m,
    )
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
    free_displacement_n_b_m = (
        0.0
        if structural_lateral_stiffness_n_per_m <= 0.0
        else equivalent_lateral_force_n_b[0] / structural_lateral_stiffness_n_per_m,
        0.0
        if structural_lateral_stiffness_n_per_m <= 0.0
        else equivalent_lateral_force_n_b[1] / structural_lateral_stiffness_n_per_m,
    )
    return LateralEquilibriumStateModel(
        gravity_lateral_load_n_b_n_per_m=gravity_lateral_load_n_b_n_per_m,
        curvature_lateral_load_n_b_n_per_m=curvature_lateral_load_n_b_n_per_m,
        equivalent_lateral_load_n_b_n_per_m=equivalent_lateral_load_n_b_n_per_m,
        equivalent_lateral_force_n_b=equivalent_lateral_force_n_b,
        equivalent_lateral_load_magnitude_n_per_m=_norm2(equivalent_lateral_load_n_b_n_per_m),
        equivalent_lateral_force_magnitude_n=_norm2(equivalent_lateral_force_n_b),
        bending_lateral_stiffness_n_per_m=bending_lateral_stiffness_n_per_m,
        axial_tension_lateral_stiffness_n_per_m=axial_tension_lateral_stiffness_n_per_m,
        structural_lateral_stiffness_n_per_m=structural_lateral_stiffness_n_per_m,
        free_displacement_n_b_m=free_displacement_n_b_m,
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

    nearby_centralizer_count = 0
    support_present = False
    centering_stiffness_n_per_m = 0.0
    support_contact_clearance_m = 0.0
    for placement in placements:
        proximity_weight = placement_proximity_weight(
            placement,
            segment.measured_depth_center_m,
            segment.segment_length_m,
        )
        if proximity_weight <= 0.0:
            continue

        nearby_centralizer_count += 1
        support_present = True
        support_outer_diameter_m = max(
            support_outer_diameter_m,
            centralizer_effective_contact_diameter_m(placement),
        )
        centering_stiffness_n_per_m += (
            proximity_weight * equivalent_bow_support_stiffness_n_per_m(placement, hole_radius_m)
        )
        placement_support_contact_clearance_m = bow_contact_onset_clearance_m(
            placement,
            hole_radius_m,
        )
        support_contact_clearance_m = (
            placement_support_contact_clearance_m
            if nearby_centralizer_count == 1
            else min(support_contact_clearance_m, placement_support_contact_clearance_m)
        )

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
    effective_axial_loads_n: list[float] | None = None,
) -> tuple[
    list[DiscretizedSegmentModel],
    list[CentralizerPlacementModel],
    list[GlobalNodeInputModel],
]:
    segments, placements = discretize_case(loaded_case)
    axial_loads_n, _ = compute_buoyant_axial_load_profile(segments)
    if effective_axial_loads_n is not None:
        if len(effective_axial_loads_n) != len(segments):
            raise ValueError("Mechanical solver axial profile must match the discretized segment count.")
        axial_loads_n = list(effective_axial_loads_n)

    nodes: list[GlobalNodeInputModel] = []
    for segment, effective_axial_load_n in zip(segments, axial_loads_n, strict=True):
        lateral_state = _evaluate_vector_lateral_equilibrium(segment, effective_axial_load_n)
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
                effective_axial_load_n=effective_axial_load_n,
                bending_stiffness_n_m2=segment.bending_stiffness_n_m2,
                bending_moment_n_m=segment.bending_stiffness_n_m2 * segment.curvature_rad_per_m,
                bending_stress_pa=(
                    segment.section.young_modulus_pa
                    * segment.curvature_rad_per_m
                    * segment.section.outer_radius_m
                ),
                bending_strain_estimate=segment.curvature_rad_per_m * segment.section.outer_radius_m,
                gravity_lateral_load_n_b_n_per_m=lateral_state.gravity_lateral_load_n_b_n_per_m,
                curvature_lateral_load_n_b_n_per_m=lateral_state.curvature_lateral_load_n_b_n_per_m,
                equivalent_lateral_load_n_b_n_per_m=lateral_state.equivalent_lateral_load_n_b_n_per_m,
                equivalent_lateral_force_n_b=lateral_state.equivalent_lateral_force_n_b,
                equivalent_lateral_load_magnitude_n_per_m=lateral_state.equivalent_lateral_load_magnitude_n_per_m,
                equivalent_lateral_force_magnitude_n=lateral_state.equivalent_lateral_force_magnitude_n,
                bending_lateral_stiffness_n_per_m=lateral_state.bending_lateral_stiffness_n_per_m,
                axial_tension_lateral_stiffness_n_per_m=lateral_state.axial_tension_lateral_stiffness_n_per_m,
                structural_lateral_stiffness_n_per_m=lateral_state.structural_lateral_stiffness_n_per_m,
                centralizer_centering_stiffness_n_per_m=support_effect.centering_stiffness_n_per_m,
                support_contact_penalty_n_per_m=support_effect.support_contact_penalty_n_per_m,
                body_contact_penalty_n_per_m=(
                    2.0 * loaded_case.contact_penalty_scale * max(base_lateral_stiffness_n_per_m, 1.0)
                ),
                support_outer_diameter_m=support_effect.support_outer_diameter_m,
                pipe_body_clearance_m=support_effect.pipe_body_clearance_m,
                support_contact_clearance_m=support_effect.support_contact_clearance_m,
                nearby_centralizer_count=support_effect.nearby_centralizer_count,
                free_displacement_n_b_m=lateral_state.free_displacement_n_b_m,
                section=segment.section,
            )
        )

    return segments, placements, nodes


def assemble_global_linear_system(
    nodes: list[GlobalNodeInputModel],
    contact_states: list[GlobalContactStateModel],
) -> tuple[np.ndarray, np.ndarray]:
    if len(nodes) != len(contact_states):
        raise ValueError("Vector global assembly requires one contact state per node.")

    dof_count = 2 * len(nodes)
    stiffness_matrix = np.zeros((dof_count, dof_count), dtype=float)
    load_vector = np.zeros(dof_count, dtype=float)

    def dof_index(node_index: int, component_index: int) -> int:
        return (2 * node_index) + component_index

    for node_index, node in enumerate(nodes):
        state = contact_states[node_index]
        for component_index in range(2):
            component_dof = dof_index(node_index, component_index)
            load_vector[component_dof] += node.equivalent_lateral_force_n_b[component_index]
            stiffness_matrix[component_dof][component_dof] += node.centralizer_centering_stiffness_n_per_m

        def add_contact_penalty(penalty_n_per_m: float, clearance_m: float) -> None:
            for row_component in range(2):
                for column_component in range(2):
                    stiffness_matrix[dof_index(node_index, row_component)][
                        dof_index(node_index, column_component)
                    ] += (
                        penalty_n_per_m
                        * state.contact_direction_n_b[row_component]
                        * state.contact_direction_n_b[column_component]
                    )
                load_vector[dof_index(node_index, row_component)] += (
                    penalty_n_per_m * clearance_m * state.contact_direction_n_b[row_component]
                )

        if state.support_contact_active:
            add_contact_penalty(node.support_contact_penalty_n_per_m, node.support_contact_clearance_m)
        if state.pipe_body_contact_active:
            add_contact_penalty(node.body_contact_penalty_n_per_m, node.pipe_body_clearance_m)

    for node_index in range(1, len(nodes)):
        spacing_m = max(
            nodes[node_index].measured_depth_m - nodes[node_index - 1].measured_depth_m,
            1.0e-6,
        )
        effective_axial_load_n = max(
            0.0,
            0.5 * (nodes[node_index].effective_axial_load_n + nodes[node_index - 1].effective_axial_load_n),
        )
        axial_coefficient = effective_axial_load_n / spacing_m

        for component_index in range(2):
            lower_dof = dof_index(node_index - 1, component_index)
            upper_dof = dof_index(node_index, component_index)
            stiffness_matrix[lower_dof][lower_dof] += axial_coefficient
            stiffness_matrix[lower_dof][upper_dof] -= axial_coefficient
            stiffness_matrix[upper_dof][lower_dof] -= axial_coefficient
            stiffness_matrix[upper_dof][upper_dof] += axial_coefficient

    for node_index in range(1, len(nodes) - 1):
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

        for component_index in range(2):
            lower_dof = dof_index(node_index - 1, component_index)
            middle_dof = dof_index(node_index, component_index)
            upper_dof = dof_index(node_index + 1, component_index)
            stiffness_matrix[lower_dof][lower_dof] += bending_coefficient
            stiffness_matrix[lower_dof][middle_dof] -= 2.0 * bending_coefficient
            stiffness_matrix[lower_dof][upper_dof] += bending_coefficient
            stiffness_matrix[middle_dof][lower_dof] -= 2.0 * bending_coefficient
            stiffness_matrix[middle_dof][middle_dof] += 4.0 * bending_coefficient
            stiffness_matrix[middle_dof][upper_dof] -= 2.0 * bending_coefficient
            stiffness_matrix[upper_dof][lower_dof] += bending_coefficient
            stiffness_matrix[upper_dof][middle_dof] -= 2.0 * bending_coefficient
            stiffness_matrix[upper_dof][upper_dof] += bending_coefficient

    def apply_centered_end_boundary(node_index: int) -> None:
        for component_index in range(2):
            boundary_dof = dof_index(node_index, component_index)
            for row_index in range(dof_count):
                stiffness_matrix[row_index][boundary_dof] = 0.0
            for column_index in range(dof_count):
                stiffness_matrix[boundary_dof][column_index] = 0.0
            stiffness_matrix[boundary_dof][boundary_dof] = 1.0
            load_vector[boundary_dof] = 0.0

    if len(nodes) == 1:
        apply_centered_end_boundary(0)
    elif len(nodes) > 1:
        apply_centered_end_boundary(0)
        apply_centered_end_boundary(len(nodes) - 1)

    return stiffness_matrix, load_vector


def _solve_dense_linear_system(
    matrix: np.ndarray | list[list[float]],
    rhs: np.ndarray | list[float],
) -> list[float]:
    try:
        solution = np.linalg.solve(
            np.asarray(matrix, dtype=float),
            np.asarray(rhs, dtype=float),
        )
    except np.linalg.LinAlgError as exc:
        raise ValueError("NumPy global solver encountered a singular linear system.") from exc
    return solution.tolist()
