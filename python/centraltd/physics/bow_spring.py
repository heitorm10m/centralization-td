"""
Bow-spring centralizer physics.

Reference: Dao et al. 2023, Geoenergy Science and Engineering 222, 211457

Equations implemented in this module:
  Eq. 1  — Restoring force as vector sum of blade forces
  Eq. 4  — Blade force power law Fi = kblade * delta_i^p
  Eq. 5  — Blade deflection geometry
  Eq. 22 — Blade position vector (PENDING)
  Eq. 23 — Blade radial contact force two-regime (PENDING)
  Eq. 24 — Smoothed blade radial contact force (PENDING)
  Eq. 25 — Axial friction scalar sum (PENDING)
  Eq. 26 — Tangential friction vector resultant (PENDING)
  Eq. 27 — Tangential friction force direction (PENDING)
  Eq. 28 — Individual blade tangential friction (PENDING)
  Eq. 29 — Full nodal force and moment per blade (PENDING)

See CHANGELOG.md for implementation status of each equation.
See DECISIONS.md for decisions about this module.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


def _norm2(vector: tuple[float, float]) -> float:
    return math.sqrt((vector[0] ** 2) + (vector[1] ** 2))


@dataclass(slots=True)
class BowForceDetailModel:
    source_name: str
    placement_measured_depth_m: float
    bow_index: int
    angle_rad: float
    direction_n_b: tuple[float, float]
    deflection_m: float
    force_magnitude_n: float
    force_vector_n_b: tuple[float, float]


@dataclass(slots=True)
class CentralizerPlacementResultantDetailModel:
    source_name: str
    placement_measured_depth_m: float
    effective_contact_radius_m: float
    axial_force_ratio: float
    tangential_force_ratio: float
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float


@dataclass(slots=True)
class BowSpringSegmentResultModel:
    support_outer_diameter_m: float
    support_contact_clearance_m: float
    equivalent_centering_stiffness_n_per_m: float
    nearby_centralizer_count: int
    support_present: bool
    bow_force_details: list[BowForceDetailModel]
    placement_resultant_details: list[CentralizerPlacementResultantDetailModel]
    bow_resultant_vector_n_b: tuple[float, float]
    bow_resultant_magnitude_n: float
    centralizer_axial_friction_n: float
    centralizer_tangential_friction_n: float
    centralizer_torque_increment_n_m: float
    effective_contact_radius_m: float


def placement_proximity_weight(
    placement: Any,
    segment_center_md_m: float,
    segment_length_m: float,
) -> float:
    half_influence_length_m = max(0.5 * placement.influence_length_m, 0.5 * segment_length_m)
    if half_influence_length_m <= 0.0:
        return 0.0
    distance_to_segment_center_m = abs(placement.measured_depth_m - segment_center_md_m)
    if distance_to_segment_center_m > half_influence_length_m:
        return 0.0
    return max(0.0, 1.0 - (distance_to_segment_center_m / half_influence_length_m))


def centralizer_effective_contact_diameter_m(placement: Any) -> float:
    diameter_m = placement.support_outer_diameter_m
    if placement.min_contact_diameter_m is not None:
        diameter_m = max(diameter_m, placement.min_contact_diameter_m)
    if placement.max_contact_diameter_m is not None:
        diameter_m = min(diameter_m, placement.max_contact_diameter_m)
    return diameter_m


def centralizer_support_clearance_m(placement: Any, hole_radius_m: float) -> float:
    return max(0.0, hole_radius_m - (0.5 * centralizer_effective_contact_diameter_m(placement)))


def bow_contact_onset_clearance_m(placement: Any, hole_radius_m: float) -> float:
    return placement.inner_clearance_to_pipe_m + centralizer_support_clearance_m(
        placement,
        hole_radius_m,
    )


def build_bow_directions(placement: Any) -> list[tuple[int, float, tuple[float, float]]]:
    directions: list[tuple[int, float, tuple[float, float]]] = []
    angle_reference_rad = math.radians(placement.angular_orientation_reference_deg)
    angle_increment_rad = (2.0 * math.pi) / float(placement.number_of_bows)
    for bow_index in range(placement.number_of_bows):
        angle_rad = angle_reference_rad + (float(bow_index) * angle_increment_rad)
        directions.append((bow_index, angle_rad, (math.cos(angle_rad), math.sin(angle_rad))))
    return directions


def bow_projection_m(
    eccentricity_n_b_m: tuple[float, float],
    direction_n_b: tuple[float, float],
) -> float:
    return (eccentricity_n_b_m[0] * direction_n_b[0]) + (eccentricity_n_b_m[1] * direction_n_b[1])


def bow_deflection_m(
    placement: Any,
    hole_radius_m: float,
    eccentricity_n_b_m: tuple[float, float],
    direction_n_b: tuple[float, float],
) -> float:
    return max(
        0.0,
        bow_projection_m(eccentricity_n_b_m, direction_n_b)
        - bow_contact_onset_clearance_m(placement, hole_radius_m),
    )


def bow_reference_deflection_m(placement: Any, hole_radius_m: float) -> float:
    return max(
        bow_contact_onset_clearance_m(placement, hole_radius_m),
        max(0.05 * centralizer_effective_contact_diameter_m(placement), 1.0e-4),
    )


def resolved_blade_power_law_k(placement: Any, hole_radius_m: float) -> float:
    if placement.blade_power_law_k is not None:
        return placement.blade_power_law_k
    loaded_bow_count = max(1.0, math.ceil(0.5 * float(placement.number_of_bows)))
    reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m)
    return placement.nominal_restoring_force_n / (
        loaded_bow_count * (reference_deflection_m ** placement.blade_power_law_p)
    )


def bow_force_magnitude_n(
    placement: Any,
    hole_radius_m: float,
    deflection_m: float,
) -> float:
    if deflection_m <= 0.0:
        return 0.0
    return resolved_blade_power_law_k(placement, hole_radius_m) * (
        deflection_m ** placement.blade_power_law_p
    )


def equivalent_bow_support_stiffness_n_per_m(placement: Any, hole_radius_m: float) -> float:
    loaded_bow_count = max(1.0, math.ceil(0.5 * float(placement.number_of_bows)))
    reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m)
    tangent_stiffness_per_bow_n_per_m = (
        resolved_blade_power_law_k(placement, hole_radius_m)
        * placement.blade_power_law_p
        * (reference_deflection_m ** (placement.blade_power_law_p - 1.0))
    )
    return loaded_bow_count * tangent_stiffness_per_bow_n_per_m


def centralizer_running_force_ratio(placement: Any) -> float:
    if placement.nominal_restoring_force_n <= 0.0:
        return 0.0
    return max(0.0, placement.nominal_running_force_n / placement.nominal_restoring_force_n)


def centralizer_axial_force_ratio(placement: Any) -> float:
    if getattr(placement, "axial_force_ratio", None) is not None:
        return max(0.0, float(placement.axial_force_ratio))
    return centralizer_running_force_ratio(placement)


def centralizer_tangential_force_ratio(placement: Any) -> float:
    if getattr(placement, "tangential_force_ratio", None) is not None:
        return max(0.0, float(placement.tangential_force_ratio))
    return centralizer_running_force_ratio(placement)


def evaluate_bow_spring_segment_result(
    segment: Any,
    placements: list[Any],
    eccentricity_n_b_m: tuple[float, float],
) -> BowSpringSegmentResultModel:
    result = BowSpringSegmentResultModel(
        support_outer_diameter_m=segment.section.outer_diameter_m,
        support_contact_clearance_m=0.0,
        equivalent_centering_stiffness_n_per_m=0.0,
        nearby_centralizer_count=0,
        support_present=False,
        bow_force_details=[],
        placement_resultant_details=[],
        bow_resultant_vector_n_b=(0.0, 0.0),
        bow_resultant_magnitude_n=0.0,
        centralizer_axial_friction_n=0.0,
        centralizer_tangential_friction_n=0.0,
        centralizer_torque_increment_n_m=0.0,
        effective_contact_radius_m=0.0,
    )
    if segment.reference_hole_diameter_m <= 0.0:
        return result

    hole_radius_m = 0.5 * segment.reference_hole_diameter_m
    bow_resultant_n = 0.0
    bow_resultant_b = 0.0

    for placement in placements:
        proximity_weight = placement_proximity_weight(
            placement,
            segment.measured_depth_center_m,
            segment.segment_length_m,
        )
        if proximity_weight <= 0.0:
            continue

        result.nearby_centralizer_count += 1
        result.support_present = True
        result.support_outer_diameter_m = max(
            result.support_outer_diameter_m,
            centralizer_effective_contact_diameter_m(placement),
        )
        placement_support_contact_clearance_m = bow_contact_onset_clearance_m(
            placement,
            hole_radius_m,
        )
        result.support_contact_clearance_m = (
            placement_support_contact_clearance_m
            if result.nearby_centralizer_count == 1
            else min(result.support_contact_clearance_m, placement_support_contact_clearance_m)
        )
        result.equivalent_centering_stiffness_n_per_m += (
            proximity_weight * equivalent_bow_support_stiffness_n_per_m(placement, hole_radius_m)
        )

        placement_resultant_n = 0.0
        placement_resultant_b = 0.0
        for bow_index, angle_rad, direction_n_b in build_bow_directions(placement):
            deflection_m = bow_deflection_m(
                placement,
                hole_radius_m,
                eccentricity_n_b_m,
                direction_n_b,
            )
            force_magnitude_n = proximity_weight * bow_force_magnitude_n(
                placement,
                hole_radius_m,
                deflection_m,
            )
            force_vector_n_b = (
                force_magnitude_n * direction_n_b[0],
                force_magnitude_n * direction_n_b[1],
            )
            placement_resultant_n += force_vector_n_b[0]
            placement_resultant_b += force_vector_n_b[1]
            result.bow_force_details.append(
                BowForceDetailModel(
                    source_name=placement.source_name,
                    placement_measured_depth_m=placement.measured_depth_m,
                    bow_index=bow_index,
                    angle_rad=angle_rad,
                    direction_n_b=direction_n_b,
                    deflection_m=deflection_m,
                    force_magnitude_n=force_magnitude_n,
                    force_vector_n_b=force_vector_n_b,
                )
            )

        placement_resultant_magnitude_n = _norm2((placement_resultant_n, placement_resultant_b))
        placement_axial_force_ratio = centralizer_axial_force_ratio(placement)
        placement_tangential_force_ratio = centralizer_tangential_force_ratio(placement)
        placement_effective_contact_radius_m = 0.5 * centralizer_effective_contact_diameter_m(
            placement
        )
        result.placement_resultant_details.append(
            CentralizerPlacementResultantDetailModel(
                source_name=placement.source_name,
                placement_measured_depth_m=placement.measured_depth_m,
                effective_contact_radius_m=placement_effective_contact_radius_m,
                axial_force_ratio=placement_axial_force_ratio,
                tangential_force_ratio=placement_tangential_force_ratio,
                bow_resultant_vector_n_b=(placement_resultant_n, placement_resultant_b),
                bow_resultant_magnitude_n=placement_resultant_magnitude_n,
            )
        )
        bow_resultant_n += placement_resultant_n
        bow_resultant_b += placement_resultant_b
        result.centralizer_axial_friction_n += (
            placement_axial_force_ratio * placement_resultant_magnitude_n
        )
        result.centralizer_tangential_friction_n += (
            placement_tangential_force_ratio * placement_resultant_magnitude_n
        )
        result.centralizer_torque_increment_n_m += (
            placement_tangential_force_ratio
            * placement_resultant_magnitude_n
            * placement_effective_contact_radius_m
        )
        result.effective_contact_radius_m = max(
            result.effective_contact_radius_m,
            placement_effective_contact_radius_m,
        )

    result.bow_resultant_vector_n_b = (bow_resultant_n, bow_resultant_b)
    result.bow_resultant_magnitude_n = _norm2(result.bow_resultant_vector_n_b)
    return result
