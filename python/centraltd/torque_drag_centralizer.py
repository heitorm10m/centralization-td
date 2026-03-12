from __future__ import annotations

from dataclasses import dataclass
import math


def _norm2(vector: tuple[float, float]) -> float:
    return math.sqrt((vector[0] ** 2) + (vector[1] ** 2))


def _normalize2(vector: tuple[float, float]) -> tuple[float, float]:
    magnitude = _norm2(vector)
    if magnitude <= 1.0e-12:
        return (0.0, 0.0)
    return (vector[0] / magnitude, vector[1] / magnitude)


def _rotate_ccw_90(vector: tuple[float, float]) -> tuple[float, float]:
    return (-vector[1], vector[0])


@dataclass(slots=True)
class CentralizerTorqueContributionModel:
    tangential_direction_n_b: tuple[float, float]
    tangential_friction_vector_n_b: tuple[float, float]
    tangential_friction_vector_magnitude_n: float
    axial_friction_n: float
    tangential_friction_n: float
    torque_increment_n_m: float
    effective_contact_radius_m: float
    active: bool
    status: str


def evaluate_centralizer_torque_contribution(
    bow_segment_result: object,
) -> CentralizerTorqueContributionModel:
    axial_friction_n = 0.0
    tangential_friction_n = 0.0
    torque_increment_n_m = 0.0
    effective_contact_radius_m = 0.0
    tangential_friction_n_vec = 0.0
    tangential_friction_b_vec = 0.0
    active = False

    for placement_result in bow_segment_result.placement_resultant_details:
        if placement_result.bow_resultant_magnitude_n <= 0.0:
            continue

        radial_direction_n_b = _normalize2(placement_result.bow_resultant_vector_n_b)
        tangential_direction_n_b = _rotate_ccw_90(radial_direction_n_b)
        local_axial_friction_n = (
            placement_result.axial_force_ratio * placement_result.bow_resultant_magnitude_n
        )
        local_tangential_friction_n = (
            placement_result.tangential_force_ratio * placement_result.bow_resultant_magnitude_n
        )
        tangential_friction_n_vec += local_tangential_friction_n * tangential_direction_n_b[0]
        tangential_friction_b_vec += local_tangential_friction_n * tangential_direction_n_b[1]
        axial_friction_n += local_axial_friction_n
        tangential_friction_n += local_tangential_friction_n
        torque_increment_n_m += (
            local_tangential_friction_n * placement_result.effective_contact_radius_m
        )
        effective_contact_radius_m = max(
            effective_contact_radius_m,
            placement_result.effective_contact_radius_m,
        )
        active = True

    tangential_friction_vector_n_b = (
        tangential_friction_n_vec,
        tangential_friction_b_vec,
    )
    tangential_friction_vector_magnitude_n = _norm2(tangential_friction_vector_n_b)
    if tangential_friction_vector_magnitude_n > 1.0e-12:
        tangential_direction_n_b = (
            tangential_friction_vector_n_b[0] / tangential_friction_vector_magnitude_n,
            tangential_friction_vector_n_b[1] / tangential_friction_vector_magnitude_n,
        )
    elif bow_segment_result.bow_resultant_magnitude_n > 1.0e-12:
        tangential_direction_n_b = _rotate_ccw_90(
            _normalize2(bow_segment_result.bow_resultant_vector_n_b)
        )
    else:
        tangential_direction_n_b = (0.0, 0.0)

    return CentralizerTorqueContributionModel(
        tangential_direction_n_b=tangential_direction_n_b,
        tangential_friction_vector_n_b=tangential_friction_vector_n_b,
        tangential_friction_vector_magnitude_n=tangential_friction_vector_magnitude_n,
        axial_friction_n=axial_friction_n,
        tangential_friction_n=tangential_friction_n,
        torque_increment_n_m=torque_increment_n_m,
        effective_contact_radius_m=effective_contact_radius_m,
        active=active,
        status=(
            "phase11-reduced-vector-tangential-centralizer-torque" if active else "inactive"
        ),
    )
