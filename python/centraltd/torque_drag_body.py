from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class BodyTorqueDragContributionModel:
    axial_friction_n: float
    tangential_friction_n: float
    torque_increment_n_m: float
    contact_radius_m: float


def evaluate_body_torque_drag_contribution(
    segment: object,
    section: object,
) -> BodyTorqueDragContributionModel:
    axial_friction_n = max(0.0, section.friction_coefficient) * max(
        0.0,
        segment.body_normal_reaction_estimate_n,
    )
    contact_radius_m = max(0.0, 0.5 * section.outer_diameter_m)
    return BodyTorqueDragContributionModel(
        axial_friction_n=axial_friction_n,
        tangential_friction_n=axial_friction_n,
        torque_increment_n_m=axial_friction_n * contact_radius_m,
        contact_radius_m=contact_radius_m,
    )
