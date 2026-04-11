"""
Contact force physics for string body and borehole interaction.

Reference: Dao et al. 2023, Geoenergy Science and Engineering 222, 211457

Equations implemented in this module:
  Eq. 15 — Contact force decomposition (PENDING)
  Eq. 16 — Normal contact force with Heaviside (PENDING)
  Eq. 17 — Smooth Heaviside function h(x) (PENDING)
  Eq. 18 — Friction force components (PENDING)
  Eq. 19 — Directional friction coefficient split (PENDING)
  Eq. 20 — Velocity-dependent friction coefficient (PENDING)
  Eq. 21 — Contact force and moment in local frame (PENDING)

See CHANGELOG.md for implementation status of each equation.
See DECISIONS.md for decisions about this module.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .constants import DTYPE
from .fem import (
    GlobalContactStateModel,
    GlobalNodeInputModel,
    _clamp,
    _norm2,
    _normalize2,
    _solve_dense_linear_system,
    assemble_global_linear_system,
)


def smooth_heaviside(
    x: float | np.ndarray,
    lambda_contact: float,
) -> float | np.ndarray:
    """
    Eq. 17 from Dao et al. 2023.

    h(x) = (1 - 1 / (1 + (x / lambda)^2)) * H(x)

    The implementation preserves the paper requirements:
    - h(x) = 0 exactly for x <= 0
    - h(lambda) = 0.5
    """
    if lambda_contact <= 0.0:
        raise ValueError("Eq. 17 requires a strictly positive lambda_contact.")

    x_array = np.asarray(x, dtype=DTYPE)
    positive_mask = x_array > 0.0

    result = np.zeros_like(x_array, dtype=DTYPE)
    if np.any(positive_mask):
        ratio_squared = np.square(x_array[positive_mask] / lambda_contact, dtype=DTYPE)
        result[positive_mask] = 1.0 - (1.0 / (1.0 + ratio_squared))

    if np.isscalar(x):
        return float(result)
    return result


@dataclass(slots=True)
class GlobalNodeSolutionModel:
    measured_depth_m: float
    lateral_displacement_normal_m: float
    lateral_displacement_binormal_m: float
    eccentricity_estimate_m: float
    eccentricity_ratio: float
    standoff_estimate: float
    contact_direction_n_b: tuple[float, float]
    support_normal_reaction_vector_n_b: tuple[float, float]
    body_normal_reaction_vector_n_b: tuple[float, float]
    normal_reaction_vector_n_b: tuple[float, float]
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


def _select_contact_direction(
    displacement_n_b_m: tuple[float, float],
    equivalent_force_n_b: tuple[float, float],
) -> tuple[float, float]:
    if _norm2(displacement_n_b_m) > 1.0e-12:
        return _normalize2(displacement_n_b_m)
    if _norm2(equivalent_force_n_b) > 1.0e-12:
        return _normalize2(equivalent_force_n_b)
    return (1.0, 0.0)


def run_global_lateral_solver(
    nodes: list[GlobalNodeInputModel],
    *,
    max_iterations: int,
) -> GlobalSolverResultModel:
    contact_states = [GlobalContactStateModel() for _ in nodes]
    displacements_n_b_m = [(0.0, 0.0) for _ in nodes]
    final_update_norm_m = 0.0
    iteration_count = 0

    for iteration_index in range(max_iterations):
        stiffness_matrix, load_vector = assemble_global_linear_system(nodes, contact_states)
        updated_solution = _solve_dense_linear_system(stiffness_matrix, load_vector)

        final_update_norm_m = 0.0
        contact_state_changed = False
        contact_direction_changed = False
        for node_index, node in enumerate(nodes):
            updated_displacement_n_b_m = (
                updated_solution[2 * node_index],
                updated_solution[(2 * node_index) + 1],
            )
            displacement_update = (
                updated_displacement_n_b_m[0] - displacements_n_b_m[node_index][0],
                updated_displacement_n_b_m[1] - displacements_n_b_m[node_index][1],
            )
            final_update_norm_m = max(final_update_norm_m, _norm2(displacement_update))
            eccentricity_magnitude_m = _norm2(updated_displacement_n_b_m)
            contact_direction_n_b = _select_contact_direction(
                updated_displacement_n_b_m,
                node.equivalent_lateral_force_n_b,
            )
            support_contact_active = (
                node.support_contact_penalty_n_per_m > 0.0
                and eccentricity_magnitude_m > node.support_contact_clearance_m
            )
            pipe_body_contact_active = eccentricity_magnitude_m > node.pipe_body_clearance_m
            direction_delta = (
                contact_direction_n_b[0] - contact_states[node_index].contact_direction_n_b[0],
                contact_direction_n_b[1] - contact_states[node_index].contact_direction_n_b[1],
            )

            if (
                support_contact_active != contact_states[node_index].support_contact_active
                or pipe_body_contact_active != contact_states[node_index].pipe_body_contact_active
            ):
                contact_state_changed = True
            if _norm2(direction_delta) > 1.0e-6:
                contact_direction_changed = True

            contact_states[node_index].support_contact_active = support_contact_active
            contact_states[node_index].pipe_body_contact_active = pipe_body_contact_active
            contact_states[node_index].contact_direction_n_b = contact_direction_n_b
            displacements_n_b_m[node_index] = updated_displacement_n_b_m

        iteration_count = iteration_index + 1
        if not contact_state_changed and not contact_direction_changed and final_update_norm_m <= 1.0e-8:
            break

    node_solutions: list[GlobalNodeSolutionModel] = []
    for node_index, node in enumerate(nodes):
        displacement_n_b_m = displacements_n_b_m[node_index]
        eccentricity_estimate_m = _norm2(displacement_n_b_m)
        contact_direction_n_b = _select_contact_direction(
            displacement_n_b_m,
            node.equivalent_lateral_force_n_b,
        )
        support_penetration_m = (
            0.0
            if node.support_contact_penalty_n_per_m <= 0.0
            else max(0.0, eccentricity_estimate_m - node.support_contact_clearance_m)
        )
        body_penetration_m = max(0.0, eccentricity_estimate_m - node.pipe_body_clearance_m)
        support_normal_reaction_estimate_n = (
            node.support_contact_penalty_n_per_m * support_penetration_m
        )
        body_normal_reaction_estimate_n = node.body_contact_penalty_n_per_m * body_penetration_m
        support_normal_reaction_vector_n_b = (
            support_normal_reaction_estimate_n * contact_direction_n_b[0],
            support_normal_reaction_estimate_n * contact_direction_n_b[1],
        )
        body_normal_reaction_vector_n_b = (
            body_normal_reaction_estimate_n * contact_direction_n_b[0],
            body_normal_reaction_estimate_n * contact_direction_n_b[1],
        )
        normal_reaction_vector_n_b = (
            support_normal_reaction_vector_n_b[0] + body_normal_reaction_vector_n_b[0],
            support_normal_reaction_vector_n_b[1] + body_normal_reaction_vector_n_b[1],
        )
        normal_reaction_estimate_n = _norm2(normal_reaction_vector_n_b)
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
                lateral_displacement_normal_m=displacement_n_b_m[0],
                lateral_displacement_binormal_m=displacement_n_b_m[1],
                eccentricity_estimate_m=eccentricity_estimate_m,
                eccentricity_ratio=eccentricity_ratio,
                standoff_estimate=standoff_estimate,
                contact_direction_n_b=contact_direction_n_b,
                support_normal_reaction_vector_n_b=support_normal_reaction_vector_n_b,
                body_normal_reaction_vector_n_b=body_normal_reaction_vector_n_b,
                normal_reaction_vector_n_b=normal_reaction_vector_n_b,
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
