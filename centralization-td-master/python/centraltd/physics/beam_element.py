"""
6-DOF Euler-Bernoulli beam element utilities.

Reference:
  Dao et al. 2023, Section 3.1, Eq. 12
  Lalanne & Ferraris (1998)

Implementation note:
  The paper requires two-node beam elements with six DOFs per node and
  global assembly of K. The visible repository reference does not include
  the full 12x12 element matrix, so this isolated module uses the standard
  Euler-Bernoulli beam assumption cited by the user for axial, torsional,
  and uncoupled bending contributions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np

from .constants import DTYPE


@dataclass(slots=True)
class BeamElementDefinition:
    node_start_index: int
    node_end_index: int
    length_m: float
    young_modulus_pa: float
    poisson_ratio: float
    outer_radius_m: float
    inner_radius_m: float
    eu: tuple[float, float, float]
    ev: tuple[float, float, float]
    ew: tuple[float, float, float]


def _get_value(source: Any, name: str) -> Any:
    if isinstance(source, dict):
        return source[name]
    return getattr(source, name)


def _normalize(vector: Sequence[float]) -> np.ndarray:
    array = np.asarray(vector, dtype=DTYPE)
    norm = np.linalg.norm(array)
    if norm <= 1.0e-12:
        raise ValueError("Beam basis vectors must be non-zero.")
    return array / norm


def _orthonormal_rotation(
    eu: Sequence[float],
    ev: Sequence[float],
    ew: Sequence[float],
) -> np.ndarray:
    ew_array = _normalize(ew)
    eu_array = _normalize(np.asarray(eu, dtype=DTYPE) - (np.dot(eu, ew_array) * ew_array))
    ev_array = np.cross(ew_array, eu_array)
    ev_norm = np.linalg.norm(ev_array)
    if ev_norm <= 1.0e-12:
        ev_array = _normalize(ev)
        ev_array = _normalize(ev_array - (np.dot(ev_array, ew_array) * ew_array))
    else:
        ev_array = ev_array / ev_norm
    eu_array = _normalize(np.cross(ev_array, ew_array))
    return np.column_stack((eu_array, ev_array, ew_array)).astype(DTYPE, copy=False)


def stiffness_matrix_local(
    *,
    length_m: float,
    young_modulus_pa: float,
    poisson_ratio: float,
    outer_radius_m: float,
    inner_radius_m: float,
) -> np.ndarray:
    if length_m <= 0.0:
        raise ValueError("Beam element length_m must be positive.")
    if young_modulus_pa <= 0.0:
        raise ValueError("Beam element young_modulus_pa must be positive.")
    if inner_radius_m < 0.0 or outer_radius_m <= inner_radius_m:
        raise ValueError("Beam element radii must satisfy outer_radius_m > inner_radius_m >= 0.")

    area_m2 = math.pi * ((outer_radius_m**2) - (inner_radius_m**2))
    second_moment_m4 = (math.pi / 4.0) * ((outer_radius_m**4) - (inner_radius_m**4))
    polar_moment_m4 = (math.pi / 2.0) * ((outer_radius_m**4) - (inner_radius_m**4))
    shear_modulus_pa = young_modulus_pa / (2.0 * (1.0 + poisson_ratio))

    axial_stiffness = young_modulus_pa * area_m2 / length_m
    torsional_stiffness = shear_modulus_pa * polar_moment_m4 / length_m
    bending_stiffness_l3 = 12.0 * young_modulus_pa * second_moment_m4 / (length_m**3)
    bending_stiffness_l2 = 6.0 * young_modulus_pa * second_moment_m4 / (length_m**2)
    bending_stiffness_4 = 4.0 * young_modulus_pa * second_moment_m4 / length_m
    bending_stiffness_2 = 2.0 * young_modulus_pa * second_moment_m4 / length_m

    matrix = np.zeros((12, 12), dtype=DTYPE)

    matrix[2, 2] = axial_stiffness
    matrix[2, 8] = -axial_stiffness
    matrix[8, 2] = -axial_stiffness
    matrix[8, 8] = axial_stiffness

    matrix[5, 5] = torsional_stiffness
    matrix[5, 11] = -torsional_stiffness
    matrix[11, 5] = -torsional_stiffness
    matrix[11, 11] = torsional_stiffness

    bending_submatrix = np.asarray(
        [
            [bending_stiffness_l3, bending_stiffness_l2, -bending_stiffness_l3, bending_stiffness_l2],
            [bending_stiffness_l2, bending_stiffness_4, -bending_stiffness_l2, bending_stiffness_2],
            [-bending_stiffness_l3, -bending_stiffness_l2, bending_stiffness_l3, -bending_stiffness_l2],
            [bending_stiffness_l2, bending_stiffness_2, -bending_stiffness_l2, bending_stiffness_4],
        ],
        dtype=DTYPE,
    )

    u_theta_y_indices = (0, 4, 6, 10)
    v_theta_x_indices = (1, 3, 7, 9)
    for row, row_index in enumerate(u_theta_y_indices):
        for column, column_index in enumerate(u_theta_y_indices):
            matrix[row_index, column_index] += bending_submatrix[row, column]
    for row, row_index in enumerate(v_theta_x_indices):
        for column, column_index in enumerate(v_theta_x_indices):
            matrix[row_index, column_index] += bending_submatrix[row, column]

    return matrix


def local_to_global_matrix(
    K_local: np.ndarray,
    eu: Sequence[float],
    ev: Sequence[float],
    ew: Sequence[float],
) -> np.ndarray:
    if K_local.shape != (12, 12):
        raise ValueError("K_local must have shape (12, 12).")

    rotation = _orthonormal_rotation(eu, ev, ew)
    transform = np.zeros((12, 12), dtype=DTYPE)
    for block_index in range(4):
        start = 3 * block_index
        transform[start : start + 3, start : start + 3] = rotation

    return (transform @ np.asarray(K_local, dtype=DTYPE) @ transform.T).astype(DTYPE, copy=False)


def assemble_global_stiffness(
    elements: Sequence[BeamElementDefinition | dict[str, Any] | Any],
    nodes: Sequence[Any],
) -> np.ndarray:
    dof_count = 6 * len(nodes)
    global_matrix = np.zeros((dof_count, dof_count), dtype=DTYPE)

    for element in elements:
        node_start_index = int(_get_value(element, "node_start_index"))
        node_end_index = int(_get_value(element, "node_end_index"))
        K_local = stiffness_matrix_local(
            length_m=float(_get_value(element, "length_m")),
            young_modulus_pa=float(_get_value(element, "young_modulus_pa")),
            poisson_ratio=float(_get_value(element, "poisson_ratio")),
            outer_radius_m=float(_get_value(element, "outer_radius_m")),
            inner_radius_m=float(_get_value(element, "inner_radius_m")),
        )
        K_global = local_to_global_matrix(
            K_local,
            _get_value(element, "eu"),
            _get_value(element, "ev"),
            _get_value(element, "ew"),
        )

        element_dofs = [
            (6 * node_start_index) + 0,
            (6 * node_start_index) + 1,
            (6 * node_start_index) + 2,
            (6 * node_start_index) + 3,
            (6 * node_start_index) + 4,
            (6 * node_start_index) + 5,
            (6 * node_end_index) + 0,
            (6 * node_end_index) + 1,
            (6 * node_end_index) + 2,
            (6 * node_end_index) + 3,
            (6 * node_end_index) + 4,
            (6 * node_end_index) + 5,
        ]
        for row_local, row_global in enumerate(element_dofs):
            for column_local, column_global in enumerate(element_dofs):
                global_matrix[row_global, column_global] += K_global[row_local, column_local]

    return global_matrix
