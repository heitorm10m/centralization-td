from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.physics.constants import DTYPE  # noqa: E402
from centraltd.physics.fem import (  # noqa: E402
    ConstantForceElementModel,
    build_gravity_buoyancy_force_vector,
    distributed_gravity_buoyancy_force_n_per_m,
    equivalent_gravity_buoyancy_nodal_loads,
)


def test_zero_buoyancy_case_produces_zero_nodal_forces() -> None:
    element = ConstantForceElementModel(
        node_start_index=0,
        node_end_index=1,
        length_m=10.0,
        density_kg_per_m3=1100.0,
        fluid_density_kg_per_m3=1100.0,
        cross_sectional_area_m2=0.01,
        tangent_start_north_east_tvd=(1.0, 0.0, 0.0),
    )

    global_force = build_gravity_buoyancy_force_vector(
        [element],
        node_count=2,
        gravity_vector_north_east_tvd=(0.0, 0.0, 9.81),
    )

    assert global_force.dtype == DTYPE
    assert np.allclose(global_force, 0.0)


def test_moment_contribution_is_equal_and_opposite_between_nodes() -> None:
    distributed_force = distributed_gravity_buoyancy_force_n_per_m(
        density_kg_per_m3=7850.0,
        fluid_density_kg_per_m3=1100.0,
        cross_sectional_area_m2=0.02,
        gravity_vector_north_east_tvd=(0.0, 0.0, 9.81),
    )
    node_1_load, node_2_load = equivalent_gravity_buoyancy_nodal_loads(
        length_m=8.0,
        tangent_start_north_east_tvd=(1.0, 0.0, 0.0),
        distributed_force_n_per_m=distributed_force,
    )

    assert np.allclose(node_1_load[3:], -node_2_load[3:])
    assert not np.allclose(node_1_load[3:], 0.0)


def test_interior_node_assembly_sums_adjacent_element_contributions() -> None:
    gravity_vector = (0.0, 0.0, 9.81)
    element_left = ConstantForceElementModel(
        node_start_index=0,
        node_end_index=1,
        length_m=4.0,
        density_kg_per_m3=7850.0,
        fluid_density_kg_per_m3=1100.0,
        cross_sectional_area_m2=0.01,
        tangent_start_north_east_tvd=(1.0, 0.0, 0.0),
    )
    element_right = ConstantForceElementModel(
        node_start_index=1,
        node_end_index=2,
        length_m=6.0,
        density_kg_per_m3=7850.0,
        fluid_density_kg_per_m3=1100.0,
        cross_sectional_area_m2=0.01,
        tangent_start_north_east_tvd=(0.0, 1.0, 0.0),
    )

    assembled = build_gravity_buoyancy_force_vector(
        [element_left, element_right],
        node_count=3,
        gravity_vector_north_east_tvd=gravity_vector,
    )
    left_node_1, left_node_2 = equivalent_gravity_buoyancy_nodal_loads(
        length_m=element_left.length_m,
        tangent_start_north_east_tvd=element_left.tangent_start_north_east_tvd,
        distributed_force_n_per_m=distributed_gravity_buoyancy_force_n_per_m(
            density_kg_per_m3=element_left.density_kg_per_m3,
            fluid_density_kg_per_m3=element_left.fluid_density_kg_per_m3,
            cross_sectional_area_m2=element_left.cross_sectional_area_m2,
            gravity_vector_north_east_tvd=gravity_vector,
        ),
    )
    right_node_1, right_node_2 = equivalent_gravity_buoyancy_nodal_loads(
        length_m=element_right.length_m,
        tangent_start_north_east_tvd=element_right.tangent_start_north_east_tvd,
        distributed_force_n_per_m=distributed_gravity_buoyancy_force_n_per_m(
            density_kg_per_m3=element_right.density_kg_per_m3,
            fluid_density_kg_per_m3=element_right.fluid_density_kg_per_m3,
            cross_sectional_area_m2=element_right.cross_sectional_area_m2,
            gravity_vector_north_east_tvd=gravity_vector,
        ),
    )

    interior_slice = slice(6, 12)
    expected_interior = left_node_2 + right_node_1

    assert np.allclose(assembled[interior_slice], expected_interior)
    assert np.allclose(assembled[:6], left_node_1)
    assert np.allclose(assembled[12:18], right_node_2)


def test_gravity_force_points_in_ez_direction() -> None:
    distributed_force = distributed_gravity_buoyancy_force_n_per_m(
        density_kg_per_m3=7850.0,
        fluid_density_kg_per_m3=1100.0,
        cross_sectional_area_m2=0.02,
        gravity_vector_north_east_tvd=(0.0, 0.0, 9.81),
    )
    node_1_load, node_2_load = equivalent_gravity_buoyancy_nodal_loads(
        length_m=5.0,
        tangent_start_north_east_tvd=(1.0, 0.0, 0.0),
        distributed_force_n_per_m=distributed_force,
    )

    assert distributed_force[0] == pytest.approx(0.0)
    assert distributed_force[1] == pytest.approx(0.0)
    assert distributed_force[2] > 0.0
    assert node_1_load[0] == pytest.approx(0.0)
    assert node_1_load[1] == pytest.approx(0.0)
    assert node_1_load[2] > 0.0
    assert node_2_load[0] == pytest.approx(0.0)
    assert node_2_load[1] == pytest.approx(0.0)
    assert node_2_load[2] > 0.0
