from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ..models import LoadedCase, Nodal6DOFOutputModel
from ..physics.bow_spring import (
    BowSpringContactModel,
    assemble_bow_spring_contact_force_and_jacobian,
    build_bow_directions,
    centralizer_axial_force_ratio,
    centralizer_effective_contact_diameter_m,
    centralizer_tangential_force_ratio,
    evaluate_bow_spring_segment_result,
    placement_proximity_weight,
    resolved_blade_power_law_k,
)
from ..physics.beam_element import BeamElementDefinition, assemble_global_stiffness
from ..physics.constants import DTYPE
from ..physics.contact import (
    BodyContactModel,
    assemble_body_contact_force_and_jacobian,
    run_global_lateral_solver,
)
from ..physics.fem import (
    CentralizerPlacementModel,
    ConstantForceElementModel,
    GlobalContactStateModel,
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
    NormalReactionPointModel,
    _norm2,
    assemble_global_linear_system,
    build_constant_force_vector,
    build_global_node_inputs,
    compute_buoyant_axial_load_profile,
    discretize_case,
)
from ..torque_drag_centralizer import evaluate_centralizer_torque_contribution


@dataclass(slots=True)
class NewtonRaphson6DOFResultModel:
    state_vector: np.ndarray
    residual_vector: np.ndarray
    residual_norm: float
    iteration_count: int
    converged: bool


def solve_6dof_newton_raphson(
    stiffness_matrix: np.ndarray,
    constant_force_vector: np.ndarray,
    *,
    body_contacts: Sequence[BodyContactModel] | None = None,
    bow_spring_contacts: Sequence[BowSpringContactModel] | None = None,
    initial_state_vector: Sequence[float] | None = None,
    tolerance: float = 1.0e-6,
    max_iterations: int = 50,
) -> NewtonRaphson6DOFResultModel:
    """
    Solve the isolated 6-DOF nonlinear equilibrium with Newton-Raphson.

    Reference:
      Dao et al. 2023, Sec. 3.4 / Eq. 31

    Notes:
    - `tolerance=1e-6` and `max_iterations=50` are implementation defaults,
      not paper-asserted values.
    - This function is intentionally isolated from the legacy reduced solver
      and is not wired into the runtime pipeline in this task.
    """
    stiffness_matrix = np.asarray(stiffness_matrix, dtype=DTYPE)
    constant_force_vector = np.asarray(constant_force_vector, dtype=DTYPE)
    if stiffness_matrix.ndim != 2 or stiffness_matrix.shape[0] != stiffness_matrix.shape[1]:
        raise ValueError("stiffness_matrix must be a square matrix.")
    if constant_force_vector.shape != (stiffness_matrix.shape[0],):
        raise ValueError("constant_force_vector must match the stiffness matrix dimension.")
    if tolerance <= 0.0:
        raise ValueError("tolerance must be strictly positive.")
    if max_iterations <= 0:
        raise ValueError("max_iterations must be strictly positive.")

    body_contacts = tuple(body_contacts or ())
    bow_spring_contacts = tuple(bow_spring_contacts or ())
    if initial_state_vector is None:
        state_vector = np.zeros_like(constant_force_vector, dtype=DTYPE)
    else:
        state_vector = np.asarray(initial_state_vector, dtype=DTYPE)
        if state_vector.shape != constant_force_vector.shape:
            raise ValueError("initial_state_vector must match the stiffness matrix dimension.")

    residual_vector = np.zeros_like(constant_force_vector, dtype=DTYPE)
    residual_norm = float("inf")
    for iteration_index in range(max_iterations):
        evaluation_state_vector = _regularize_bow_contact_state(
            state_vector,
            bow_spring_contacts,
        )
        body_force_vector, body_jacobian_matrix = assemble_body_contact_force_and_jacobian(
            body_contacts,
            evaluation_state_vector,
        )
        bow_force_vector, bow_jacobian_matrix = assemble_bow_spring_contact_force_and_jacobian(
            bow_spring_contacts,
            evaluation_state_vector,
        )
        contact_force_vector = body_force_vector + bow_force_vector
        contact_jacobian_matrix = body_jacobian_matrix + bow_jacobian_matrix

        residual_vector = (
            (stiffness_matrix @ state_vector)
            - constant_force_vector
            - contact_force_vector
        ).astype(DTYPE, copy=False)
        residual_norm = float(np.linalg.norm(residual_vector))
        if residual_norm < tolerance:
            return NewtonRaphson6DOFResultModel(
                state_vector=state_vector,
                residual_vector=residual_vector,
                residual_norm=residual_norm,
                iteration_count=iteration_index,
                converged=True,
            )

        jacobian_matrix = (stiffness_matrix - contact_jacobian_matrix).astype(DTYPE, copy=False)
        delta_state = np.linalg.solve(jacobian_matrix, -residual_vector).astype(DTYPE, copy=False)
        state_vector = (state_vector + delta_state).astype(DTYPE, copy=False)

    evaluation_state_vector = _regularize_bow_contact_state(
        state_vector,
        bow_spring_contacts,
    )
    body_force_vector, _ = assemble_body_contact_force_and_jacobian(
        body_contacts,
        evaluation_state_vector,
    )
    bow_force_vector, _ = assemble_bow_spring_contact_force_and_jacobian(
        bow_spring_contacts,
        evaluation_state_vector,
    )
    residual_vector = (
        (stiffness_matrix @ state_vector)
        - constant_force_vector
        - body_force_vector
        - bow_force_vector
    ).astype(DTYPE, copy=False)
    residual_norm = float(np.linalg.norm(residual_vector))
    return NewtonRaphson6DOFResultModel(
        state_vector=state_vector,
        residual_vector=residual_vector,
        residual_norm=residual_norm,
        iteration_count=max_iterations,
        converged=residual_norm < tolerance,
    )


def _vector_to_nodal_output(
    vector: Sequence[float],
) -> Nodal6DOFOutputModel:
    return Nodal6DOFOutputModel.from_sequence(vector, context="6-DOF nodal state")


def _regularize_bow_contact_state(
    state_vector: np.ndarray,
    bow_spring_contacts: Sequence[BowSpringContactModel],
) -> np.ndarray:
    regularized_state = np.asarray(state_vector, dtype=DTYPE).copy()
    regularized_nodes: set[int] = set()
    for contact in bow_spring_contacts:
        if contact.node_index in regularized_nodes:
            continue
        start = 6 * contact.node_index
        u_m = float(regularized_state[start + 0])
        v_m = float(regularized_state[start + 1])
        if abs(u_m) > 1.0e-12 or abs(v_m) > 1.0e-12:
            continue
        perturbation_m = max(1.0e-9, 1.0e-3 * float(contact.lambda_contact))
        regularized_state[start + 0] = DTYPE(perturbation_m * np.cos(DTYPE(contact.alpha_b_rad)))
        regularized_state[start + 1] = DTYPE(perturbation_m * np.sin(DTYPE(contact.alpha_b_rad)))
        regularized_nodes.add(contact.node_index)
    return regularized_state


def _default_lambda_contact_m(loaded_case: LoadedCase) -> float:
    # Implementation default for runtime wiring only. The paper states the role
    # of lambda and the condition h(lambda)=1/2 but does not expose a numerical
    # value in the available extracts.
    reference_clearance_m = max(loaded_case.minimum_nominal_radial_clearance_m(), 1.0e-3)
    return min(1.0e-3, reference_clearance_m)


def _apply_fixed_boundary_conditions(
    stiffness_matrix: np.ndarray,
    force_vector: np.ndarray,
    *,
    node_index: int,
) -> None:
    for dof_offset in range(6):
        dof_index = (6 * node_index) + dof_offset
        stiffness_matrix[dof_index, :] = 0.0
        stiffness_matrix[:, dof_index] = 0.0
        stiffness_matrix[dof_index, dof_index] = 1.0
        force_vector[dof_index] = 0.0


def _build_compatibility_6dof_solution(
    loaded_case: LoadedCase,
    segments: list[object],
    placements: list[CentralizerPlacementModel],
    nodes: list[object],
) -> NewtonRaphson6DOFResultModel:
    if not nodes:
        return NewtonRaphson6DOFResultModel(
            state_vector=np.zeros(0, dtype=DTYPE),
            residual_vector=np.zeros(0, dtype=DTYPE),
            residual_norm=0.0,
            iteration_count=0,
            converged=True,
        )

    node_placeholders = [object() for _ in nodes]
    if len(nodes) == 1:
        stiffness_matrix = np.eye(6, dtype=DTYPE)
        constant_force_vector = np.zeros(6, dtype=DTYPE)
        for component_index in range(2):
            constant_force_vector[component_index] = DTYPE(
                nodes[0].equivalent_lateral_force_n_b[component_index]
            )
        initial_state_vector = np.zeros(6, dtype=DTYPE)
        initial_state_vector[0] = DTYPE(nodes[0].free_displacement_n_b_m[0])
        initial_state_vector[1] = DTYPE(nodes[0].free_displacement_n_b_m[1])
        return solve_6dof_newton_raphson(
            stiffness_matrix,
            constant_force_vector,
            initial_state_vector=initial_state_vector,
            tolerance=1.0e-6,
            max_iterations=loaded_case.global_solver_max_iterations,
        )

    beam_elements: list[BeamElementDefinition] = []
    constant_force_elements: list[ConstantForceElementModel] = []
    for node_index in range(len(nodes) - 1):
        left_node = nodes[node_index]
        right_node = nodes[node_index + 1]
        left_segment = segments[node_index]
        spacing_m = max(right_node.measured_depth_m - left_node.measured_depth_m, 1.0e-6)
        section = left_segment.section
        poisson_ratio = max(
            -0.49,
            min(0.49, (section.young_modulus_pa / (2.0 * section.shear_modulus_pa)) - 1.0),
        )
        beam_elements.append(
            BeamElementDefinition(
                node_start_index=node_index,
                node_end_index=node_index + 1,
                length_m=spacing_m,
                young_modulus_pa=section.young_modulus_pa,
                poisson_ratio=poisson_ratio,
                outer_radius_m=0.5 * section.outer_diameter_m,
                inner_radius_m=0.5 * section.inner_diameter_m,
                eu=(1.0, 0.0, 0.0),
                ev=(0.0, 1.0, 0.0),
                ew=(0.0, 0.0, 1.0),
            )
        )
        constant_force_elements.append(
            ConstantForceElementModel(
                node_start_index=node_index,
                node_end_index=node_index + 1,
                length_m=spacing_m,
                density_kg_per_m3=section.density_kg_per_m3,
                fluid_density_kg_per_m3=loaded_case.fluid_density_kg_per_m3,
                cross_sectional_area_m2=section.cross_sectional_area_m2,
                tangent_start_north_east_tvd=left_segment.tangent_north_east_tvd,
            )
        )

    stiffness_matrix = assemble_global_stiffness(beam_elements, node_placeholders).astype(
        DTYPE,
        copy=False,
    )
    constant_force_vector = build_constant_force_vector(
        constant_force_elements,
        node_count=len(nodes),
        gravity_vector_north_east_tvd=(0.0, 0.0, 9.80665),
    ).astype(DTYPE, copy=False)

    for node_index, node in enumerate(nodes):
        constant_force_vector[(6 * node_index) + 0] += DTYPE(node.equivalent_lateral_force_n_b[0])
        constant_force_vector[(6 * node_index) + 1] += DTYPE(node.equivalent_lateral_force_n_b[1])

    _apply_fixed_boundary_conditions(stiffness_matrix, constant_force_vector, node_index=0)
    _apply_fixed_boundary_conditions(
        stiffness_matrix,
        constant_force_vector,
        node_index=len(nodes) - 1,
    )

    lambda_contact_m = _default_lambda_contact_m(loaded_case)
    initial_state_vector = np.zeros(6 * len(nodes), dtype=DTYPE)
    body_contacts: list[BodyContactModel] = []
    bow_spring_contacts: list[BowSpringContactModel] = []
    for node_index, (segment, node) in enumerate(zip(segments, nodes, strict=True)):
        initial_state_vector[(6 * node_index) + 0] = DTYPE(node.free_displacement_n_b_m[0])
        initial_state_vector[(6 * node_index) + 1] = DTYPE(node.free_displacement_n_b_m[1])
        section = segment.section
        outer_radius_m = 0.5 * section.outer_diameter_m
        body_contacts.append(
            BodyContactModel(
                node_index=node_index,
                outer_radius_m=outer_radius_m,
                wellbore_radius_m=outer_radius_m + node.pipe_body_clearance_m,
                contact_stiffness_n_per_m=max(1.0, node.body_contact_penalty_n_per_m),
                friction_coefficient_theta=max(0.0, section.friction_coefficient),
                friction_coefficient_z=max(0.0, section.friction_coefficient),
                effective_contact_radius_m=outer_radius_m,
                lambda_contact=lambda_contact_m,
            )
        )
        if segment.reference_hole_diameter_m <= 0.0:
            continue

        hole_radius_m = 0.5 * segment.reference_hole_diameter_m
        node_has_bow_contact = False
        fallback_angle_rad = 0.0
        for placement in placements:
            proximity_weight = placement_proximity_weight(
                placement,
                segment.measured_depth_center_m,
                segment.segment_length_m,
            )
            if proximity_weight <= 0.0:
                continue
            effective_contact_radius_m = 0.5 * centralizer_effective_contact_diameter_m(placement)
            min_blade_radius_m = (
                0.5 * placement.min_contact_diameter_m
                if placement.min_contact_diameter_m is not None
                else 0.5 * placement.support_outer_diameter_m
            )
            delta_rb_max_m = max(0.0, effective_contact_radius_m - min_blade_radius_m)
            blade_stiffness_n = proximity_weight * resolved_blade_power_law_k(
                placement,
                hole_radius_m,
            )
            for bow_index, angle_rad, _ in build_bow_directions(placement):
                if not node_has_bow_contact:
                    fallback_angle_rad = angle_rad
                    node_has_bow_contact = True
                bow_spring_contacts.append(
                    BowSpringContactModel(
                        node_index=node_index,
                        alpha_b_rad=angle_rad,
                        radial_clearance_m=placement.inner_clearance_to_pipe_m,
                        bow_radius_m=effective_contact_radius_m,
                        wellbore_radius_m=hole_radius_m,
                        blade_stiffness_n=blade_stiffness_n,
                        blade_power_p=placement.blade_power_law_p,
                        delta_rb_max_m=delta_rb_max_m,
                        contact_stiffness_n_per_m=max(
                            1.0,
                            max(node.support_contact_penalty_n_per_m, node.body_contact_penalty_n_per_m),
                        ),
                        friction_coefficient_theta=max(0.0, centralizer_tangential_force_ratio(placement)),
                        friction_coefficient_z=max(0.0, centralizer_axial_force_ratio(placement)),
                        effective_contact_radius_m=effective_contact_radius_m,
                        lambda_contact=lambda_contact_m,
                    )
                )
        node_u_m = float(initial_state_vector[(6 * node_index) + 0])
        node_v_m = float(initial_state_vector[(6 * node_index) + 1])
        if node_has_bow_contact and abs(node_u_m) <= 1.0e-12 and abs(node_v_m) <= 1.0e-12:
            perturbation_m = max(1.0e-9, 1.0e-3 * lambda_contact_m)
            initial_state_vector[(6 * node_index) + 0] = DTYPE(
                perturbation_m * np.cos(DTYPE(fallback_angle_rad))
            )
            initial_state_vector[(6 * node_index) + 1] = DTYPE(
                perturbation_m * np.sin(DTYPE(fallback_angle_rad))
            )

    return solve_6dof_newton_raphson(
        stiffness_matrix,
        constant_force_vector,
        body_contacts=body_contacts,
        bow_spring_contacts=bow_spring_contacts,
        initial_state_vector=initial_state_vector,
        tolerance=1.0e-6,
        max_iterations=loaded_case.global_solver_max_iterations,
    )


def _solve_mechanical_with_axial_profile_legacy(
    loaded_case: LoadedCase,
    effective_axial_loads_n: list[float],
    top_effective_axial_load_n: float,
) -> tuple[MechanicalSummaryModel, list[MechanicalSegmentResultModel], list[CentralizerPlacementModel]]:
    segments, placements, nodes = build_global_node_inputs(loaded_case, effective_axial_loads_n)
    if not nodes:
        return (
            MechanicalSummaryModel(
                segment_count=0,
                target_segment_length_m=loaded_case.discretization_step_m,
                global_solver_iteration_count=0,
                global_solver_final_update_norm_m=0.0,
                top_effective_axial_load_n=max(0.0, top_effective_axial_load_n),
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

    global_result = run_global_lateral_solver(nodes, max_iterations=loaded_case.global_solver_max_iterations)
    compatibility_6dof_result = _build_compatibility_6dof_solution(
        loaded_case,
        segments,
        placements,
        nodes,
    )
    free_nodal_outputs_6dof = [
        _vector_to_nodal_output(
            (
                node.free_displacement_n_b_m[0],
                node.free_displacement_n_b_m[1],
                0.0,
                0.0,
                0.0,
                0.0,
            )
        )
        for node in nodes
    ]
    solved_nodal_outputs_6dof = [
        _vector_to_nodal_output(
            compatibility_6dof_result.state_vector[(6 * node_index) : (6 * (node_index + 1))]
        )
        for node_index in range(len(nodes))
    ]

    profile: list[MechanicalSegmentResultModel] = []
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

    for node_index, (segment, node, solution) in enumerate(
        zip(segments, nodes, global_result.node_solutions, strict=True)
    ):
        bow_segment_result = evaluate_bow_spring_segment_result(
            segment,
            placements,
            (
                solution.lateral_displacement_normal_m,
                solution.lateral_displacement_binormal_m,
            ),
        )
        total_normal_reaction_vector_n_b = (
            bow_segment_result.bow_resultant_vector_n_b[0] + solution.body_normal_reaction_vector_n_b[0],
            bow_segment_result.bow_resultant_vector_n_b[1] + solution.body_normal_reaction_vector_n_b[1],
        )
        total_normal_reaction_magnitude_n = _norm2(total_normal_reaction_vector_n_b)
        contact_direction_n_b = solution.contact_direction_n_b
        if total_normal_reaction_magnitude_n > 1.0e-12:
            contact_direction_n_b = (
                total_normal_reaction_vector_n_b[0] / total_normal_reaction_magnitude_n,
                total_normal_reaction_vector_n_b[1] / total_normal_reaction_magnitude_n,
            )
        centralizer_torque_contribution = evaluate_centralizer_torque_contribution(
            bow_segment_result,
            contact_direction_n_b,
            solution.body_normal_reaction_vector_n_b,
        )

        result = MechanicalSegmentResultModel(
            measured_depth_start_m=segment.measured_depth_start_m,
            measured_depth_end_m=segment.measured_depth_end_m,
            measured_depth_center_m=segment.measured_depth_center_m,
            segment_length_m=segment.segment_length_m,
            section_name=segment.section.name,
            inclination_rad=segment.inclination_rad,
            curvature_rad_per_m=segment.curvature_rad_per_m,
            curvature_normal_component_rad_per_m=segment.curvature_normal_component_rad_per_m,
            curvature_binormal_component_rad_per_m=segment.curvature_binormal_component_rad_per_m,
            frame_rotation_change_rad=segment.frame_rotation_change_rad,
            tangent_north_east_tvd=segment.tangent_north_east_tvd,
            normal_north_east_tvd=segment.normal_north_east_tvd,
            binormal_north_east_tvd=segment.binormal_north_east_tvd,
            effective_line_weight_n_per_m=segment.effective_line_weight_n_per_m,
            effective_axial_load_n=node.effective_axial_load_n,
            bending_stiffness_n_m2=segment.bending_stiffness_n_m2,
            bending_moment_n_m=node.bending_moment_n_m,
            bending_stress_pa=node.bending_stress_pa,
            bending_strain_estimate=node.bending_strain_estimate,
            bending_severity_estimate=node.bending_strain_estimate,
            gravity_lateral_load_n_per_m=_norm2(node.gravity_lateral_load_n_b_n_per_m),
            curvature_lateral_load_n_per_m=_norm2(node.curvature_lateral_load_n_b_n_per_m),
            equivalent_lateral_load_n_per_m=node.equivalent_lateral_load_magnitude_n_per_m,
            equivalent_lateral_force_n=node.equivalent_lateral_force_magnitude_n,
            gravity_lateral_load_normal_n_per_m=node.gravity_lateral_load_n_b_n_per_m[0],
            gravity_lateral_load_binormal_n_per_m=node.gravity_lateral_load_n_b_n_per_m[1],
            curvature_lateral_load_normal_n_per_m=node.curvature_lateral_load_n_b_n_per_m[0],
            curvature_lateral_load_binormal_n_per_m=node.curvature_lateral_load_n_b_n_per_m[1],
            equivalent_lateral_load_normal_n_per_m=node.equivalent_lateral_load_n_b_n_per_m[0],
            equivalent_lateral_load_binormal_n_per_m=node.equivalent_lateral_load_n_b_n_per_m[1],
            bending_lateral_stiffness_n_per_m=node.bending_lateral_stiffness_n_per_m,
            axial_tension_lateral_stiffness_n_per_m=node.axial_tension_lateral_stiffness_n_per_m,
            structural_lateral_stiffness_n_per_m=node.structural_lateral_stiffness_n_per_m,
            centralizer_centering_stiffness_n_per_m=node.centralizer_centering_stiffness_n_per_m,
            support_contact_penalty_n_per_m=node.support_contact_penalty_n_per_m,
            body_contact_penalty_n_per_m=node.body_contact_penalty_n_per_m,
            support_outer_diameter_m=(
                bow_segment_result.support_outer_diameter_m
                if bow_segment_result.support_present
                else node.support_outer_diameter_m
            ),
            pipe_body_clearance_m=node.pipe_body_clearance_m,
            support_contact_clearance_m=(
                bow_segment_result.support_contact_clearance_m
                if bow_segment_result.support_present
                else node.support_contact_clearance_m
            ),
            bow_force_details=list(bow_segment_result.bow_force_details),
            free_nodal_output_6dof=free_nodal_outputs_6dof[node_index],
            solved_nodal_output_6dof=solved_nodal_outputs_6dof[node_index],
            free_eccentricity_estimate_m=_norm2(node.free_displacement_n_b_m),
            free_lateral_displacement_normal_m=node.free_displacement_n_b_m[0],
            free_lateral_displacement_binormal_m=node.free_displacement_n_b_m[1],
            lateral_displacement_normal_m=solution.lateral_displacement_normal_m,
            lateral_displacement_binormal_m=solution.lateral_displacement_binormal_m,
            eccentricity_normal_m=solution.lateral_displacement_normal_m,
            eccentricity_binormal_m=solution.lateral_displacement_binormal_m,
            eccentricity_estimate_m=solution.eccentricity_estimate_m,
            eccentricity_ratio=solution.eccentricity_ratio,
            standoff_estimate=solution.standoff_estimate,
            contact_direction_normal=contact_direction_n_b[0],
            contact_direction_binormal=contact_direction_n_b[1],
            support_normal_reaction_normal_n=bow_segment_result.bow_resultant_vector_n_b[0],
            support_normal_reaction_binormal_n=bow_segment_result.bow_resultant_vector_n_b[1],
            body_normal_reaction_normal_n=solution.body_normal_reaction_vector_n_b[0],
            body_normal_reaction_binormal_n=solution.body_normal_reaction_vector_n_b[1],
            normal_reaction_normal_n=total_normal_reaction_vector_n_b[0],
            normal_reaction_binormal_n=total_normal_reaction_vector_n_b[1],
            support_normal_reaction_estimate_n=bow_segment_result.bow_resultant_magnitude_n,
            body_normal_reaction_estimate_n=solution.body_normal_reaction_estimate_n,
            normal_reaction_estimate_n=total_normal_reaction_magnitude_n,
            normal_reaction_estimate_n_per_m=(
                0.0
                if segment.segment_length_m <= 0.0
                else total_normal_reaction_magnitude_n / segment.segment_length_m
            ),
            bow_resultant_normal_n=bow_segment_result.bow_resultant_vector_n_b[0],
            bow_resultant_binormal_n=bow_segment_result.bow_resultant_vector_n_b[1],
            bow_resultant_magnitude_n=bow_segment_result.bow_resultant_magnitude_n,
            centralizer_effective_radial_direction_normal=(
                centralizer_torque_contribution.effective_radial_direction_n_b[0]
            ),
            centralizer_effective_radial_direction_binormal=(
                centralizer_torque_contribution.effective_radial_direction_n_b[1]
            ),
            centralizer_tangential_direction_normal=(
                centralizer_torque_contribution.tangential_direction_n_b[0]
            ),
            centralizer_tangential_direction_binormal=(
                centralizer_torque_contribution.tangential_direction_n_b[1]
            ),
            centralizer_tangential_friction_normal_n=(
                centralizer_torque_contribution.tangential_friction_vector_n_b[0]
            ),
            centralizer_tangential_friction_binormal_n=(
                centralizer_torque_contribution.tangential_friction_vector_n_b[1]
            ),
            centralizer_tangential_friction_vector_magnitude_n=(
                centralizer_torque_contribution.tangential_friction_vector_magnitude_n
            ),
            centralizer_projected_contact_normal_n=centralizer_torque_contribution.projected_contact_normal_n,
            centralizer_friction_interaction_scale=centralizer_torque_contribution.friction_interaction_scale,
            centralizer_axial_friction_n=centralizer_torque_contribution.axial_friction_n,
            centralizer_tangential_friction_n=centralizer_torque_contribution.tangential_friction_n,
            centralizer_torque_increment_n_m=centralizer_torque_contribution.torque_increment_n_m,
            centralizer_effective_contact_radius_m=centralizer_torque_contribution.effective_contact_radius_m,
            centralizer_torque_details=list(centralizer_torque_contribution.placement_contributions),
            centralizer_torque_status=centralizer_torque_contribution.status,
            nearby_centralizer_count=node.nearby_centralizer_count,
            contact_iteration_count=max(1, compatibility_6dof_result.iteration_count),
            contact_state=(
                "pipe-body-contact"
                if solution.pipe_body_in_contact
                else "bow-spring-contact"
                if bow_segment_result.bow_resultant_magnitude_n > 0.0
                else "free"
            ),
            support_in_contact=bow_segment_result.bow_resultant_magnitude_n > 0.0,
            pipe_body_in_contact=solution.pipe_body_in_contact,
        )
        profile.append(result)
        minimum_effective_axial_load_n = min(minimum_effective_axial_load_n, result.effective_axial_load_n)
        maximum_effective_axial_load_n = max(maximum_effective_axial_load_n, result.effective_axial_load_n)
        maximum_bending_moment_n_m = max(maximum_bending_moment_n_m, result.bending_moment_n_m)
        maximum_bending_stress_pa = max(maximum_bending_stress_pa, result.bending_stress_pa)
        maximum_bending_strain_estimate = max(maximum_bending_strain_estimate, result.bending_strain_estimate)
        maximum_equivalent_lateral_load_n_per_m = max(
            maximum_equivalent_lateral_load_n_per_m,
            result.equivalent_lateral_load_n_per_m,
        )
        maximum_eccentricity_estimate_m = max(maximum_eccentricity_estimate_m, result.eccentricity_estimate_m)
        maximum_eccentricity_ratio = max(maximum_eccentricity_ratio, result.eccentricity_ratio)
        minimum_standoff_estimate = min(minimum_standoff_estimate, result.standoff_estimate)
        maximum_normal_reaction_estimate_n = max(maximum_normal_reaction_estimate_n, result.normal_reaction_estimate_n)
        contact_segment_count += int(result.support_in_contact or result.pipe_body_in_contact)
        support_contact_segment_count += int(result.support_in_contact)
        pipe_body_contact_segment_count += int(result.pipe_body_in_contact)

    summary = MechanicalSummaryModel(
        segment_count=len(profile),
        target_segment_length_m=loaded_case.discretization_step_m,
        global_solver_iteration_count=max(1, compatibility_6dof_result.iteration_count),
        global_solver_final_update_norm_m=compatibility_6dof_result.residual_norm,
        top_effective_axial_load_n=max(0.0, top_effective_axial_load_n),
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


def run_mechanical_baseline(
    loaded_case: LoadedCase,
) -> tuple[MechanicalSummaryModel, list[MechanicalSegmentResultModel], list[CentralizerPlacementModel]]:
    segments, _ = discretize_case(loaded_case)
    axial_profile_n, top_effective_axial_load_n = compute_buoyant_axial_load_profile(segments)
    return run_mechanical_with_axial_profile(
        loaded_case,
        axial_profile_n,
        top_effective_axial_load_n,
    )


def run_mechanical_with_axial_profile(
    loaded_case: LoadedCase,
    effective_axial_loads_n: list[float],
    top_effective_axial_load_n: float,
) -> tuple[MechanicalSummaryModel, list[MechanicalSegmentResultModel], list[CentralizerPlacementModel]]:
    return _solve_mechanical_with_axial_profile_legacy(
        loaded_case,
        effective_axial_loads_n,
        top_effective_axial_load_n,
    )
