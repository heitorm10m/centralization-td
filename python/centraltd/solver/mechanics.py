from __future__ import annotations

from ..models import LoadedCase
from ..physics.bow_spring import evaluate_bow_spring_segment_result
from ..physics.contact import run_global_lateral_solver
from ..physics.fem import (
    CentralizerPlacementModel,
    GlobalContactStateModel,
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
    NormalReactionPointModel,
    _norm2,
    assemble_global_linear_system,
    build_global_node_inputs,
    compute_buoyant_axial_load_profile,
    discretize_case,
)
from ..torque_drag_centralizer import evaluate_centralizer_torque_contribution


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

    for segment, node, solution in zip(segments, nodes, global_result.node_solutions, strict=True):
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
                0.0 if segment.segment_length_m <= 0.0 else total_normal_reaction_magnitude_n / segment.segment_length_m
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
            contact_iteration_count=global_result.iteration_count,
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
        global_solver_iteration_count=global_result.iteration_count,
        global_solver_final_update_norm_m=global_result.final_update_norm_m,
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
