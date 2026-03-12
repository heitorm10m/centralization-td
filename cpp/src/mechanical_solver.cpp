#include "centraltd/mechanical_solver.hpp"

#include "centraltd/bow_spring_resultants.hpp"
#include "centraltd/centralizer_support.hpp"
#include "centraltd/torque_drag_centralizer.hpp"
#include "centraltd/vector_global_solver.hpp"
#include "centraltd/vector_lateral_equilibrium.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace centraltd {
namespace {

constexpr Scalar kMinimumPenaltyStiffnessNPerM = 1.0;
constexpr Scalar kBodyPenaltyMultiplier = 2.0;

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

}  // namespace

AxialLoadProfile compute_buoyant_axial_load_profile(
    const std::vector<MechanicalSegmentInput>& segments) {
  AxialLoadProfile profile;
  profile.segment_center_loads_n.assign(segments.size(), 0.0);
  Scalar load_below_n = 0.0;

  for (Index reverse_index = segments.size(); reverse_index > 0U; --reverse_index) {
    const Index segment_index = reverse_index - 1U;
    const auto& segment = segments.at(segment_index);
    const Scalar tangential_weight_n =
        segment.effective_line_weight_n_per_m *
        std::cos(segment.inclination_rad) *
        segment.segment_length_m;
    profile.segment_center_loads_n.at(segment_index) = load_below_n + (0.5 * tangential_weight_n);
    load_below_n += tangential_weight_n;
  }

  profile.top_boundary_load_n = load_below_n;
  return profile;
}

MechanicalSolverResult run_mechanical_with_axial_profile(
    const DiscretizedProblem& problem,
    const std::vector<Scalar>& effective_axial_loads_n,
    Scalar top_effective_axial_load_n) {
  MechanicalSolverResult result;
  result.summary.segment_count = problem.segments.size();
  result.summary.target_segment_length_m = problem.settings.target_segment_length_m;
  result.summary.maximum_effective_axial_load_n = std::numeric_limits<Scalar>::lowest();
  result.summary.minimum_effective_axial_load_n = std::numeric_limits<Scalar>::max();
  result.summary.minimum_standoff_estimate = 1.0;

  if (problem.segments.empty()) {
    result.summary.minimum_effective_axial_load_n = 0.0;
    result.summary.maximum_effective_axial_load_n = 0.0;
    result.summary.minimum_standoff_estimate = 0.0;
    result.surface_torque_n_m = std::nullopt;
    return result;
  }

  if (effective_axial_loads_n.size() != problem.segments.size()) {
    throw ValidationError("Mechanical solver axial profile must match the discretized segment count.");
  }

  std::vector<VectorLateralEquilibriumState> lateral_states;
  std::vector<VectorNodeInput> global_nodes;
  lateral_states.reserve(problem.segments.size());
  global_nodes.reserve(problem.segments.size());

  for (Index segment_index = 0; segment_index < problem.segments.size(); ++segment_index) {
    const auto& segment = problem.segments.at(segment_index);
    const Scalar effective_axial_load_n = effective_axial_loads_n.at(segment_index);
    const auto lateral_state = evaluate_vector_lateral_equilibrium(segment, effective_axial_load_n);
    const auto support_effect = evaluate_centralizer_support_effect(
        segment,
        problem.centralizer_placements,
        lateral_state.structural_lateral_stiffness_n_per_m,
        problem.settings.contact_penalty_scale);
    const Scalar base_lateral_stiffness_n_per_m =
        lateral_state.structural_lateral_stiffness_n_per_m +
        support_effect.centering_stiffness_n_per_m;

    VectorNodeInput node;
    node.node_index = segment_index;
    node.measured_depth_m = segment.measured_depth_center_m;
    node.segment_length_m = segment.segment_length_m;
    node.effective_axial_load_n = effective_axial_load_n;
    node.bending_stiffness_n_m2 = segment.bending_stiffness_n_m2;
    node.bending_moment_n_m =
        segment.bending_stiffness_n_m2 * segment.curvature_rad_per_m;
    node.bending_stress_pa =
        segment.section.young_modulus_pa *
        segment.curvature_rad_per_m *
        segment.section.outer_radius_m();
    node.bending_strain_estimate =
        segment.curvature_rad_per_m * segment.section.outer_radius_m();
    node.gravity_lateral_load_n_b_n_per_m = lateral_state.gravity_lateral_load_n_b_n_per_m;
    node.curvature_lateral_load_n_b_n_per_m = lateral_state.curvature_lateral_load_n_b_n_per_m;
    node.equivalent_lateral_load_n_b_n_per_m =
        lateral_state.equivalent_lateral_load_n_b_n_per_m;
    node.equivalent_lateral_force_n_b = lateral_state.equivalent_lateral_force_n_b;
    node.equivalent_lateral_load_magnitude_n_per_m =
        lateral_state.equivalent_lateral_load_magnitude_n_per_m;
    node.equivalent_lateral_force_magnitude_n =
        lateral_state.equivalent_lateral_force_magnitude_n;
    node.bending_lateral_stiffness_n_per_m =
        lateral_state.bending_lateral_stiffness_n_per_m;
    node.axial_tension_lateral_stiffness_n_per_m =
        lateral_state.axial_tension_lateral_stiffness_n_per_m;
    node.structural_lateral_stiffness_n_per_m =
        lateral_state.structural_lateral_stiffness_n_per_m;
    node.centralizer_centering_stiffness_n_per_m =
        support_effect.centering_stiffness_n_per_m;
    node.support_contact_penalty_n_per_m =
        support_effect.support_contact_penalty_n_per_m;
    node.body_contact_penalty_n_per_m = kBodyPenaltyMultiplier *
        problem.settings.contact_penalty_scale *
        std::max(base_lateral_stiffness_n_per_m, kMinimumPenaltyStiffnessNPerM);
    node.support_outer_diameter_m = support_effect.support_outer_diameter_m;
    node.pipe_body_clearance_m = support_effect.pipe_body_clearance_m;
    node.support_contact_clearance_m = support_effect.support_contact_clearance_m;
    node.nearby_centralizer_count = support_effect.nearby_centralizer_count;
    node.free_displacement_n_b_m = lateral_state.free_displacement_n_b_m;
    node.section = segment.section;

    lateral_states.push_back(lateral_state);
    global_nodes.push_back(node);
  }

  const auto global_solution = run_vector_global_lateral_solver(global_nodes, problem.settings);
  result.segment_results.resize(problem.segments.size());
  result.summary.global_solver_iteration_count = global_solution.iteration_count;
  result.summary.global_solver_final_update_norm_m = global_solution.final_update_norm_m;

  for (Index segment_index = 0; segment_index < problem.segments.size(); ++segment_index) {
    const auto& segment = problem.segments.at(segment_index);
    const auto& lateral_state = lateral_states.at(segment_index);
    const auto& node = global_nodes.at(segment_index);
    const auto& node_solution = global_solution.node_solutions.at(segment_index);
    const auto bow_segment_result = evaluate_bow_spring_segment_result(
        segment,
        problem.centralizer_placements,
        {
            node_solution.lateral_displacement_normal_m,
            node_solution.lateral_displacement_binormal_m,
        });
    const auto centralizer_torque_contribution =
        evaluate_centralizer_torque_contribution(bow_segment_result);
    const Vector2 total_normal_reaction_vector_n_b{
        bow_segment_result.bow_resultant_vector_n_b[0] +
            node_solution.body_normal_reaction_vector_n_b[0],
        bow_segment_result.bow_resultant_vector_n_b[1] +
            node_solution.body_normal_reaction_vector_n_b[1],
    };
    const Scalar total_normal_reaction_magnitude_n =
        vector_norm(total_normal_reaction_vector_n_b);
    Vector2 contact_direction_n_b = node_solution.contact_direction_n_b;
    if (total_normal_reaction_magnitude_n > 1.0e-12) {
      contact_direction_n_b = {
          total_normal_reaction_vector_n_b[0] / total_normal_reaction_magnitude_n,
          total_normal_reaction_vector_n_b[1] / total_normal_reaction_magnitude_n,
      };
    }

    MechanicalSegmentResult segment_result;
    segment_result.measured_depth_start_m = segment.measured_depth_start_m;
    segment_result.measured_depth_end_m = segment.measured_depth_end_m;
    segment_result.measured_depth_center_m = segment.measured_depth_center_m;
    segment_result.segment_length_m = segment.segment_length_m;
    segment_result.section_name = segment.section.name;
    segment_result.inclination_rad = segment.inclination_rad;
    segment_result.curvature_rad_per_m = segment.curvature_rad_per_m;
    segment_result.curvature_normal_component_rad_per_m =
        segment.curvature_normal_component_rad_per_m;
    segment_result.curvature_binormal_component_rad_per_m =
        segment.curvature_binormal_component_rad_per_m;
    segment_result.frame_rotation_change_rad = segment.frame_rotation_change_rad;
    segment_result.tangent_north_east_tvd = segment.tangent_north_east_tvd;
    segment_result.normal_north_east_tvd = segment.normal_north_east_tvd;
    segment_result.binormal_north_east_tvd = segment.binormal_north_east_tvd;
    segment_result.effective_line_weight_n_per_m = segment.effective_line_weight_n_per_m;
    segment_result.effective_axial_load_n = effective_axial_loads_n.at(segment_index);
    segment_result.bending_stiffness_n_m2 = segment.bending_stiffness_n_m2;
    segment_result.bending_moment_n_m = node.bending_moment_n_m;
    segment_result.bending_stress_pa = node.bending_stress_pa;
    segment_result.bending_strain_estimate = node.bending_strain_estimate;
    segment_result.bending_severity_estimate = node.bending_strain_estimate;
    segment_result.gravity_lateral_load_n_per_m = vector_norm(
        lateral_state.gravity_lateral_load_n_b_n_per_m);
    segment_result.curvature_lateral_load_n_per_m = vector_norm(
        lateral_state.curvature_lateral_load_n_b_n_per_m);
    segment_result.equivalent_lateral_load_n_per_m =
        lateral_state.equivalent_lateral_load_magnitude_n_per_m;
    segment_result.equivalent_lateral_force_n =
        lateral_state.equivalent_lateral_force_magnitude_n;
    segment_result.gravity_lateral_load_normal_n_per_m =
        lateral_state.gravity_lateral_load_n_b_n_per_m[0];
    segment_result.gravity_lateral_load_binormal_n_per_m =
        lateral_state.gravity_lateral_load_n_b_n_per_m[1];
    segment_result.curvature_lateral_load_normal_n_per_m =
        lateral_state.curvature_lateral_load_n_b_n_per_m[0];
    segment_result.curvature_lateral_load_binormal_n_per_m =
        lateral_state.curvature_lateral_load_n_b_n_per_m[1];
    segment_result.equivalent_lateral_load_normal_n_per_m =
        lateral_state.equivalent_lateral_load_n_b_n_per_m[0];
    segment_result.equivalent_lateral_load_binormal_n_per_m =
        lateral_state.equivalent_lateral_load_n_b_n_per_m[1];
    segment_result.bending_lateral_stiffness_n_per_m = node.bending_lateral_stiffness_n_per_m;
    segment_result.axial_tension_lateral_stiffness_n_per_m = node.axial_tension_lateral_stiffness_n_per_m;
    segment_result.structural_lateral_stiffness_n_per_m =
        lateral_state.structural_lateral_stiffness_n_per_m;
    segment_result.centralizer_centering_stiffness_n_per_m =
        node.centralizer_centering_stiffness_n_per_m;
    segment_result.support_contact_penalty_n_per_m =
        node.support_contact_penalty_n_per_m;
    segment_result.body_contact_penalty_n_per_m =
        node.body_contact_penalty_n_per_m;
    segment_result.support_outer_diameter_m = bow_segment_result.support_present
        ? bow_segment_result.support_outer_diameter_m
        : node.support_outer_diameter_m;
    segment_result.pipe_body_clearance_m = node.pipe_body_clearance_m;
    segment_result.support_contact_clearance_m = bow_segment_result.support_present
        ? bow_segment_result.support_contact_clearance_m
        : node.support_contact_clearance_m;
    segment_result.bow_force_details = bow_segment_result.bow_force_details;
    segment_result.free_lateral_displacement_normal_m = node.free_displacement_n_b_m[0];
    segment_result.free_lateral_displacement_binormal_m = node.free_displacement_n_b_m[1];
    segment_result.free_eccentricity_estimate_m = vector_norm(node.free_displacement_n_b_m);
    segment_result.lateral_displacement_normal_m =
        node_solution.lateral_displacement_normal_m;
    segment_result.lateral_displacement_binormal_m =
        node_solution.lateral_displacement_binormal_m;
    segment_result.eccentricity_normal_m = node_solution.lateral_displacement_normal_m;
    segment_result.eccentricity_binormal_m = node_solution.lateral_displacement_binormal_m;
    segment_result.eccentricity_estimate_m = node_solution.eccentricity_estimate_m;
    segment_result.eccentricity_ratio = node_solution.eccentricity_ratio;
    segment_result.standoff_estimate = node_solution.standoff_estimate;
    segment_result.contact_direction_normal = contact_direction_n_b[0];
    segment_result.contact_direction_binormal = contact_direction_n_b[1];
    segment_result.support_normal_reaction_normal_n =
        bow_segment_result.bow_resultant_vector_n_b[0];
    segment_result.support_normal_reaction_binormal_n =
        bow_segment_result.bow_resultant_vector_n_b[1];
    segment_result.body_normal_reaction_normal_n =
        node_solution.body_normal_reaction_vector_n_b[0];
    segment_result.body_normal_reaction_binormal_n =
        node_solution.body_normal_reaction_vector_n_b[1];
    segment_result.normal_reaction_normal_n =
        total_normal_reaction_vector_n_b[0];
    segment_result.normal_reaction_binormal_n =
        total_normal_reaction_vector_n_b[1];
    segment_result.support_normal_reaction_estimate_n =
        bow_segment_result.bow_resultant_magnitude_n;
    segment_result.body_normal_reaction_estimate_n =
        node_solution.body_normal_reaction_estimate_n;
    segment_result.normal_reaction_estimate_n =
        total_normal_reaction_magnitude_n;
    segment_result.normal_reaction_estimate_n_per_m =
        segment.segment_length_m > 0.0
            ? total_normal_reaction_magnitude_n / segment.segment_length_m
            : 0.0;
    segment_result.bow_resultant_normal_n =
        bow_segment_result.bow_resultant_vector_n_b[0];
    segment_result.bow_resultant_binormal_n =
        bow_segment_result.bow_resultant_vector_n_b[1];
    segment_result.bow_resultant_magnitude_n =
        bow_segment_result.bow_resultant_magnitude_n;
    segment_result.centralizer_tangential_direction_normal =
        centralizer_torque_contribution.tangential_direction_n_b[0];
    segment_result.centralizer_tangential_direction_binormal =
        centralizer_torque_contribution.tangential_direction_n_b[1];
    segment_result.centralizer_tangential_friction_normal_n =
        centralizer_torque_contribution.tangential_friction_vector_n_b[0];
    segment_result.centralizer_tangential_friction_binormal_n =
        centralizer_torque_contribution.tangential_friction_vector_n_b[1];
    segment_result.centralizer_tangential_friction_vector_magnitude_n =
        centralizer_torque_contribution.tangential_friction_vector_magnitude_n;
    segment_result.centralizer_axial_friction_n =
        centralizer_torque_contribution.axial_friction_n;
    segment_result.centralizer_tangential_friction_n =
        centralizer_torque_contribution.tangential_friction_n;
    segment_result.centralizer_torque_increment_n_m =
        centralizer_torque_contribution.torque_increment_n_m;
    segment_result.centralizer_effective_contact_radius_m =
        centralizer_torque_contribution.effective_contact_radius_m;
    segment_result.nearby_centralizer_count = node.nearby_centralizer_count;
    segment_result.contact_iteration_count = global_solution.iteration_count;
    segment_result.support_in_contact =
        bow_segment_result.bow_resultant_magnitude_n > 0.0;
    segment_result.pipe_body_in_contact = node_solution.pipe_body_in_contact;
    segment_result.contact_state = segment_result.pipe_body_in_contact
        ? "pipe-body-contact"
        : segment_result.support_in_contact
            ? "bow-spring-contact"
            : "free";
    result.segment_results.at(segment_index) = segment_result;

    result.summary.maximum_effective_axial_load_n = std::max(
        result.summary.maximum_effective_axial_load_n,
        segment_result.effective_axial_load_n);
    result.summary.minimum_effective_axial_load_n = std::min(
        result.summary.minimum_effective_axial_load_n,
        segment_result.effective_axial_load_n);
    result.summary.maximum_bending_moment_n_m = std::max(
        result.summary.maximum_bending_moment_n_m,
        segment_result.bending_moment_n_m);
    result.summary.maximum_bending_stress_pa = std::max(
        result.summary.maximum_bending_stress_pa,
        segment_result.bending_stress_pa);
    result.summary.maximum_bending_strain_estimate = std::max(
        result.summary.maximum_bending_strain_estimate,
        segment_result.bending_strain_estimate);
    result.summary.maximum_equivalent_lateral_load_n_per_m = std::max(
        result.summary.maximum_equivalent_lateral_load_n_per_m,
        segment_result.equivalent_lateral_load_n_per_m);
    result.summary.maximum_eccentricity_estimate_m = std::max(
        result.summary.maximum_eccentricity_estimate_m,
        segment_result.eccentricity_estimate_m);
    result.summary.maximum_eccentricity_ratio = std::max(
        result.summary.maximum_eccentricity_ratio,
        segment_result.eccentricity_ratio);
    result.summary.minimum_standoff_estimate = std::min(
        result.summary.minimum_standoff_estimate,
        segment_result.standoff_estimate);
    result.summary.maximum_normal_reaction_estimate_n = std::max(
        result.summary.maximum_normal_reaction_estimate_n,
        segment_result.normal_reaction_estimate_n);
    result.summary.contact_segment_count +=
        (segment_result.support_in_contact || segment_result.pipe_body_in_contact) ? 1U : 0U;
    result.summary.support_contact_segment_count +=
        segment_result.support_in_contact ? 1U : 0U;
    result.summary.pipe_body_contact_segment_count +=
        segment_result.pipe_body_in_contact ? 1U : 0U;
  }

  result.summary.top_effective_axial_load_n = std::max(0.0, top_effective_axial_load_n);
  result.surface_torque_n_m = std::nullopt;
  return result;
}

MechanicalSolverResult run_mechanical_baseline(const DiscretizedProblem& problem) {
  const auto buoyant_axial_profile = compute_buoyant_axial_load_profile(problem.segments);
  return run_mechanical_with_axial_profile(
      problem,
      buoyant_axial_profile.segment_center_loads_n,
      buoyant_axial_profile.top_boundary_load_n);
}

}  // namespace centraltd
