#include "centraltd/mechanical_solver.hpp"

#include "centraltd/centralizer_support.hpp"
#include "centraltd/global_solver.hpp"
#include "centraltd/lateral_equilibrium.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace centraltd {
namespace {

constexpr Scalar kMinimumPenaltyStiffnessNPerM = 1.0;
constexpr Scalar kBodyPenaltyMultiplier = 2.0;

std::vector<Scalar> compute_effective_axial_load_profile(
    const std::vector<MechanicalSegmentInput>& segments) {
  std::vector<Scalar> axial_loads_n(segments.size(), 0.0);
  Scalar load_below_n = 0.0;

  for (Index reverse_index = segments.size(); reverse_index > 0U; --reverse_index) {
    const Index segment_index = reverse_index - 1U;
    const auto& segment = segments.at(segment_index);
    const Scalar tangential_weight_n =
        segment.effective_line_weight_n_per_m *
        std::cos(segment.inclination_rad) *
        segment.segment_length_m;
    axial_loads_n.at(segment_index) = load_below_n + (0.5 * tangential_weight_n);
    load_below_n += tangential_weight_n;
  }

  return axial_loads_n;
}

}  // namespace

MechanicalSolverResult run_mechanical_baseline(const DiscretizedProblem& problem) {
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

  const auto axial_loads_n = compute_effective_axial_load_profile(problem.segments);
  std::vector<LateralEquilibriumState> lateral_states;
  std::vector<CentralizerSupportEffect> support_effects;
  std::vector<GlobalNodeInput> global_nodes;
  lateral_states.reserve(problem.segments.size());
  support_effects.reserve(problem.segments.size());
  global_nodes.reserve(problem.segments.size());

  for (Index segment_index = 0; segment_index < problem.segments.size(); ++segment_index) {
    const auto& segment = problem.segments.at(segment_index);
    const Scalar effective_axial_load_n = axial_loads_n.at(segment_index);
    const auto lateral_state = evaluate_lateral_equilibrium(segment, effective_axial_load_n);
    const auto support_effect = evaluate_centralizer_support_effect(
        segment,
        problem.centralizer_placements,
        lateral_state.structural_lateral_stiffness_n_per_m,
        problem.settings.contact_penalty_scale);
    const Scalar base_lateral_stiffness_n_per_m =
        lateral_state.structural_lateral_stiffness_n_per_m +
        support_effect.centering_stiffness_n_per_m;

    GlobalNodeInput node;
    node.node_index = segment_index;
    node.measured_depth_m = segment.measured_depth_center_m;
    node.segment_length_m = segment.segment_length_m;
    node.inclination_rad = segment.inclination_rad;
    node.curvature_rad_per_m = segment.curvature_rad_per_m;
    node.effective_line_weight_n_per_m = segment.effective_line_weight_n_per_m;
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
    node.gravity_lateral_load_n_per_m = lateral_state.gravity_lateral_load_n_per_m;
    node.curvature_lateral_load_n_per_m = lateral_state.curvature_lateral_load_n_per_m;
    node.equivalent_lateral_load_n_per_m = lateral_state.equivalent_lateral_load_n_per_m;
    node.equivalent_lateral_force_n = lateral_state.equivalent_lateral_force_n;
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
    node.section = segment.section;

    lateral_states.push_back(lateral_state);
    support_effects.push_back(support_effect);
    global_nodes.push_back(node);
  }

  const auto global_solution = run_global_lateral_solver(global_nodes, problem.settings);
  result.segment_results.resize(problem.segments.size());
  result.summary.global_solver_iteration_count = global_solution.iteration_count;
  result.summary.global_solver_final_update_norm_m = global_solution.final_update_norm_m;

  Scalar top_effective_axial_load_n = 0.0;
  for (Index segment_index = 0; segment_index < problem.segments.size(); ++segment_index) {
    const auto& segment = problem.segments.at(segment_index);
    const auto& lateral_state = lateral_states.at(segment_index);
    const auto& node = global_nodes.at(segment_index);
    const auto& node_solution = global_solution.node_solutions.at(segment_index);
    const Scalar tangential_weight_n =
        segment.effective_line_weight_n_per_m *
        std::cos(segment.inclination_rad) *
        segment.segment_length_m;
    top_effective_axial_load_n += tangential_weight_n;

    MechanicalSegmentResult segment_result;
    segment_result.measured_depth_start_m = segment.measured_depth_start_m;
    segment_result.measured_depth_end_m = segment.measured_depth_end_m;
    segment_result.measured_depth_center_m = segment.measured_depth_center_m;
    segment_result.segment_length_m = segment.segment_length_m;
    segment_result.section_name = segment.section.name;
    segment_result.inclination_rad = segment.inclination_rad;
    segment_result.curvature_rad_per_m = segment.curvature_rad_per_m;
    segment_result.effective_line_weight_n_per_m = segment.effective_line_weight_n_per_m;
    segment_result.effective_axial_load_n = axial_loads_n.at(segment_index);
    segment_result.bending_stiffness_n_m2 = segment.bending_stiffness_n_m2;
    segment_result.bending_moment_n_m = node.bending_moment_n_m;
    segment_result.bending_stress_pa = node.bending_stress_pa;
    segment_result.bending_strain_estimate = node.bending_strain_estimate;
    segment_result.gravity_lateral_load_n_per_m =
        lateral_state.gravity_lateral_load_n_per_m;
    segment_result.curvature_lateral_load_n_per_m =
        lateral_state.curvature_lateral_load_n_per_m;
    segment_result.equivalent_lateral_load_n_per_m =
        lateral_state.equivalent_lateral_load_n_per_m;
    segment_result.equivalent_lateral_force_n =
        lateral_state.equivalent_lateral_force_n;
    segment_result.bending_lateral_stiffness_n_per_m =
        lateral_state.bending_lateral_stiffness_n_per_m;
    segment_result.axial_tension_lateral_stiffness_n_per_m =
        lateral_state.axial_tension_lateral_stiffness_n_per_m;
    segment_result.structural_lateral_stiffness_n_per_m =
        lateral_state.structural_lateral_stiffness_n_per_m;
    segment_result.centralizer_centering_stiffness_n_per_m =
        node.centralizer_centering_stiffness_n_per_m;
    segment_result.support_contact_penalty_n_per_m =
        node.support_contact_penalty_n_per_m;
    segment_result.body_contact_penalty_n_per_m =
        node.body_contact_penalty_n_per_m;
    segment_result.support_outer_diameter_m = node.support_outer_diameter_m;
    segment_result.pipe_body_clearance_m = node.pipe_body_clearance_m;
    segment_result.support_contact_clearance_m = node.support_contact_clearance_m;
    segment_result.free_eccentricity_estimate_m =
        lateral_state.free_eccentricity_estimate_m;
    segment_result.eccentricity_estimate_m =
        node_solution.eccentricity_estimate_m;
    segment_result.eccentricity_ratio = node_solution.eccentricity_ratio;
    segment_result.standoff_estimate = node_solution.standoff_estimate;
    segment_result.support_normal_reaction_estimate_n =
        node_solution.support_normal_reaction_estimate_n;
    segment_result.body_normal_reaction_estimate_n =
        node_solution.body_normal_reaction_estimate_n;
    segment_result.normal_reaction_estimate_n =
        node_solution.normal_reaction_estimate_n;
    segment_result.normal_reaction_estimate_n_per_m =
        node_solution.normal_reaction_estimate_n_per_m;
    segment_result.nearby_centralizer_count = node.nearby_centralizer_count;
    segment_result.contact_iteration_count = global_solution.iteration_count;
    segment_result.contact_state = node_solution.contact_state;
    segment_result.support_in_contact = node_solution.support_in_contact;
    segment_result.pipe_body_in_contact = node_solution.pipe_body_in_contact;
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

  result.summary.top_effective_axial_load_n = top_effective_axial_load_n;
  result.surface_torque_n_m = std::nullopt;
  return result;
}

}  // namespace centraltd
