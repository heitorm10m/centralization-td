#pragma once

#include "centraltd/bow_spring_resultants.hpp"
#include "centraltd/discretization.hpp"
#include "centraltd/torque_drag_centralizer.hpp"

#include <optional>
#include <string>
#include <vector>

namespace centraltd {

struct MechanicalSegmentResult {
  Scalar measured_depth_start_m{0.0};
  Scalar measured_depth_end_m{0.0};
  Scalar measured_depth_center_m{0.0};
  Scalar segment_length_m{0.0};
  std::string section_name;
  Scalar inclination_rad{0.0};
  Scalar curvature_rad_per_m{0.0};
  Scalar curvature_normal_component_rad_per_m{0.0};
  Scalar curvature_binormal_component_rad_per_m{0.0};
  Scalar frame_rotation_change_rad{0.0};
  Vector3 tangent_north_east_tvd{0.0, 0.0, 1.0};
  Vector3 normal_north_east_tvd{1.0, 0.0, 0.0};
  Vector3 binormal_north_east_tvd{0.0, 1.0, 0.0};
  Scalar effective_line_weight_n_per_m{0.0};
  Scalar effective_axial_load_n{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  Scalar bending_moment_n_m{0.0};
  Scalar bending_stress_pa{0.0};
  Scalar bending_strain_estimate{0.0};
  Scalar bending_severity_estimate{0.0};
  Scalar gravity_lateral_load_n_per_m{0.0};
  Scalar curvature_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_force_n{0.0};
  Scalar gravity_lateral_load_normal_n_per_m{0.0};
  Scalar gravity_lateral_load_binormal_n_per_m{0.0};
  Scalar curvature_lateral_load_normal_n_per_m{0.0};
  Scalar curvature_lateral_load_binormal_n_per_m{0.0};
  Scalar equivalent_lateral_load_normal_n_per_m{0.0};
  Scalar equivalent_lateral_load_binormal_n_per_m{0.0};
  Scalar bending_lateral_stiffness_n_per_m{0.0};
  Scalar axial_tension_lateral_stiffness_n_per_m{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Scalar centralizer_centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  Scalar body_contact_penalty_n_per_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  std::vector<BowForceDetail> bow_force_details;
  Scalar free_eccentricity_estimate_m{0.0};
  Scalar free_lateral_displacement_normal_m{0.0};
  Scalar free_lateral_displacement_binormal_m{0.0};
  Scalar lateral_displacement_normal_m{0.0};
  Scalar lateral_displacement_binormal_m{0.0};
  Scalar eccentricity_normal_m{0.0};
  Scalar eccentricity_binormal_m{0.0};
  Scalar eccentricity_estimate_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  Scalar contact_direction_normal{1.0};
  Scalar contact_direction_binormal{0.0};
  Scalar support_normal_reaction_normal_n{0.0};
  Scalar support_normal_reaction_binormal_n{0.0};
  Scalar body_normal_reaction_normal_n{0.0};
  Scalar body_normal_reaction_binormal_n{0.0};
  Scalar normal_reaction_normal_n{0.0};
  Scalar normal_reaction_binormal_n{0.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  Scalar bow_resultant_normal_n{0.0};
  Scalar bow_resultant_binormal_n{0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Scalar centralizer_effective_radial_direction_normal{0.0};
  Scalar centralizer_effective_radial_direction_binormal{0.0};
  Scalar centralizer_tangential_direction_normal{0.0};
  Scalar centralizer_tangential_direction_binormal{0.0};
  Scalar centralizer_tangential_friction_normal_n{0.0};
  Scalar centralizer_tangential_friction_binormal_n{0.0};
  Scalar centralizer_tangential_friction_vector_magnitude_n{0.0};
  Scalar centralizer_projected_contact_normal_n{0.0};
  Scalar centralizer_friction_interaction_scale{1.0};
  Scalar centralizer_axial_friction_n{0.0};
  Scalar centralizer_tangential_friction_n{0.0};
  Scalar centralizer_torque_increment_n_m{0.0};
  Scalar centralizer_effective_contact_radius_m{0.0};
  std::vector<CentralizerPlacementTorqueContribution> centralizer_torque_details;
  std::string centralizer_torque_status{"inactive"};
  std::size_t nearby_centralizer_count{0};
  std::size_t contact_iteration_count{0};
  std::string contact_state{"free"};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
};

struct NormalReactionPoint {
  Scalar measured_depth_m{0.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
};

struct MechanicalSummary {
  std::size_t segment_count{0};
  Scalar target_segment_length_m{0.0};
  std::size_t global_solver_iteration_count{0};
  Scalar global_solver_final_update_norm_m{0.0};
  Scalar top_effective_axial_load_n{0.0};
  Scalar minimum_effective_axial_load_n{0.0};
  Scalar maximum_effective_axial_load_n{0.0};
  Scalar maximum_bending_moment_n_m{0.0};
  Scalar maximum_bending_stress_pa{0.0};
  Scalar maximum_bending_strain_estimate{0.0};
  Scalar maximum_equivalent_lateral_load_n_per_m{0.0};
  Scalar maximum_eccentricity_estimate_m{0.0};
  Scalar maximum_eccentricity_ratio{0.0};
  Scalar minimum_standoff_estimate{1.0};
  Scalar maximum_normal_reaction_estimate_n{0.0};
  std::size_t contact_segment_count{0};
  std::size_t support_contact_segment_count{0};
  std::size_t pipe_body_contact_segment_count{0};
};

struct MechanicalSolverResult {
  MechanicalSummary summary;
  std::vector<MechanicalSegmentResult> segment_results;
  std::optional<Scalar> surface_torque_n_m;
};

struct AxialLoadProfile {
  std::vector<Scalar> segment_center_loads_n;
  Scalar top_boundary_load_n{0.0};
};

AxialLoadProfile compute_buoyant_axial_load_profile(
    const std::vector<MechanicalSegmentInput>& segments);

MechanicalSolverResult run_mechanical_with_axial_profile(
    const DiscretizedProblem& problem,
    const std::vector<Scalar>& effective_axial_loads_n,
    Scalar top_effective_axial_load_n);

MechanicalSolverResult run_mechanical_baseline(const DiscretizedProblem& problem);

}  // namespace centraltd
