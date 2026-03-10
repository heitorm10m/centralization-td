#pragma once

#include "centraltd/discretization.hpp"

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
  Scalar effective_line_weight_n_per_m{0.0};
  Scalar effective_axial_load_n{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  Scalar bending_moment_n_m{0.0};
  Scalar bending_stress_pa{0.0};
  Scalar bending_strain_estimate{0.0};
  Scalar gravity_lateral_load_n_per_m{0.0};
  Scalar curvature_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_force_n{0.0};
  Scalar bending_lateral_stiffness_n_per_m{0.0};
  Scalar axial_tension_lateral_stiffness_n_per_m{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Scalar centralizer_centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  Scalar body_contact_penalty_n_per_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  Scalar free_eccentricity_estimate_m{0.0};
  Scalar eccentricity_estimate_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  std::size_t nearby_centralizer_count{0};
  std::size_t contact_iteration_count{0};
  std::string contact_state{"free"};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
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

MechanicalSolverResult run_mechanical_baseline(const DiscretizedProblem& problem);

}  // namespace centraltd
