#pragma once

#include "centraltd/bow_spring_resultants.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct CentralizerPlacementTorqueContribution {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar axial_force_ratio{0.0};
  Scalar tangential_force_ratio{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar torsional_slip_indicator{0.0};
  Scalar tangential_mobilization{0.0};
  Scalar torsional_tangential_demand_factor{1.0};
  Scalar tangential_traction_indicator{0.0};
  std::string tangential_regime{"inactive"};
  Vector2 bow_resultant_vector_n_b{0.0, 0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Vector2 radial_direction_n_b{0.0, 0.0};
  Vector2 local_contact_direction_n_b{0.0, 0.0};
  Vector2 effective_radial_direction_n_b{0.0, 0.0};
  Vector2 tangential_direction_n_b{0.0, 0.0};
  Vector2 tangential_friction_vector_n_b{0.0, 0.0};
  Scalar tangential_friction_vector_magnitude_n{0.0};
  Scalar local_contact_weight{0.0};
  Scalar direction_alignment_cosine{0.0};
  Scalar projected_contact_normal_n{0.0};
  Scalar friction_interaction_scale{1.0};
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar torque_increment_n_m{0.0};
  bool active{false};
  std::string status{"inactive"};
};

struct CentralizerTangentialVectorContribution {
  Scalar measured_depth_m{0.0};
  Scalar effective_radial_direction_normal{0.0};
  Scalar effective_radial_direction_binormal{0.0};
  Scalar tangential_direction_normal{0.0};
  Scalar tangential_direction_binormal{0.0};
  Scalar tangential_friction_normal_n{0.0};
  Scalar tangential_friction_binormal_n{0.0};
  Scalar projected_contact_normal_n{0.0};
  Scalar friction_interaction_scale{1.0};
  Scalar tangential_friction_magnitude_n{0.0};
  std::string status{"inactive"};
};

struct CentralizerTorquePartitionSummary {
  Scalar body_surface_torque_n_m{0.0};
  Scalar centralizer_surface_torque_n_m{0.0};
  Scalar total_surface_torque_n_m{0.0};
  Scalar body_axial_friction_sum_n{0.0};
  Scalar centralizer_axial_friction_sum_n{0.0};
  Scalar body_tangential_friction_sum_n{0.0};
  Scalar centralizer_tangential_friction_sum_n{0.0};
  std::string status{"phase14-reduced-body-centralizer-torque-partition"};
};

struct CentralizerTorqueContribution {
  std::vector<CentralizerPlacementTorqueContribution> placement_contributions;
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar torsional_slip_indicator{0.0};
  Scalar tangential_mobilization{0.0};
  Scalar torsional_tangential_demand_factor{1.0};
  Scalar tangential_traction_indicator{0.0};
  std::string tangential_regime{"inactive"};
  Vector2 bow_resultant_vector_n_b{0.0, 0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Vector2 local_contact_direction_n_b{0.0, 0.0};
  Vector2 effective_radial_direction_n_b{0.0, 0.0};
  Vector2 tangential_direction_n_b{0.0, 0.0};
  Vector2 tangential_friction_vector_n_b{0.0, 0.0};
  Scalar tangential_friction_vector_magnitude_n{0.0};
  Scalar projected_contact_normal_n{0.0};
  Scalar friction_interaction_scale{1.0};
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar torque_increment_n_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  bool active{false};
  std::string status{"inactive"};
};

CentralizerTorqueContribution evaluate_centralizer_torque_contribution(
    const BowSpringSegmentResult& bow_segment_result,
    const Vector2& local_contact_direction_n_b,
    const Vector2& body_normal_reaction_vector_n_b);

// This reduced feedback law reuses the contact-informed bow/resultant details
// from the structural pass and modulates only the tangential centralizer demand
// with a bounded torsional mobilization factor based on |twist_rate| * r_eff.
// It remains an `n-b`-frame reduced law, not a full 3D tangential-contact solve.
CentralizerTorqueContribution evaluate_centralizer_torque_contribution_from_details(
    const std::vector<CentralizerPlacementTorqueContribution>& placement_details,
    Scalar reduced_torsional_load_n_m,
    Scalar reduced_twist_rate_rad_per_m);

}  // namespace centraltd
