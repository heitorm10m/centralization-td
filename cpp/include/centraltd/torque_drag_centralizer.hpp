#pragma once

#include "centraltd/bow_spring_resultants.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct CentralizerTangentialVectorContribution {
  Scalar measured_depth_m{0.0};
  Scalar tangential_direction_normal{0.0};
  Scalar tangential_direction_binormal{0.0};
  Scalar tangential_friction_normal_n{0.0};
  Scalar tangential_friction_binormal_n{0.0};
  Scalar tangential_friction_magnitude_n{0.0};
};

struct CentralizerTorquePartitionSummary {
  Scalar body_surface_torque_n_m{0.0};
  Scalar centralizer_surface_torque_n_m{0.0};
  Scalar total_surface_torque_n_m{0.0};
  Scalar body_axial_friction_sum_n{0.0};
  Scalar centralizer_axial_friction_sum_n{0.0};
  Scalar body_tangential_friction_sum_n{0.0};
  Scalar centralizer_tangential_friction_sum_n{0.0};
  std::string status{"phase11-reduced-body-centralizer-torque-partition"};
};

struct CentralizerTorqueContribution {
  Vector2 tangential_direction_n_b{0.0, 0.0};
  Vector2 tangential_friction_vector_n_b{0.0, 0.0};
  Scalar tangential_friction_vector_magnitude_n{0.0};
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar torque_increment_n_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  bool active{false};
  std::string status{"inactive"};
};

CentralizerTorqueContribution evaluate_centralizer_torque_contribution(
    const BowSpringSegmentResult& bow_segment_result);

}  // namespace centraltd
