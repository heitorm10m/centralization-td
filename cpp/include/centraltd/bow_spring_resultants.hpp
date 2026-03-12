#pragma once

#include "centraltd/discretization.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct BowForceDetail {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};
  Index bow_index{0};
  Scalar angle_rad{0.0};
  Vector2 direction_n_b{1.0, 0.0};
  Scalar deflection_m{0.0};
  Scalar force_magnitude_n{0.0};
  Vector2 force_vector_n_b{0.0, 0.0};
};

struct CentralizerPlacementResultantDetail {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar axial_force_ratio{0.0};
  Scalar tangential_force_ratio{0.0};
  Vector2 bow_resultant_vector_n_b{0.0, 0.0};
  Scalar bow_resultant_magnitude_n{0.0};
};

struct BowSpringSegmentResult {
  Scalar support_outer_diameter_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  Scalar equivalent_centering_stiffness_n_per_m{0.0};
  std::size_t nearby_centralizer_count{0};
  bool support_present{false};
  std::vector<BowForceDetail> bow_force_details;
  std::vector<CentralizerPlacementResultantDetail> placement_resultant_details;
  Vector2 bow_resultant_vector_n_b{0.0, 0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Scalar centralizer_axial_friction_n{0.0};
  Scalar centralizer_tangential_friction_n{0.0};
  Scalar centralizer_torque_increment_n_m{0.0};
  Scalar effective_contact_radius_m{0.0};
};

BowSpringSegmentResult evaluate_bow_spring_segment_result(
    const MechanicalSegmentInput& segment,
    const std::vector<CentralizerPlacement>& placements,
    const Vector2& eccentricity_n_b_m);

}  // namespace centraltd
