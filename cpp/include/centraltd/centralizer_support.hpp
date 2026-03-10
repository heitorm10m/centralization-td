#pragma once

#include "centraltd/discretization.hpp"

namespace centraltd {

struct CentralizerSupportEffect {
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  std::size_t nearby_centralizer_count{0};
  bool support_present{false};
};

CentralizerSupportEffect evaluate_centralizer_support_effect(
    const MechanicalSegmentInput& segment,
    const std::vector<CentralizerPlacement>& placements,
    Scalar structural_lateral_stiffness_n_per_m,
    Scalar contact_penalty_scale);

}  // namespace centraltd
