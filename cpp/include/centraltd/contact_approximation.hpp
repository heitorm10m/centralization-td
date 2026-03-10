#pragma once

#include "centraltd/centralizer_support.hpp"
#include "centraltd/lateral_equilibrium.hpp"

#include <string>

namespace centraltd {

struct ContactApproximation {
  Scalar support_outer_diameter_m{0.0};
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  Scalar centralizer_centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  Scalar body_contact_penalty_n_per_m{0.0};
  Scalar free_eccentricity_estimate_m{0.0};
  Scalar eccentricity_estimate_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  Scalar support_contact_penetration_m{0.0};
  Scalar body_contact_penetration_m{0.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  Scalar final_update_norm_m{0.0};
  Scalar final_residual_n{0.0};
  std::size_t nearby_centralizer_count{0};
  std::size_t iteration_count{0};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
  std::string contact_state{"free"};
};

ContactApproximation evaluate_contact_approximation(
    const MechanicalSegmentInput& segment,
    const LateralEquilibriumState& lateral_state,
    const CentralizerSupportEffect& support_effect);

}  // namespace centraltd
