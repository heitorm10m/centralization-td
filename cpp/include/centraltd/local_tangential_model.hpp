#pragma once

#include "centraltd/types.hpp"

#include <string>

namespace centraltd {

struct LocalTangentialModelInputs {
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar normal_capacity_n{0.0};
  Scalar baseline_tangential_demand_n{0.0};
};

struct LocalTangentialState {
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar normal_capacity_n{0.0};
  Scalar baseline_tangential_demand_n{0.0};
  Scalar tangential_slip_indicator{0.0};
  Scalar tangential_mobilization{0.0};
  Scalar tangential_demand_factor{1.0};
  Scalar mobilized_tangential_demand_n{0.0};
  Scalar tangential_traction_indicator{0.0};
  bool active{false};
  bool feedback_applied{false};
  std::string tangential_regime{"inactive"};
  std::string status{"inactive"};
};

// Shared reduced local tangential state for pipe-body and centralizer contact.
// `normal_capacity_n` is the local normal support proxy [N] used only to build
// a bounded traction indicator, not a claim of full 3D tangential contact.
LocalTangentialState evaluate_local_tangential_state(
    const LocalTangentialModelInputs& inputs);

}  // namespace centraltd
