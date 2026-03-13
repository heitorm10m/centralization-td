#pragma once

#include "centraltd/mechanical_solver.hpp"
#include "centraltd/string_section.hpp"

#include <string>

namespace centraltd {

struct BodyTorqueDragContribution {
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar torque_increment_n_m{0.0};
  Scalar contact_radius_m{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar torsional_slip_indicator{0.0};
  Scalar tangential_mobilization{0.0};
  Scalar tangential_demand_factor{1.0};
  Scalar tangential_traction_indicator{0.0};
  std::string tangential_regime{"inactive"};
  bool feedback_applied{false};
  std::string status{"inactive"};
};

BodyTorqueDragContribution evaluate_body_torque_drag_contribution(
    const MechanicalSegmentResult& segment,
    const StringSection& section,
    Scalar reduced_torsional_load_n_m = 0.0,
    Scalar reduced_twist_rate_rad_per_m = 0.0);

}  // namespace centraltd
