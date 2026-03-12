#pragma once

#include "centraltd/mechanical_solver.hpp"
#include "centraltd/string_section.hpp"

namespace centraltd {

struct BodyTorqueDragContribution {
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar torque_increment_n_m{0.0};
  Scalar contact_radius_m{0.0};
};

BodyTorqueDragContribution evaluate_body_torque_drag_contribution(
    const MechanicalSegmentResult& segment,
    const StringSection& section);

}  // namespace centraltd
