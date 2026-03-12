#include "centraltd/torque_drag_body.hpp"

#include <algorithm>

namespace centraltd {
namespace {

Scalar body_contact_radius_m(const StringSection& section) {
  return std::max(0.0, 0.5 * section.outer_diameter_m);
}

}  // namespace

BodyTorqueDragContribution evaluate_body_torque_drag_contribution(
    const MechanicalSegmentResult& segment,
    const StringSection& section) {
  const Scalar friction_force_n =
      std::max(0.0, section.friction_coefficient) *
      std::max(0.0, segment.body_normal_reaction_estimate_n);
  const Scalar contact_radius_m = body_contact_radius_m(section);
  return BodyTorqueDragContribution{
      friction_force_n,
      friction_force_n,
      friction_force_n * contact_radius_m,
      contact_radius_m,
  };
}

}  // namespace centraltd
