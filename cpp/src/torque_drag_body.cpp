#include "centraltd/torque_drag_body.hpp"

#include "centraltd/local_tangential_model.hpp"

#include <algorithm>

namespace centraltd {
namespace {

Scalar body_contact_radius_m(const StringSection& section) {
  return std::max(0.0, 0.5 * section.outer_diameter_m);
}

}  // namespace

BodyTorqueDragContribution evaluate_body_torque_drag_contribution(
    const MechanicalSegmentResult& segment,
    const StringSection& section,
    Scalar reduced_torsional_load_n_m,
    Scalar reduced_twist_rate_rad_per_m) {
  const Scalar axial_friction_n =
      std::max(0.0, section.friction_coefficient) *
      std::max(0.0, segment.body_normal_reaction_estimate_n);
  const Scalar contact_radius_m = body_contact_radius_m(section);
  const auto tangential_state = evaluate_local_tangential_state(
      LocalTangentialModelInputs{
          std::max(0.0, reduced_torsional_load_n_m),
          std::abs(reduced_twist_rate_rad_per_m),
          contact_radius_m,
          std::max(0.0, segment.body_normal_reaction_estimate_n),
          axial_friction_n,
      });
  const Scalar tangential_friction_n =
      tangential_state.mobilized_tangential_demand_n;
  return BodyTorqueDragContribution{
      axial_friction_n,
      tangential_friction_n,
      tangential_friction_n * contact_radius_m,
      contact_radius_m,
      tangential_state.reduced_torsional_load_n_m,
      tangential_state.reduced_twist_rate_rad_per_m,
      tangential_state.tangential_slip_indicator,
      tangential_state.tangential_mobilization,
      tangential_state.tangential_demand_factor,
      tangential_state.tangential_traction_indicator,
      tangential_state.tangential_regime,
      tangential_state.feedback_applied,
      axial_friction_n > 0.0
          ? (tangential_state.feedback_applied
                 ? "phase14-reduced-local-tangential-body-law"
                 : "phase14-reduced-local-tangential-body-law-baseline")
                 : "inactive",
  };
}

}  // namespace centraltd
