#include "centraltd/local_tangential_model.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kEpsilon = 1.0e-12;

Scalar tangential_slip_indicator(
    Scalar reduced_twist_rate_rad_per_m,
    Scalar effective_contact_radius_m) {
  return std::max(
      0.0,
      std::abs(reduced_twist_rate_rad_per_m) * std::max(0.0, effective_contact_radius_m));
}

Scalar tangential_mobilization(Scalar tangential_slip_indicator_value) {
  return tangential_slip_indicator_value /
         (1.0 + tangential_slip_indicator_value);
}

}  // namespace

LocalTangentialState evaluate_local_tangential_state(
    const LocalTangentialModelInputs& inputs) {
  LocalTangentialState state;
  state.reduced_torsional_load_n_m = std::max(0.0, inputs.reduced_torsional_load_n_m);
  state.reduced_twist_rate_rad_per_m = std::abs(inputs.reduced_twist_rate_rad_per_m);
  state.effective_contact_radius_m = std::max(0.0, inputs.effective_contact_radius_m);
  state.normal_capacity_n = std::max(0.0, inputs.normal_capacity_n);
  state.baseline_tangential_demand_n =
      std::max(0.0, inputs.baseline_tangential_demand_n);
  state.tangential_slip_indicator = tangential_slip_indicator(
      state.reduced_twist_rate_rad_per_m,
      state.effective_contact_radius_m);
  state.tangential_mobilization =
      tangential_mobilization(state.tangential_slip_indicator);
  state.tangential_demand_factor = 1.0 + state.tangential_mobilization;
  state.mobilized_tangential_demand_n =
      state.baseline_tangential_demand_n * state.tangential_demand_factor;
  state.tangential_traction_indicator =
      state.normal_capacity_n > kEpsilon
          ? std::min(
                1.0,
                state.mobilized_tangential_demand_n / state.normal_capacity_n)
          : 0.0;
  state.active = state.normal_capacity_n > kEpsilon &&
                 state.effective_contact_radius_m > kEpsilon;
  state.feedback_applied = state.active &&
                           state.tangential_mobilization > kEpsilon &&
                           state.baseline_tangential_demand_n > kEpsilon;

  if (!state.active || state.mobilized_tangential_demand_n <= kEpsilon) {
    state.tangential_regime = "inactive";
    state.status = "inactive";
    return state;
  }

  if (state.tangential_traction_indicator < 0.25) {
    state.tangential_regime = "reduced-partial-adhesion-proxy";
  } else if (state.tangential_traction_indicator < 0.95) {
    state.tangential_regime = "reduced-mobilizing-traction";
  } else {
    state.tangential_regime = "reduced-slip-limited-traction";
  }

  state.status = state.feedback_applied
                     ? "phase14-reduced-local-tangential-state-mobilized"
                     : "phase14-reduced-local-tangential-state-baseline";
  return state;
}

}  // namespace centraltd
