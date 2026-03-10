#include "centraltd/lateral_equilibrium.hpp"

#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumSegmentLengthM = 1.0e-6;

}  // namespace

LateralEquilibriumState evaluate_lateral_equilibrium(
    const MechanicalSegmentInput& segment,
    Scalar effective_axial_load_n) {
  LateralEquilibriumState state;
  const Scalar segment_length_m = std::max(segment.segment_length_m, kMinimumSegmentLengthM);

  state.gravity_lateral_load_n_per_m =
      std::abs(segment.effective_line_weight_n_per_m * std::sin(segment.inclination_rad));
  state.curvature_lateral_load_n_per_m =
      std::max(effective_axial_load_n, 0.0) * segment.curvature_rad_per_m;
  state.equivalent_lateral_load_n_per_m =
      state.gravity_lateral_load_n_per_m + state.curvature_lateral_load_n_per_m;
  state.equivalent_lateral_force_n =
      state.equivalent_lateral_load_n_per_m * segment.segment_length_m;

  if (segment.bending_stiffness_n_m2 > 0.0) {
    // Equivalent lateral stiffness of a simply supported beam under uniform
    // load, derived from delta_max = 5 q L^4 / (384 E I).
    state.bending_lateral_stiffness_n_per_m =
        (384.0 * segment.bending_stiffness_n_m2) /
        (5.0 * std::pow(segment_length_m, 3.0));
  }

  if (effective_axial_load_n > 0.0) {
    state.axial_tension_lateral_stiffness_n_per_m =
        (4.0 * effective_axial_load_n) / segment_length_m;
  }

  state.structural_lateral_stiffness_n_per_m =
      state.bending_lateral_stiffness_n_per_m +
      state.axial_tension_lateral_stiffness_n_per_m;

  if (state.structural_lateral_stiffness_n_per_m > 0.0) {
    state.free_eccentricity_estimate_m =
        state.equivalent_lateral_force_n / state.structural_lateral_stiffness_n_per_m;
  }

  return state;
}

}  // namespace centraltd
