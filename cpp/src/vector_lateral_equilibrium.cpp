#include "centraltd/vector_lateral_equilibrium.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumSegmentLengthM = 1.0e-6;

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

}  // namespace

VectorLateralEquilibriumState evaluate_vector_lateral_equilibrium(
    const MechanicalSegmentInput& segment,
    Scalar effective_axial_load_n) {
  VectorLateralEquilibriumState state;
  const Scalar segment_length_m = std::max(segment.segment_length_m, kMinimumSegmentLengthM);

  const Vector3 global_gravity_direction_north_east_tvd{0.0, 0.0, 1.0};
  const Scalar gravity_normal_component =
      segment.effective_line_weight_n_per_m *
      ((global_gravity_direction_north_east_tvd[0] * segment.normal_north_east_tvd[0]) +
       (global_gravity_direction_north_east_tvd[1] * segment.normal_north_east_tvd[1]) +
       (global_gravity_direction_north_east_tvd[2] * segment.normal_north_east_tvd[2]));
  const Scalar gravity_binormal_component =
      segment.effective_line_weight_n_per_m *
      ((global_gravity_direction_north_east_tvd[0] * segment.binormal_north_east_tvd[0]) +
       (global_gravity_direction_north_east_tvd[1] * segment.binormal_north_east_tvd[1]) +
       (global_gravity_direction_north_east_tvd[2] * segment.binormal_north_east_tvd[2]));
  state.gravity_lateral_load_n_b_n_per_m = {
      gravity_normal_component,
      gravity_binormal_component,
  };

  state.curvature_lateral_load_n_b_n_per_m = {
      std::max(effective_axial_load_n, 0.0) * segment.curvature_normal_component_rad_per_m,
      std::max(effective_axial_load_n, 0.0) * segment.curvature_binormal_component_rad_per_m,
  };
  state.equivalent_lateral_load_n_b_n_per_m = {
      state.gravity_lateral_load_n_b_n_per_m[0] + state.curvature_lateral_load_n_b_n_per_m[0],
      state.gravity_lateral_load_n_b_n_per_m[1] + state.curvature_lateral_load_n_b_n_per_m[1],
  };
  state.equivalent_lateral_force_n_b = {
      state.equivalent_lateral_load_n_b_n_per_m[0] * segment.segment_length_m,
      state.equivalent_lateral_load_n_b_n_per_m[1] * segment.segment_length_m,
  };
  state.equivalent_lateral_load_magnitude_n_per_m =
      vector_norm(state.equivalent_lateral_load_n_b_n_per_m);
  state.equivalent_lateral_force_magnitude_n = vector_norm(state.equivalent_lateral_force_n_b);

  if (segment.bending_stiffness_n_m2 > 0.0) {
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
    state.free_displacement_n_b_m = {
        state.equivalent_lateral_force_n_b[0] / state.structural_lateral_stiffness_n_per_m,
        state.equivalent_lateral_force_n_b[1] / state.structural_lateral_stiffness_n_per_m,
    };
  }

  return state;
}

}  // namespace centraltd
