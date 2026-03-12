#include "centraltd/torque_drag_centralizer.hpp"

#include <cmath>

namespace centraltd {
namespace {

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

Vector2 normalize_or_zero(const Vector2& vector) {
  const Scalar magnitude = vector_norm(vector);
  if (magnitude <= 1.0e-12) {
    return {0.0, 0.0};
  }
  return {vector[0] / magnitude, vector[1] / magnitude};
}

Vector2 rotate_ccw_90(const Vector2& vector) {
  return {-vector[1], vector[0]};
}

}  // namespace

CentralizerTorqueContribution evaluate_centralizer_torque_contribution(
    const BowSpringSegmentResult& bow_segment_result) {
  CentralizerTorqueContribution result;

  for (const auto& placement_result : bow_segment_result.placement_resultant_details) {
    if (placement_result.bow_resultant_magnitude_n <= 0.0) {
      continue;
    }

    const Vector2 radial_direction_n_b = normalize_or_zero(
        placement_result.bow_resultant_vector_n_b);
    const Vector2 tangential_direction_n_b = rotate_ccw_90(radial_direction_n_b);
    const Scalar axial_friction_n =
        placement_result.axial_force_ratio * placement_result.bow_resultant_magnitude_n;
    const Scalar tangential_friction_n =
        placement_result.tangential_force_ratio * placement_result.bow_resultant_magnitude_n;
    const Vector2 tangential_friction_vector_n_b{
        tangential_friction_n * tangential_direction_n_b[0],
        tangential_friction_n * tangential_direction_n_b[1],
    };

    result.active = true;
    result.axial_friction_n += axial_friction_n;
    result.tangential_friction_n += tangential_friction_n;
    result.tangential_friction_vector_n_b[0] += tangential_friction_vector_n_b[0];
    result.tangential_friction_vector_n_b[1] += tangential_friction_vector_n_b[1];
    result.torque_increment_n_m +=
        tangential_friction_n * placement_result.effective_contact_radius_m;
    result.effective_contact_radius_m = std::max(
        result.effective_contact_radius_m,
        placement_result.effective_contact_radius_m);
  }

  result.tangential_friction_vector_magnitude_n =
      vector_norm(result.tangential_friction_vector_n_b);
  if (result.tangential_friction_vector_magnitude_n > 1.0e-12) {
    result.tangential_direction_n_b = {
        result.tangential_friction_vector_n_b[0] / result.tangential_friction_vector_magnitude_n,
        result.tangential_friction_vector_n_b[1] / result.tangential_friction_vector_magnitude_n,
    };
    result.status = "phase11-reduced-vector-tangential-centralizer-torque";
    return result;
  }

  if (bow_segment_result.bow_resultant_magnitude_n > 1.0e-12) {
    result.tangential_direction_n_b = rotate_ccw_90(normalize_or_zero(
        bow_segment_result.bow_resultant_vector_n_b));
    result.status = "phase11-reduced-vector-tangential-centralizer-torque";
    return result;
  }

  result.status = "inactive";
  return result;
}

}  // namespace centraltd
