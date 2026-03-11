#include "centraltd/vector_contact.hpp"

#include "centraltd/vector_global_assembly.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumNorm = 1.0e-12;

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

Vector2 normalize(const Vector2& vector) {
  const Scalar magnitude = vector_norm(vector);
  if (magnitude <= kMinimumNorm) {
    return {1.0, 0.0};
  }
  return {vector[0] / magnitude, vector[1] / magnitude};
}

}  // namespace

Vector2 select_contact_direction(
    const Vector2& displacement_n_b_m,
    const Vector2& equivalent_force_n_b) {
  if (vector_norm(displacement_n_b_m) > kMinimumNorm) {
    return normalize(displacement_n_b_m);
  }
  if (vector_norm(equivalent_force_n_b) > kMinimumNorm) {
    return normalize(equivalent_force_n_b);
  }
  return {1.0, 0.0};
}

VectorContactResponse evaluate_vector_contact(
    const VectorNodeInput& node,
    const Vector2& displacement_n_b_m) {
  VectorContactResponse response;
  response.contact_direction_n_b =
      select_contact_direction(displacement_n_b_m, node.equivalent_lateral_force_n_b);
  response.eccentricity_magnitude_m = vector_norm(displacement_n_b_m);

  const Scalar support_penetration_m =
      node.support_contact_penalty_n_per_m > 0.0
          ? std::max(0.0, response.eccentricity_magnitude_m - node.support_contact_clearance_m)
          : 0.0;
  const Scalar body_penetration_m =
      std::max(0.0, response.eccentricity_magnitude_m - node.pipe_body_clearance_m);

  response.support_normal_reaction_estimate_n =
      node.support_contact_penalty_n_per_m * support_penetration_m;
  response.body_normal_reaction_estimate_n =
      node.body_contact_penalty_n_per_m * body_penetration_m;
  response.support_normal_reaction_vector_n_b = {
      response.support_normal_reaction_estimate_n * response.contact_direction_n_b[0],
      response.support_normal_reaction_estimate_n * response.contact_direction_n_b[1],
  };
  response.body_normal_reaction_vector_n_b = {
      response.body_normal_reaction_estimate_n * response.contact_direction_n_b[0],
      response.body_normal_reaction_estimate_n * response.contact_direction_n_b[1],
  };
  response.normal_reaction_vector_n_b = {
      response.support_normal_reaction_vector_n_b[0] + response.body_normal_reaction_vector_n_b[0],
      response.support_normal_reaction_vector_n_b[1] + response.body_normal_reaction_vector_n_b[1],
  };
  response.normal_reaction_estimate_n = vector_norm(response.normal_reaction_vector_n_b);
  response.normal_reaction_estimate_n_per_m =
      node.segment_length_m > 0.0 ? response.normal_reaction_estimate_n / node.segment_length_m : 0.0;
  response.support_in_contact = response.support_normal_reaction_estimate_n > 0.0;
  response.pipe_body_in_contact = response.body_normal_reaction_estimate_n > 0.0;

  if (node.pipe_body_clearance_m > 0.0) {
    response.eccentricity_ratio = response.eccentricity_magnitude_m / node.pipe_body_clearance_m;
    response.standoff_estimate = std::clamp(1.0 - response.eccentricity_ratio, 0.0, 1.0);
  } else {
    response.eccentricity_ratio = 1.0;
    response.standoff_estimate = 0.0;
  }

  if (response.pipe_body_in_contact) {
    response.contact_state = "pipe-body-contact";
  } else if (response.support_in_contact) {
    response.contact_state = "support-contact";
  }

  return response;
}

}  // namespace centraltd
