#pragma once

#include "centraltd/types.hpp"

#include <string>

namespace centraltd {

struct VectorNodeInput;
struct VectorContactState;

struct VectorContactResponse {
  Vector2 contact_direction_n_b{1.0, 0.0};
  Vector2 support_normal_reaction_vector_n_b{0.0, 0.0};
  Vector2 body_normal_reaction_vector_n_b{0.0, 0.0};
  Vector2 normal_reaction_vector_n_b{0.0, 0.0};
  Scalar eccentricity_magnitude_m{0.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
  std::string contact_state{"free"};
};

Vector2 select_contact_direction(
    const Vector2& displacement_n_b_m,
    const Vector2& equivalent_force_n_b);

VectorContactResponse evaluate_vector_contact(
    const VectorNodeInput& node,
    const Vector2& displacement_n_b_m);

}  // namespace centraltd
