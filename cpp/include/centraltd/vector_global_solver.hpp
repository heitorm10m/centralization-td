#pragma once

#include "centraltd/discretization.hpp"
#include "centraltd/vector_contact.hpp"
#include "centraltd/vector_global_assembly.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct VectorNodeSolution {
  Scalar measured_depth_m{0.0};
  Scalar lateral_displacement_normal_m{0.0};
  Scalar lateral_displacement_binormal_m{0.0};
  Scalar eccentricity_estimate_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  Vector2 contact_direction_n_b{1.0, 0.0};
  Vector2 support_normal_reaction_vector_n_b{0.0, 0.0};
  Vector2 body_normal_reaction_vector_n_b{0.0, 0.0};
  Vector2 normal_reaction_vector_n_b{0.0, 0.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
  std::string contact_state{"free"};
};

struct VectorGlobalSolverResult {
  std::vector<VectorNodeSolution> node_solutions;
  std::vector<VectorContactState> contact_states;
  std::size_t iteration_count{0};
  Scalar final_update_norm_m{0.0};
};

VectorGlobalSolverResult run_vector_global_lateral_solver(
    const std::vector<VectorNodeInput>& nodes,
    const DiscretizationSettings& settings);

}  // namespace centraltd
