#pragma once

#include "centraltd/global_assembly.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct GlobalNodeSolution {
  Scalar measured_depth_m{0.0};
  Scalar eccentricity_estimate_m{0.0};
  Scalar eccentricity_ratio{0.0};
  Scalar standoff_estimate{1.0};
  Scalar support_normal_reaction_estimate_n{0.0};
  Scalar body_normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n{0.0};
  Scalar normal_reaction_estimate_n_per_m{0.0};
  bool support_in_contact{false};
  bool pipe_body_in_contact{false};
  std::string contact_state{"free"};
};

struct GlobalSolverResult {
  std::vector<GlobalNodeSolution> node_solutions;
  std::vector<GlobalContactState> contact_states;
  std::size_t iteration_count{0};
  Scalar final_update_norm_m{0.0};
};

GlobalSolverResult run_global_lateral_solver(
    const std::vector<GlobalNodeInput>& nodes,
    const DiscretizationSettings& settings);

}  // namespace centraltd
