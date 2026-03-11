#pragma once

#include "centraltd/mechanical_solver.hpp"
#include "centraltd/torque_drag.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct CouplingDriverResult {
  MechanicalSolverResult mechanical_result;
  TorqueDragResult torque_drag_result;
  std::vector<AxialForcePoint> converged_axial_profile;
  std::vector<NormalReactionPoint> converged_normal_reaction_profile;
  std::vector<TorquePoint> converged_torque_profile;
  std::size_t iteration_count{0};
  bool converged{false};
  std::string status{"max_iterations_reached"};
};

CouplingDriverResult run_coupled_global_baseline(
    const DiscretizedProblem& problem,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode);

}  // namespace centraltd
