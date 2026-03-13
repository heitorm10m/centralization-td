#pragma once

#include "centraltd/mechanical_solver.hpp"
#include "centraltd/torsional_reduced_model.hpp"
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
  std::vector<ReducedTorqueAccumulationPoint> reduced_torque_accumulation_profile;
  std::vector<TorsionalStatePoint> torsional_state_profile;
  std::size_t iteration_count{0};
  Scalar maximum_profile_update_n{0.0};
  Scalar maximum_torque_update_n_m{0.0};
  Scalar maximum_torsional_load_update_n_m{0.0};
  bool converged{false};
  std::string status{"max_iterations_reached"};
  std::string torque_feedback_mode{
      "reduced-unified-local-tangential-state-fed-by-carried-torsional-state-plus-centralizer-axial-tangential-budget-and-convergence"};
  std::string torsional_feedback_status{
      "inactive"};
};

CouplingDriverResult run_coupled_global_baseline(
    const DiscretizedProblem& problem,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode);

}  // namespace centraltd
