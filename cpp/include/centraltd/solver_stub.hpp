#pragma once

#include "centraltd/centralizer.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/well.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct SolverStubInput {
  WellTrajectory well;
  std::vector<StringSection> string_sections;
  std::vector<CentralizerSpec> centralizers;

  void validate() const;
};

struct SolverStubResult {
  std::string status{"stub"};
  std::string message;
  Scalar estimated_hookload_n{0.0};
  Scalar estimated_surface_torque_n_m{0.0};
  Scalar minimum_standoff_ratio{0.0};
  std::size_t contact_nodes{0};
  std::vector<std::string> todos;
};

SolverStubResult run_solver_stub(const SolverStubInput& input);

}  // namespace centraltd

