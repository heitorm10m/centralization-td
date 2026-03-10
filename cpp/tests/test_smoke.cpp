#include "centraltd/solver_stub.hpp"

#include <cmath>
#include <vector>

int main() {
  centraltd::WellTrajectory well({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {500.0, 0.1, 0.0, 497.0, 50.0, 0.0},
      {1500.0, 0.4, 0.7, 1380.0, 420.0, 400.0},
  });

  std::vector<centraltd::StringSection> sections = {
      {"dp", 1200.0, 0.1397, 0.1086, 330.0, "S-135"},
      {"hwdp", 300.0, 0.1778, 0.0762, 550.0, "S-135"},
  };

  std::vector<centraltd::CentralizerSpec> centralizers = {
      {"bow-spring-01", 0.216, 900.0, 30.0, 12, "bow-spring"},
  };

  centraltd::SolverStubInput input{well, sections, centralizers};
  const auto result = centraltd::run_solver_stub(input);

  if (result.status != "stub") {
    return 1;
  }
  if (result.todos.size() != 6) {
    return 1;
  }
  if (result.estimated_hookload_n <= 0.0) {
    return 1;
  }
  if (!(result.minimum_standoff_ratio > 0.0 && result.minimum_standoff_ratio <= 0.95)) {
    return 1;
  }
  if (result.contact_nodes != 2U) {
    return 1;
  }
  if (std::isnan(result.estimated_surface_torque_n_m)) {
    return 1;
  }

  return 0;
}
