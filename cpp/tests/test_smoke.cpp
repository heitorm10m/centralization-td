#include "centraltd/solver_stub.hpp"

#include <cmath>
#include <optional>
#include <vector>

int main() {
  centraltd::WellTrajectory well({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {400.0, 0.08, 0.0, 0.0, 0.0, 0.0},
      {1200.0, 0.35, 0.45, 0.0, 0.0, 0.0},
      {1800.0, 0.60, 0.90, 0.0, 0.0, 0.0},
  });

  std::vector<centraltd::StringSection> sections = {
      {"casing-top", 0.0, 600.0, 0.1778, 0.1524, 680.0, 2.07e11, 8.0e10, 7850.0, 0.24},
      {"casing-mid", 600.0, 1300.0, 0.1778, 0.1524, 700.0, 2.07e11, 8.0e10, 7850.0, 0.26},
      {"casing-bottom", 1300.0, 1750.0, 0.1778, 0.1524, 730.0, 2.07e11, 8.0e10, 7850.0, 0.28},
  };

  std::vector<centraltd::CentralizerSpec> centralizers = {
      {"bow-spring-standard", "bow-spring", 0.208, 1800.0, 900.0, 25.0, 18, {}},
      {"bow-spring-targeted", "bow-spring", 0.208, 2200.0, 1050.0, 15.0, std::nullopt, {1500.0, 1600.0, 1700.0}},
  };

  const auto interpolated = well.interpolate(900.0);
  const auto trajectory_summary = well.summary();
  if (!(interpolated.tvd_m > 0.0)) {
    return 1;
  }
  if (trajectory_summary.point_count != 4U) {
    return 1;
  }
  if (!(trajectory_summary.max_curvature_rad_per_m >= 0.0)) {
    return 1;
  }

  centraltd::SolverStubInput input{well, 0.216, sections, centralizers};
  const auto result = centraltd::run_solver_stub(input);

  if (result.status != "phase2-baseline") {
    return 1;
  }
  if (result.todos.size() != 6) {
    return 1;
  }
  if (result.warnings.size() < 3U) {
    return 1;
  }
  if (result.estimated_hookload_n <= 0.0) {
    return 1;
  }
  if (!(result.minimum_standoff_ratio > 0.0 && result.minimum_standoff_ratio <= 1.0)) {
    return 1;
  }
  if (result.section_summaries.size() != 3U) {
    return 1;
  }
  if (std::isnan(result.estimated_surface_torque_n_m)) {
    return 1;
  }
  if (!(result.trajectory_summary.vertical_depth_m > 0.0)) {
    return 1;
  }
  if (result.centralizer_summary.explicit_installation_count != 3U) {
    return 1;
  }

  return 0;
}
