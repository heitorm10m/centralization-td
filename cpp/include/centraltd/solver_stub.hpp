#pragma once

#include "centraltd/centralizer.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/well.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct SolverStubInput {
  WellTrajectory well;
  Scalar reference_hole_diameter_m{0.0};
  std::vector<StringSection> string_sections;
  std::vector<CentralizerSpec> centralizers;

  void validate() const;
};

struct StringSectionSummary {
  std::string name;
  Scalar md_start_m{0.0};
  Scalar md_end_m{0.0};
  Scalar length_m{0.0};
  Scalar outer_diameter_m{0.0};
  Scalar inner_diameter_m{0.0};
  Scalar linear_weight_n_per_m{0.0};
  Scalar friction_coefficient{0.0};
  Scalar nominal_radial_clearance_m{0.0};
};

struct StringSummary {
  std::size_t section_count{0};
  Scalar total_length_m{0.0};
  Scalar total_weight_n{0.0};
  Scalar max_outer_diameter_m{0.0};
  Scalar min_inner_diameter_m{0.0};
  Scalar average_friction_coefficient{0.0};
  Scalar average_density_kg_per_m3{0.0};
};

struct CentralizerSummary {
  std::size_t spec_count{0};
  std::size_t explicit_installation_count{0};
  std::size_t count_hint_total{0};
  std::size_t spacing_based_installation_estimate{0};
  Scalar max_outer_diameter_m{0.0};
  Scalar min_nominal_radial_clearance_m{0.0};
};

struct SolverStubResult {
  std::string status{"phase2-baseline"};
  std::string message;
  bool geometry_is_approximate{true};
  TrajectorySummary trajectory_summary;
  StringSummary string_summary;
  CentralizerSummary centralizer_summary;
  std::vector<StringSectionSummary> section_summaries;
  Scalar estimated_hookload_n{0.0};
  Scalar estimated_surface_torque_n_m{0.0};
  Scalar minimum_standoff_ratio{0.0};
  Scalar minimum_nominal_radial_clearance_m{0.0};
  std::size_t contact_nodes{0};
  std::vector<std::string> warnings;
  std::vector<std::string> todos;
};

SolverStubResult run_solver_stub(const SolverStubInput& input);

}  // namespace centraltd
