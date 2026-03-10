#pragma once

#include "centraltd/centralizer.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/well.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct DiscretizationSettings {
  Scalar target_segment_length_m{30.0};
  std::size_t global_solver_max_iterations{8};
  Scalar contact_penalty_scale{25.0};

  void validate() const;
};

struct CentralizerPlacement {
  std::string source_name;
  std::string type;
  Scalar measured_depth_m{0.0};
  Scalar outer_diameter_m{0.0};
  Scalar nominal_restoring_force_n{0.0};
  Scalar nominal_running_force_n{0.0};
  Scalar influence_length_m{0.0};
};

struct MechanicalSegmentInput {
  Index segment_index{0};
  Index section_index{0};
  Scalar measured_depth_start_m{0.0};
  Scalar measured_depth_end_m{0.0};
  Scalar measured_depth_center_m{0.0};
  Scalar segment_length_m{0.0};
  Scalar inclination_rad{0.0};
  Scalar azimuth_rad{0.0};
  Scalar curvature_rad_per_m{0.0};
  Scalar tvd_m{0.0};
  Scalar northing_m{0.0};
  Scalar easting_m{0.0};
  Scalar reference_hole_diameter_m{0.0};
  Scalar fluid_density_kg_per_m3{0.0};
  Scalar effective_line_weight_n_per_m{0.0};
  Scalar second_moment_of_area_m4{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  StringSection section;
};

struct DiscretizedProblem {
  DiscretizationSettings settings;
  TrajectorySummary trajectory_summary;
  Scalar coverage_start_md_m{0.0};
  Scalar coverage_end_md_m{0.0};
  std::vector<CentralizerPlacement> centralizer_placements;
  std::vector<MechanicalSegmentInput> segments;
};

std::vector<CentralizerPlacement> expand_centralizer_placements(
    const std::vector<CentralizerSpec>& centralizers,
    Scalar coverage_start_md_m,
    Scalar coverage_end_md_m);

DiscretizedProblem discretize_problem(
    const WellTrajectory& well,
    Scalar reference_hole_diameter_m,
    Scalar fluid_density_kg_per_m3,
    const DiscretizationSettings& settings,
    const std::vector<StringSection>& string_sections,
    const std::vector<CentralizerSpec>& centralizers);

}  // namespace centraltd
