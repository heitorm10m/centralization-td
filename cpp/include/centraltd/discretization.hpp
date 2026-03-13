#pragma once

#include "centraltd/centralizer.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/types.hpp"
#include "centraltd/well.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct DiscretizationSettings {
  Scalar target_segment_length_m{30.0};
  std::size_t global_solver_max_iterations{8};
  Scalar contact_penalty_scale{25.0};
  std::size_t coupling_max_iterations{6};
  Scalar coupling_tolerance_n{25.0};
  Scalar coupling_torque_tolerance_n_m{5.0};
  Scalar relaxation_factor{0.5};
  std::string frame_method{"parallel-transport"};

  void validate() const;
};

struct CentralizerPlacement {
  std::string source_name;
  std::string type;
  Scalar measured_depth_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar nominal_restoring_force_n{0.0};
  Scalar nominal_running_force_n{0.0};
  std::optional<Scalar> axial_force_ratio;
  std::optional<Scalar> tangential_force_ratio;
  std::size_t number_of_bows{6U};
  Scalar angular_orientation_reference_deg{0.0};
  Scalar inner_clearance_to_pipe_m{0.0};
  std::optional<Scalar> blade_power_law_k;
  Scalar blade_power_law_p{1.0};
  std::optional<Scalar> min_contact_diameter_m;
  std::optional<Scalar> max_contact_diameter_m;
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
  Vector3 tangent_north_east_tvd{0.0, 0.0, 1.0};
  Vector3 normal_north_east_tvd{1.0, 0.0, 0.0};
  Vector3 binormal_north_east_tvd{0.0, 1.0, 0.0};
  Scalar reference_hole_diameter_m{0.0};
  Scalar fluid_density_kg_per_m3{0.0};
  Scalar effective_line_weight_n_per_m{0.0};
  Scalar second_moment_of_area_m4{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  Scalar curvature_normal_component_rad_per_m{0.0};
  Scalar curvature_binormal_component_rad_per_m{0.0};
  Scalar frame_rotation_change_rad{0.0};
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
