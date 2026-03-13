#pragma once

#include "centraltd/torque_drag.hpp"

#include <string>
#include <vector>

namespace centraltd {

struct ReducedTorqueAccumulationPoint {
  Scalar measured_depth_m{0.0};
  Scalar body_torque_increment_n_m{0.0};
  Scalar centralizer_torque_increment_n_m{0.0};
  Scalar local_torque_increment_n_m{0.0};
  Scalar raw_cumulative_torque_n_m{0.0};
  Scalar carried_torsional_load_n_m{0.0};
  std::string status{"inactive"};
};

struct TorsionalStatePoint {
  Scalar measured_depth_m{0.0};
  Scalar segment_length_m{0.0};
  std::string section_name;
  Scalar body_torque_increment_n_m{0.0};
  Scalar centralizer_torque_increment_n_m{0.0};
  Scalar local_torque_increment_n_m{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar section_torsional_stiffness_n_m2{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar reduced_twist_increment_rad{0.0};
  Scalar cumulative_reduced_twist_rad{0.0};
  std::string status{"inactive"};
};

struct TorsionalReducedResult {
  std::string status{"inactive"};
  std::vector<ReducedTorqueAccumulationPoint> reduced_torque_accumulation_profile;
  std::vector<TorsionalStatePoint> torsional_state_profile;
  Scalar estimated_surface_torsional_load_n_m{0.0};
  Scalar maximum_cumulative_reduced_twist_rad{0.0};
};

// The torsional layer remains reduced: it uses the local torque partition from
// the current torque-and-drag pass and a tube torsional rigidity GJ to build a
// propagated torsional-load profile plus a reduced twist indicator. When a
// carried load profile is provided, that profile is interpreted as the relaxed
// torsional state being fed back through the coupling loop.
TorsionalReducedResult run_reduced_torsional_model(
    const std::vector<MechanicalSegmentResult>& mechanical_profile,
    const std::vector<StringSection>& string_sections,
    const std::vector<TorquePoint>& torque_profile,
    const std::vector<Scalar>& carried_torsional_load_profile_n_m = {});

}  // namespace centraltd
