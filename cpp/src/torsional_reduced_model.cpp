#include "centraltd/torsional_reduced_model.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

const StringSection& section_for_name(
    const std::vector<StringSection>& string_sections,
    const std::string& section_name) {
  for (const auto& section : string_sections) {
    if (section.name == section_name) {
      return section;
    }
  }

  throw ValidationError("Reduced torsional model could not resolve the string section name.");
}

}  // namespace

TorsionalReducedResult run_reduced_torsional_model(
    const std::vector<MechanicalSegmentResult>& mechanical_profile,
    const std::vector<StringSection>& string_sections,
    const std::vector<TorquePoint>& torque_profile,
    const std::vector<Scalar>& carried_torsional_load_profile_n_m) {
  if (mechanical_profile.size() != torque_profile.size()) {
    throw ValidationError(
        "Reduced torsional model requires mechanical and torque profiles with matching sizes.");
  }
  if (!carried_torsional_load_profile_n_m.empty() &&
      carried_torsional_load_profile_n_m.size() != torque_profile.size()) {
    throw ValidationError(
        "Reduced torsional model received a carried torsional-load profile with incompatible size.");
  }

  TorsionalReducedResult result;
  result.reduced_torque_accumulation_profile.resize(torque_profile.size());
  result.torsional_state_profile.resize(torque_profile.size());

  Scalar cumulative_twist_below_rad = 0.0;
  for (Index reverse_index = torque_profile.size(); reverse_index > 0U; --reverse_index) {
    const Index segment_index = reverse_index - 1U;
    const auto& segment = mechanical_profile.at(segment_index);
    const auto& torque_point = torque_profile.at(segment_index);
    const auto& section = section_for_name(string_sections, segment.section_name);

    const Scalar raw_cumulative_torque_n_m =
        std::max(0.0, torque_point.cumulative_torque_n_m);
    const Scalar carried_torsional_load_n_m =
        carried_torsional_load_profile_n_m.empty()
            ? raw_cumulative_torque_n_m
            : std::max(0.0, carried_torsional_load_profile_n_m.at(segment_index));
    const Scalar torsional_stiffness_n_m2 =
        std::max(0.0, section.torsional_stiffness_n_m2());
    const Scalar reduced_twist_rate_rad_per_m =
        torsional_stiffness_n_m2 > 1.0e-12
            ? (carried_torsional_load_n_m / torsional_stiffness_n_m2)
            : 0.0;
    const Scalar reduced_twist_increment_rad =
        reduced_twist_rate_rad_per_m * segment.segment_length_m;
    const std::string point_status =
        carried_torsional_load_n_m > 0.0 || raw_cumulative_torque_n_m > 0.0
            ? "phase11-reduced-torsional-load-and-twist-state"
            : "inactive";

    result.reduced_torque_accumulation_profile.at(segment_index) =
        ReducedTorqueAccumulationPoint{
            segment.measured_depth_center_m,
            torque_point.body_torque_increment_n_m,
            torque_point.centralizer_torque_increment_n_m,
            torque_point.local_torque_increment_n_m,
            raw_cumulative_torque_n_m,
            carried_torsional_load_n_m,
            point_status,
        };
    result.torsional_state_profile.at(segment_index) = TorsionalStatePoint{
        segment.measured_depth_center_m,
        segment.segment_length_m,
        segment.section_name,
        torque_point.body_torque_increment_n_m,
        torque_point.centralizer_torque_increment_n_m,
        torque_point.local_torque_increment_n_m,
        carried_torsional_load_n_m,
        torsional_stiffness_n_m2,
        reduced_twist_rate_rad_per_m,
        reduced_twist_increment_rad,
        cumulative_twist_below_rad + (0.5 * reduced_twist_increment_rad),
        point_status,
    };
    cumulative_twist_below_rad += reduced_twist_increment_rad;
    result.maximum_cumulative_reduced_twist_rad = std::max(
        result.maximum_cumulative_reduced_twist_rad,
        std::abs(result.torsional_state_profile.at(segment_index).cumulative_reduced_twist_rad));
  }

  if (!result.torsional_state_profile.empty()) {
    result.estimated_surface_torsional_load_n_m =
        result.torsional_state_profile.front().reduced_torsional_load_n_m;
    result.status = "phase11-reduced-torsional-load-and-twist-state";
  }

  return result;
}

}  // namespace centraltd
