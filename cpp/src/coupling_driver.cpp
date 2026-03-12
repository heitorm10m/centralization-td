#include "centraltd/coupling_driver.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

std::vector<AxialForcePoint> axial_points_from_loads(
    const std::vector<MechanicalSegmentInput>& segments,
    const std::vector<Scalar>& axial_loads_n) {
  std::vector<AxialForcePoint> profile;
  profile.reserve(segments.size());
  for (Index index = 0; index < segments.size(); ++index) {
    profile.push_back(AxialForcePoint{
        segments.at(index).measured_depth_center_m,
        axial_loads_n.at(index),
    });
  }
  return profile;
}

std::vector<NormalReactionPoint> normal_reaction_points_from_segments(
    const std::vector<MechanicalSegmentResult>& segment_results) {
  std::vector<NormalReactionPoint> profile;
  profile.reserve(segment_results.size());
  for (const auto& segment : segment_results) {
    profile.push_back(NormalReactionPoint{
        segment.measured_depth_center_m,
        segment.support_normal_reaction_estimate_n,
        segment.body_normal_reaction_estimate_n,
        segment.normal_reaction_estimate_n,
    });
  }
  return profile;
}

std::vector<Scalar> axial_loads_from_points(const std::vector<AxialForcePoint>& profile) {
  std::vector<Scalar> loads_n;
  loads_n.reserve(profile.size());
  for (const auto& point : profile) {
    loads_n.push_back(point.axial_force_n);
  }
  return loads_n;
}

Scalar selected_hookload_n(
    const TorqueDragResult& torque_drag_result,
    Scalar reference_buoyant_hookload_n,
    const std::string& operation_mode) {
  if (operation_mode == "pull_out") {
    return torque_drag_result.hookload_pull_out_n;
  }
  if (operation_mode == "rotate_in_place") {
    return reference_buoyant_hookload_n;
  }
  return torque_drag_result.hookload_run_in_n;
}

std::vector<Scalar> selected_axial_profile_n(
    const TorqueDragResult& torque_drag_result,
    const AxialLoadProfile& buoyant_axial_profile,
    const std::string& operation_mode) {
  if (operation_mode == "pull_out") {
    return axial_loads_from_points(torque_drag_result.axial_force_pull_out_profile);
  }
  if (operation_mode == "rotate_in_place") {
    return buoyant_axial_profile.segment_center_loads_n;
  }
  return axial_loads_from_points(torque_drag_result.axial_force_run_in_profile);
}

Scalar maximum_torque_profile_update_n_m(
    const std::vector<TorquePoint>& current_profile,
    const std::vector<TorquePoint>& previous_profile) {
  if (current_profile.size() != previous_profile.size()) {
    return 0.0;
  }

  Scalar maximum_update_n_m = 0.0;
  for (Index index = 0; index < current_profile.size(); ++index) {
    maximum_update_n_m = std::max(
        maximum_update_n_m,
        std::abs(
            current_profile.at(index).local_torque_increment_n_m -
            previous_profile.at(index).local_torque_increment_n_m));
  }
  return maximum_update_n_m;
}

}  // namespace

CouplingDriverResult run_coupled_global_baseline(
    const DiscretizedProblem& problem,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode) {
  CouplingDriverResult result;

  const auto buoyant_axial_profile = compute_buoyant_axial_load_profile(problem.segments);
  std::vector<Scalar> current_axial_profile_n = buoyant_axial_profile.segment_center_loads_n;
  Scalar current_top_axial_load_n = buoyant_axial_profile.top_boundary_load_n;

  for (std::size_t iteration_index = 0;
       iteration_index < problem.settings.coupling_max_iterations;
       ++iteration_index) {
    auto mechanical_result = run_mechanical_with_axial_profile(
        problem,
        current_axial_profile_n,
        current_top_axial_load_n);
    auto torque_drag_result = run_torque_drag_baseline(
        buoyant_axial_profile.top_boundary_load_n,
        mechanical_result.segment_results,
        string_sections,
        operation_mode);

    const auto target_axial_profile_n = selected_axial_profile_n(
        torque_drag_result,
        buoyant_axial_profile,
        operation_mode);
    const Scalar target_top_axial_load_n = selected_hookload_n(
        torque_drag_result,
        buoyant_axial_profile.top_boundary_load_n,
        operation_mode);

    Scalar maximum_profile_update_n = 0.0;
    std::vector<Scalar> relaxed_axial_profile_n(target_axial_profile_n.size(), 0.0);
    for (Index segment_index = 0; segment_index < target_axial_profile_n.size(); ++segment_index) {
      const Scalar relaxed_load_n =
          ((1.0 - problem.settings.relaxation_factor) * current_axial_profile_n.at(segment_index)) +
          (problem.settings.relaxation_factor * target_axial_profile_n.at(segment_index));
      relaxed_axial_profile_n.at(segment_index) = std::max(0.0, relaxed_load_n);
      maximum_profile_update_n = std::max(
          maximum_profile_update_n,
          std::abs(relaxed_axial_profile_n.at(segment_index) - current_axial_profile_n.at(segment_index)));
    }

    result.iteration_count = iteration_index + 1U;
    result.maximum_profile_update_n = maximum_profile_update_n;
    result.mechanical_result = std::move(mechanical_result);
    result.torque_drag_result = std::move(torque_drag_result);
    result.maximum_torque_update_n_m = maximum_torque_profile_update_n_m(
        result.torque_drag_result.torque_profile,
        result.converged_torque_profile);
    result.converged_axial_profile = axial_points_from_loads(
        problem.segments,
        relaxed_axial_profile_n);
    result.converged_normal_reaction_profile = normal_reaction_points_from_segments(
        result.mechanical_result.segment_results);
    result.converged_torque_profile = result.torque_drag_result.torque_profile;

    current_axial_profile_n = std::move(relaxed_axial_profile_n);
    current_top_axial_load_n = std::max(
        0.0,
        ((1.0 - problem.settings.relaxation_factor) * current_top_axial_load_n) +
            (problem.settings.relaxation_factor * target_top_axial_load_n));

    if (maximum_profile_update_n <= problem.settings.coupling_tolerance_n) {
      result.converged = true;
      result.status = "converged";
      break;
    }
  }

  if (!result.converged) {
    result.status = "max_iterations_reached";
  }

  result.mechanical_result = run_mechanical_with_axial_profile(
      problem,
      current_axial_profile_n,
      current_top_axial_load_n);
  result.torque_drag_result = run_torque_drag_baseline(
      buoyant_axial_profile.top_boundary_load_n,
      result.mechanical_result.segment_results,
      string_sections,
      operation_mode);
  result.converged_axial_profile = axial_points_from_loads(problem.segments, current_axial_profile_n);
  result.converged_normal_reaction_profile = normal_reaction_points_from_segments(
      result.mechanical_result.segment_results);
  result.maximum_torque_update_n_m = maximum_torque_profile_update_n_m(
      result.torque_drag_result.torque_profile,
      result.converged_torque_profile);
  result.converged_torque_profile = result.torque_drag_result.torque_profile;
  return result;
}

}  // namespace centraltd
