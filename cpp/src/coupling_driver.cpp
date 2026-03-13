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

std::vector<Scalar> torsional_loads_from_points(
    const std::vector<ReducedTorqueAccumulationPoint>& profile) {
  std::vector<Scalar> loads_n_m;
  loads_n_m.reserve(profile.size());
  for (const auto& point : profile) {
    loads_n_m.push_back(point.carried_torsional_load_n_m);
  }
  return loads_n_m;
}

std::vector<Scalar> twist_rates_from_points(
    const std::vector<TorsionalStatePoint>& profile) {
  std::vector<Scalar> twist_rates_rad_per_m;
  twist_rates_rad_per_m.reserve(profile.size());
  for (const auto& point : profile) {
    twist_rates_rad_per_m.push_back(point.reduced_twist_rate_rad_per_m);
  }
  return twist_rates_rad_per_m;
}

std::vector<TorquePoint> zero_torque_profile(
    const std::vector<MechanicalSegmentResult>& segment_results) {
  std::vector<TorquePoint> profile;
  profile.reserve(segment_results.size());
  for (const auto& segment : segment_results) {
    profile.push_back(TorquePoint{
        segment.measured_depth_center_m,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    });
  }
  return profile;
}

bool any_active_local_tangential_feedback(
    const std::vector<LocalTangentialInteractionStatePoint>& profile) {
  return std::any_of(
      profile.begin(),
      profile.end(),
      [](const LocalTangentialInteractionStatePoint& point) {
        return point.body_feedback_applied || point.centralizer_feedback_applied;
      });
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

Scalar maximum_profile_update_n_m(
    const std::vector<Scalar>& current_profile,
    const std::vector<Scalar>& previous_profile) {
  if (current_profile.size() != previous_profile.size()) {
    return 0.0;
  }

  Scalar maximum_update_n_m = 0.0;
  for (Index index = 0; index < current_profile.size(); ++index) {
    maximum_update_n_m = std::max(
        maximum_update_n_m,
        std::abs(current_profile.at(index) - previous_profile.at(index)));
  }
  return maximum_update_n_m;
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
  std::vector<Scalar> current_torsional_load_profile_n_m(problem.segments.size(), 0.0);

  for (std::size_t iteration_index = 0;
       iteration_index < problem.settings.coupling_max_iterations;
       ++iteration_index) {
    auto mechanical_result = run_mechanical_with_axial_profile(
        problem,
        current_axial_profile_n,
        current_top_axial_load_n);
    const auto current_torsional_state_result = run_reduced_torsional_model(
        mechanical_result.segment_results,
        string_sections,
        zero_torque_profile(mechanical_result.segment_results),
        current_torsional_load_profile_n_m);
    auto torque_drag_result = run_torque_drag_baseline(
        buoyant_axial_profile.top_boundary_load_n,
        mechanical_result.segment_results,
        string_sections,
        operation_mode,
        current_torsional_load_profile_n_m,
        twist_rates_from_points(current_torsional_state_result.torsional_state_profile));
    const auto target_torsional_result = run_reduced_torsional_model(
        mechanical_result.segment_results,
        string_sections,
        torque_drag_result.torque_profile);
    const auto target_torsional_load_profile_n_m = torsional_loads_from_points(
        target_torsional_result.reduced_torque_accumulation_profile);

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
    std::vector<Scalar> relaxed_torsional_load_profile_n_m(
        target_torsional_load_profile_n_m.size(),
        0.0);
    Scalar maximum_torsional_load_update_n_m = 0.0;
    if (iteration_index == 0U) {
      // The first torsional profile is an initialization step, not a response
      // update from a previously carried torsional state.
      relaxed_torsional_load_profile_n_m = target_torsional_load_profile_n_m;
    } else {
      for (Index segment_index = 0;
           segment_index < target_torsional_load_profile_n_m.size();
           ++segment_index) {
        const Scalar relaxed_load_n_m =
            ((1.0 - problem.settings.relaxation_factor) *
                 current_torsional_load_profile_n_m.at(segment_index)) +
            (problem.settings.relaxation_factor *
                 target_torsional_load_profile_n_m.at(segment_index));
        relaxed_torsional_load_profile_n_m.at(segment_index) = std::max(0.0, relaxed_load_n_m);
        maximum_torsional_load_update_n_m = std::max(
            maximum_torsional_load_update_n_m,
            std::abs(
                relaxed_torsional_load_profile_n_m.at(segment_index) -
                current_torsional_load_profile_n_m.at(segment_index)));
      }
    }
    const auto carried_torsional_result = run_reduced_torsional_model(
        mechanical_result.segment_results,
        string_sections,
        torque_drag_result.torque_profile,
        relaxed_torsional_load_profile_n_m);

    result.iteration_count = iteration_index + 1U;
    result.maximum_profile_update_n = maximum_profile_update_n;
    result.mechanical_result = std::move(mechanical_result);
    result.torque_drag_result = std::move(torque_drag_result);
    result.maximum_torque_update_n_m = maximum_torque_profile_update_n_m(
        result.torque_drag_result.torque_profile,
        result.converged_torque_profile);
    result.maximum_torsional_load_update_n_m = maximum_torsional_load_update_n_m;
    result.converged_axial_profile = axial_points_from_loads(
        problem.segments,
        relaxed_axial_profile_n);
    result.converged_normal_reaction_profile = normal_reaction_points_from_segments(
        result.mechanical_result.segment_results);
    result.converged_torque_profile = result.torque_drag_result.torque_profile;
    result.reduced_torque_accumulation_profile =
        carried_torsional_result.reduced_torque_accumulation_profile;
    result.torsional_state_profile = carried_torsional_result.torsional_state_profile;
    result.torsional_feedback_status =
        any_active_local_tangential_feedback(
            result.torque_drag_result.local_tangential_interaction_state)
            ? "phase14-reduced-torsional-state-fed-into-unified-local-tangential-state"
            : current_torsional_state_result.status;

    current_axial_profile_n = std::move(relaxed_axial_profile_n);
    current_torsional_load_profile_n_m = std::move(relaxed_torsional_load_profile_n_m);
    current_top_axial_load_n = std::max(
        0.0,
        ((1.0 - problem.settings.relaxation_factor) * current_top_axial_load_n) +
            (problem.settings.relaxation_factor * target_top_axial_load_n));

    if (maximum_profile_update_n <= problem.settings.coupling_tolerance_n &&
        result.maximum_torque_update_n_m <= problem.settings.coupling_torque_tolerance_n_m &&
        result.maximum_torsional_load_update_n_m <=
            problem.settings.coupling_torque_tolerance_n_m) {
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
  const auto current_torsional_state_result = run_reduced_torsional_model(
      result.mechanical_result.segment_results,
      string_sections,
      zero_torque_profile(result.mechanical_result.segment_results),
      current_torsional_load_profile_n_m);
  result.torque_drag_result = run_torque_drag_baseline(
      buoyant_axial_profile.top_boundary_load_n,
      result.mechanical_result.segment_results,
      string_sections,
      operation_mode,
      current_torsional_load_profile_n_m,
      twist_rates_from_points(current_torsional_state_result.torsional_state_profile));
  const auto target_torsional_result = run_reduced_torsional_model(
      result.mechanical_result.segment_results,
      string_sections,
      result.torque_drag_result.torque_profile);
  const auto target_torsional_load_profile_n_m = torsional_loads_from_points(
      target_torsional_result.reduced_torque_accumulation_profile);
  result.converged_axial_profile = axial_points_from_loads(problem.segments, current_axial_profile_n);
  result.converged_normal_reaction_profile = normal_reaction_points_from_segments(
      result.mechanical_result.segment_results);
  result.maximum_torque_update_n_m = maximum_torque_profile_update_n_m(
      result.torque_drag_result.torque_profile,
      result.converged_torque_profile);
  result.maximum_torsional_load_update_n_m = maximum_profile_update_n_m(
      target_torsional_load_profile_n_m,
      current_torsional_load_profile_n_m);
  result.converged_torque_profile = result.torque_drag_result.torque_profile;
  if (current_torsional_load_profile_n_m.size() != target_torsional_load_profile_n_m.size()) {
    current_torsional_load_profile_n_m = target_torsional_load_profile_n_m;
  }
  const auto carried_torsional_result = run_reduced_torsional_model(
      result.mechanical_result.segment_results,
      string_sections,
      result.torque_drag_result.torque_profile,
      current_torsional_load_profile_n_m);
  result.reduced_torque_accumulation_profile =
      carried_torsional_result.reduced_torque_accumulation_profile;
  result.torsional_state_profile = carried_torsional_result.torsional_state_profile;
  result.torsional_feedback_status =
      any_active_local_tangential_feedback(
          result.torque_drag_result.local_tangential_interaction_state)
          ? "phase14-reduced-torsional-state-fed-into-unified-local-tangential-state"
          : carried_torsional_result.status;
  return result;
}

}  // namespace centraltd
