#include "centraltd/torque_drag.hpp"

#include "centraltd/torque_drag_body.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr char kRunInMode[] = "run_in";
constexpr char kPullOutMode[] = "pull_out";
constexpr char kRotateInPlaceMode[] = "rotate_in_place";

const StringSection& section_for_name(
    const std::vector<StringSection>& string_sections,
    const std::string& section_name) {
  for (const auto& section : string_sections) {
    if (section.name == section_name) {
      return section;
    }
  }

  throw ValidationError("Torque and drag propagation could not resolve the string section name.");
}

Scalar segment_tangential_weight_n(const MechanicalSegmentResult& segment) {
  return segment.effective_line_weight_n_per_m *
         std::cos(segment.inclination_rad) *
         segment.segment_length_m;
}

}  // namespace

bool is_supported_operation_mode(const std::string& operation_mode) {
  return operation_mode == kRunInMode ||
         operation_mode == kPullOutMode ||
         operation_mode == kRotateInPlaceMode;
}

TorqueDragResult run_torque_drag_baseline(
    Scalar reference_buoyant_hookload_n,
    const std::vector<MechanicalSegmentResult>& mechanical_profile,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode) {
  if (!is_supported_operation_mode(operation_mode)) {
    throw ValidationError("Unsupported operation mode for reduced torque and drag.");
  }

  TorqueDragResult result;
  result.operation_mode = operation_mode;
  result.axial_force_run_in_profile.resize(mechanical_profile.size());
  result.axial_force_pull_out_profile.resize(mechanical_profile.size());
  result.torque_profile.resize(mechanical_profile.size());
  result.body_axial_friction_profile.resize(mechanical_profile.size());
  result.body_torque_profile.resize(mechanical_profile.size());
  result.centralizer_axial_friction_profile.resize(mechanical_profile.size());
  result.centralizer_tangential_friction_profile.resize(mechanical_profile.size());
  result.centralizer_tangential_friction_vector_profile.resize(mechanical_profile.size());
  result.centralizer_torque_profile.resize(mechanical_profile.size());

  Scalar axial_force_run_in_below_n = 0.0;
  Scalar axial_force_pull_out_below_n = 0.0;
  Scalar cumulative_surface_torque_n_m = 0.0;
  Scalar cumulative_body_surface_torque_n_m = 0.0;
  Scalar cumulative_centralizer_surface_torque_n_m = 0.0;

  for (Index reverse_index = mechanical_profile.size(); reverse_index > 0U; --reverse_index) {
    const Index segment_index = reverse_index - 1U;
    const auto& segment = mechanical_profile.at(segment_index);
    const auto& section = section_for_name(string_sections, segment.section_name);

    const Scalar tangential_weight_n = segment_tangential_weight_n(segment);
    // Sign convention:
    // - run_in/slackoff: downward motion, so friction acts upward and reduces
    //   the surface hookload increment.
    // - pull_out/pickup: upward motion, so friction acts downward and
    //   increases the surface hookload increment.
    const auto body_contribution = evaluate_body_torque_drag_contribution(segment, section);
    const Scalar body_friction_force_n = body_contribution.axial_friction_n;
    const Scalar centralizer_axial_friction_n =
        std::max(0.0, segment.centralizer_axial_friction_n);
    const Scalar delta_run_in_n =
        tangential_weight_n - body_friction_force_n - centralizer_axial_friction_n;
    const Scalar delta_pull_out_n =
        tangential_weight_n + body_friction_force_n + centralizer_axial_friction_n;

    result.axial_force_run_in_profile.at(segment_index) = AxialForcePoint{
        segment.measured_depth_center_m,
        axial_force_run_in_below_n + (0.5 * delta_run_in_n),
    };
    result.axial_force_pull_out_profile.at(segment_index) = AxialForcePoint{
        segment.measured_depth_center_m,
        axial_force_pull_out_below_n + (0.5 * delta_pull_out_n),
    };

    axial_force_run_in_below_n += delta_run_in_n;
    axial_force_pull_out_below_n += delta_pull_out_n;

    result.body_axial_friction_profile.at(segment_index) = BodyFrictionPoint{
        segment.measured_depth_center_m,
        body_contribution.axial_friction_n,
        body_contribution.tangential_friction_n,
    };
    result.centralizer_axial_friction_profile.at(segment_index) = CentralizerFrictionPoint{
        segment.measured_depth_center_m,
        centralizer_axial_friction_n,
        std::max(0.0, segment.centralizer_tangential_friction_n),
    };
    result.centralizer_tangential_friction_profile.at(segment_index) = CentralizerFrictionPoint{
        segment.measured_depth_center_m,
        centralizer_axial_friction_n,
        std::max(0.0, segment.centralizer_tangential_friction_n),
    };
    result.centralizer_tangential_friction_vector_profile.at(segment_index) =
        CentralizerTangentialVectorContribution{
            segment.measured_depth_center_m,
            segment.centralizer_tangential_direction_normal,
            segment.centralizer_tangential_direction_binormal,
            segment.centralizer_tangential_friction_normal_n,
            segment.centralizer_tangential_friction_binormal_n,
            segment.centralizer_tangential_friction_vector_magnitude_n,
        };

    const Scalar body_torque_increment_n_m =
        body_contribution.torque_increment_n_m;
    const Scalar centralizer_torque_increment_n_m =
        std::max(0.0, segment.centralizer_torque_increment_n_m);
    const Scalar local_torque_increment_n_m =
        body_torque_increment_n_m + centralizer_torque_increment_n_m;
    result.body_torque_profile.at(segment_index) = TorquePoint{
        segment.measured_depth_center_m,
        body_contribution.contact_radius_m,
        body_torque_increment_n_m,
        0.0,
        body_torque_increment_n_m,
        cumulative_body_surface_torque_n_m + (0.5 * body_torque_increment_n_m),
    };
    result.centralizer_torque_profile.at(segment_index) = TorquePoint{
        segment.measured_depth_center_m,
        segment.centralizer_effective_contact_radius_m,
        0.0,
        centralizer_torque_increment_n_m,
        centralizer_torque_increment_n_m,
        cumulative_centralizer_surface_torque_n_m + (0.5 * centralizer_torque_increment_n_m),
    };
    result.torque_profile.at(segment_index) = TorquePoint{
        segment.measured_depth_center_m,
        std::max(body_contribution.contact_radius_m, segment.centralizer_effective_contact_radius_m),
        body_torque_increment_n_m,
        centralizer_torque_increment_n_m,
        local_torque_increment_n_m,
        cumulative_surface_torque_n_m + (0.5 * local_torque_increment_n_m),
    };
    cumulative_surface_torque_n_m += local_torque_increment_n_m;
    cumulative_body_surface_torque_n_m += body_torque_increment_n_m;
    cumulative_centralizer_surface_torque_n_m += centralizer_torque_increment_n_m;
    result.torque_partition_summary.body_axial_friction_sum_n += body_contribution.axial_friction_n;
    result.torque_partition_summary.centralizer_axial_friction_sum_n += centralizer_axial_friction_n;
    result.torque_partition_summary.body_tangential_friction_sum_n +=
        body_contribution.tangential_friction_n;
    result.torque_partition_summary.centralizer_tangential_friction_sum_n +=
        std::max(0.0, segment.centralizer_tangential_friction_n);
  }

  result.hookload_run_in_n = std::max(0.0, axial_force_run_in_below_n);
  result.hookload_pull_out_n = std::max(0.0, axial_force_pull_out_below_n);
  result.drag_run_in_n = std::max(
      0.0,
      reference_buoyant_hookload_n - result.hookload_run_in_n);
  result.drag_pull_out_n = std::max(
      0.0,
      result.hookload_pull_out_n - reference_buoyant_hookload_n);
  result.torque_partition_summary.body_surface_torque_n_m =
      cumulative_body_surface_torque_n_m;
  result.torque_partition_summary.centralizer_surface_torque_n_m =
      cumulative_centralizer_surface_torque_n_m;
  result.torque_partition_summary.total_surface_torque_n_m =
      cumulative_surface_torque_n_m;
  result.estimated_surface_torque_n_m = cumulative_surface_torque_n_m;
  return result;
}

}  // namespace centraltd
