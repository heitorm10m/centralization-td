#include "centraltd/torque_drag.hpp"

#include "centraltd/torque_drag_body.hpp"

#include <algorithm>
#include <cmath>
#include <map>

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

struct CentralizerPlacementKey {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};

  bool operator<(const CentralizerPlacementKey& other) const {
    if (source_name != other.source_name) {
      return source_name < other.source_name;
    }
    return placement_measured_depth_m < other.placement_measured_depth_m;
  }
};

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
    const std::string& operation_mode,
    const std::vector<Scalar>& reduced_torsional_load_profile_n_m,
    const std::vector<Scalar>& reduced_twist_rate_profile_rad_per_m) {
  if (!is_supported_operation_mode(operation_mode)) {
    throw ValidationError("Unsupported operation mode for reduced torque and drag.");
  }
  if (!reduced_torsional_load_profile_n_m.empty() &&
      reduced_torsional_load_profile_n_m.size() != mechanical_profile.size()) {
    throw ValidationError(
        "Reduced torsional-load feedback profile must match the mechanical profile size.");
  }
  if (!reduced_twist_rate_profile_rad_per_m.empty() &&
      reduced_twist_rate_profile_rad_per_m.size() != mechanical_profile.size()) {
    throw ValidationError(
        "Reduced twist-rate feedback profile must match the mechanical profile size.");
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
  result.centralizer_tangential_direction_profile.resize(mechanical_profile.size());
  result.centralizer_tangential_friction_vector_profile.resize(mechanical_profile.size());
  result.centralizer_torque_profile.resize(mechanical_profile.size());
  result.centralizer_torque_breakdown_profile.resize(mechanical_profile.size());
  result.local_tangential_interaction_state.resize(mechanical_profile.size());

  Scalar axial_force_run_in_below_n = 0.0;
  Scalar axial_force_pull_out_below_n = 0.0;
  Scalar cumulative_surface_torque_n_m = 0.0;
  Scalar cumulative_body_surface_torque_n_m = 0.0;
  Scalar cumulative_centralizer_surface_torque_n_m = 0.0;
  std::map<CentralizerPlacementKey, Scalar> cumulative_placement_torque_n_m;

  for (Index reverse_index = mechanical_profile.size(); reverse_index > 0U; --reverse_index) {
    const Index segment_index = reverse_index - 1U;
    const auto& segment = mechanical_profile.at(segment_index);
    const auto& section = section_for_name(string_sections, segment.section_name);
    const Scalar reduced_torsional_load_n_m =
        reduced_torsional_load_profile_n_m.empty()
            ? 0.0
            : std::max(0.0, reduced_torsional_load_profile_n_m.at(segment_index));
    const Scalar reduced_twist_rate_rad_per_m =
        reduced_twist_rate_profile_rad_per_m.empty()
            ? 0.0
            : std::abs(reduced_twist_rate_profile_rad_per_m.at(segment_index));

    const Scalar tangential_weight_n = segment_tangential_weight_n(segment);
    // Sign convention:
    // - run_in/slackoff: downward motion, so friction acts upward and reduces
    //   the surface hookload increment.
    // - pull_out/pickup: upward motion, so friction acts downward and
    //   increases the surface hookload increment.
    const auto body_contribution = evaluate_body_torque_drag_contribution(
        segment,
        section,
        reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m);
    const auto centralizer_contribution =
        evaluate_centralizer_torque_contribution_from_details(
            segment.centralizer_torque_details,
            reduced_torsional_load_n_m,
            reduced_twist_rate_rad_per_m);
    const bool body_contact_active = segment.body_normal_reaction_estimate_n > 0.0;
    const Scalar body_friction_force_n = body_contribution.axial_friction_n;
    const Scalar centralizer_axial_friction_n =
        std::max(0.0, centralizer_contribution.axial_friction_n);
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
        std::max(0.0, centralizer_contribution.tangential_friction_n),
    };
    result.centralizer_tangential_friction_profile.at(segment_index) = CentralizerFrictionPoint{
        segment.measured_depth_center_m,
        centralizer_axial_friction_n,
        std::max(0.0, centralizer_contribution.tangential_friction_n),
    };
    result.centralizer_tangential_direction_profile.at(segment_index) =
        CentralizerTangentialDirectionPoint{
            segment.measured_depth_center_m,
            centralizer_contribution.reduced_torsional_load_n_m,
            centralizer_contribution.reduced_twist_rate_rad_per_m,
            centralizer_contribution.torsional_slip_indicator,
            centralizer_contribution.torsional_tangential_demand_factor,
            centralizer_contribution.effective_radial_direction_n_b[0],
            centralizer_contribution.effective_radial_direction_n_b[1],
            centralizer_contribution.tangential_direction_n_b[0],
            centralizer_contribution.tangential_direction_n_b[1],
            centralizer_contribution.projected_contact_normal_n,
            centralizer_contribution.friction_interaction_scale,
            centralizer_contribution.status,
        };
    result.centralizer_tangential_friction_vector_profile.at(segment_index) =
        CentralizerTangentialVectorContribution{
            segment.measured_depth_center_m,
            centralizer_contribution.effective_radial_direction_n_b[0],
            centralizer_contribution.effective_radial_direction_n_b[1],
            centralizer_contribution.tangential_direction_n_b[0],
            centralizer_contribution.tangential_direction_n_b[1],
            centralizer_contribution.tangential_friction_vector_n_b[0],
            centralizer_contribution.tangential_friction_vector_n_b[1],
            centralizer_contribution.projected_contact_normal_n,
            centralizer_contribution.friction_interaction_scale,
            centralizer_contribution.tangential_friction_vector_magnitude_n,
            centralizer_contribution.status,
        };
    const Scalar local_tangential_mobilization = std::max(
        body_contribution.tangential_mobilization,
        centralizer_contribution.tangential_mobilization);
    const Scalar local_tangential_traction_indicator = std::max(
        body_contribution.tangential_traction_indicator,
        centralizer_contribution.tangential_traction_indicator);
    const std::string local_tangential_regime =
        !(body_contact_active || centralizer_contribution.active)
            ? "inactive"
            : local_tangential_traction_indicator < 0.25
                  ? "reduced-partial-adhesion-proxy"
                  : local_tangential_traction_indicator < 0.95
                        ? "reduced-mobilizing-traction"
                        : "reduced-slip-limited-traction";
    result.local_tangential_interaction_state.at(segment_index) =
        LocalTangentialInteractionStatePoint{
            segment.measured_depth_center_m,
            reduced_torsional_load_n_m,
            reduced_twist_rate_rad_per_m,
            body_contribution.contact_radius_m,
            body_contribution.torsional_slip_indicator,
            body_contribution.tangential_mobilization,
            body_contribution.tangential_demand_factor,
            body_contribution.tangential_traction_indicator,
            body_contribution.tangential_regime,
            body_contribution.feedback_applied,
            centralizer_contribution.effective_contact_radius_m,
            centralizer_contribution.torsional_slip_indicator,
            centralizer_contribution.tangential_mobilization,
            centralizer_contribution.torsional_tangential_demand_factor,
            centralizer_contribution.tangential_traction_indicator,
            centralizer_contribution.tangential_regime,
            centralizer_contribution.active && centralizer_contribution.torsional_slip_indicator > 0.0,
            local_tangential_mobilization,
            local_tangential_traction_indicator,
            local_tangential_regime,
            body_contribution.status,
            centralizer_contribution.active
                ? centralizer_contribution.status
                : "inactive",
            (body_contact_active || centralizer_contribution.active)
                ? "phase14-reduced-local-body-centralizer-tangential-interaction-state"
                : "inactive",
        };

    const Scalar body_torque_increment_n_m =
        body_contribution.torque_increment_n_m;
    const Scalar centralizer_torque_increment_n_m =
        std::max(0.0, centralizer_contribution.torque_increment_n_m);
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
        centralizer_contribution.effective_contact_radius_m,
        0.0,
        centralizer_torque_increment_n_m,
        centralizer_torque_increment_n_m,
        cumulative_centralizer_surface_torque_n_m + (0.5 * centralizer_torque_increment_n_m),
    };
    result.torque_profile.at(segment_index) = TorquePoint{
        segment.measured_depth_center_m,
        std::max(body_contribution.contact_radius_m, centralizer_contribution.effective_contact_radius_m),
        body_torque_increment_n_m,
        centralizer_torque_increment_n_m,
        local_torque_increment_n_m,
        cumulative_surface_torque_n_m + (0.5 * local_torque_increment_n_m),
    };
    auto& breakdown_points =
        result.centralizer_torque_breakdown_profile.at(segment_index);
    breakdown_points.reserve(centralizer_contribution.placement_contributions.size());
    for (const auto& detail : centralizer_contribution.placement_contributions) {
      const CentralizerPlacementKey placement_key{
          detail.source_name,
          detail.placement_measured_depth_m,
      };
      const Scalar cumulative_before_n_m =
          cumulative_placement_torque_n_m[placement_key];
      breakdown_points.push_back(CentralizerPlacementTorquePoint{
          detail.source_name,
          detail.placement_measured_depth_m,
          segment.measured_depth_center_m,
          detail.effective_contact_radius_m,
          detail.axial_force_ratio,
          detail.tangential_force_ratio,
          detail.reduced_torsional_load_n_m,
          detail.reduced_twist_rate_rad_per_m,
          detail.torsional_slip_indicator,
          detail.torsional_tangential_demand_factor,
          detail.bow_resultant_vector_n_b[0],
          detail.bow_resultant_vector_n_b[1],
          detail.bow_resultant_magnitude_n,
          detail.local_contact_direction_n_b[0],
          detail.local_contact_direction_n_b[1],
          detail.effective_radial_direction_n_b[0],
          detail.effective_radial_direction_n_b[1],
          detail.tangential_direction_n_b[0],
          detail.tangential_direction_n_b[1],
          detail.tangential_friction_vector_n_b[0],
          detail.tangential_friction_vector_n_b[1],
          detail.tangential_friction_vector_magnitude_n,
          detail.local_contact_weight,
          detail.direction_alignment_cosine,
          detail.projected_contact_normal_n,
          detail.friction_interaction_scale,
          detail.axial_friction_n,
          detail.tangential_friction_n,
          detail.torque_increment_n_m,
          cumulative_before_n_m + (0.5 * detail.torque_increment_n_m),
          detail.status,
      });
      cumulative_placement_torque_n_m[placement_key] =
          cumulative_before_n_m + detail.torque_increment_n_m;
    }
    cumulative_surface_torque_n_m += local_torque_increment_n_m;
    cumulative_body_surface_torque_n_m += body_torque_increment_n_m;
    cumulative_centralizer_surface_torque_n_m += centralizer_torque_increment_n_m;
    result.torque_partition_summary.body_axial_friction_sum_n += body_contribution.axial_friction_n;
    result.torque_partition_summary.centralizer_axial_friction_sum_n += centralizer_axial_friction_n;
    result.torque_partition_summary.body_tangential_friction_sum_n +=
        body_contribution.tangential_friction_n;
    result.torque_partition_summary.centralizer_tangential_friction_sum_n +=
        std::max(0.0, centralizer_contribution.tangential_friction_n);
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
