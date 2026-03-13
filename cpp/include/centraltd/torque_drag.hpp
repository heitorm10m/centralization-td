#pragma once

#include "centraltd/mechanical_solver.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/torque_drag_centralizer.hpp"

#include <optional>
#include <string>
#include <vector>

namespace centraltd {

struct AxialForcePoint {
  Scalar measured_depth_m{0.0};
  Scalar axial_force_n{0.0};
};

struct TorquePoint {
  Scalar measured_depth_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar body_torque_increment_n_m{0.0};
  Scalar centralizer_torque_increment_n_m{0.0};
  Scalar local_torque_increment_n_m{0.0};
  Scalar cumulative_torque_n_m{0.0};
};

struct CentralizerFrictionPoint {
  Scalar measured_depth_m{0.0};
  Scalar centralizer_axial_friction_n{0.0};
  Scalar centralizer_tangential_friction_n{0.0};
};

struct BodyFrictionPoint {
  Scalar measured_depth_m{0.0};
  Scalar body_axial_friction_n{0.0};
  Scalar body_tangential_friction_n{0.0};
};

struct CentralizerTangentialDirectionPoint {
  Scalar measured_depth_m{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar torsional_slip_indicator{0.0};
  Scalar torsional_tangential_demand_factor{1.0};
  Scalar effective_radial_direction_normal{0.0};
  Scalar effective_radial_direction_binormal{0.0};
  Scalar tangential_direction_normal{0.0};
  Scalar tangential_direction_binormal{0.0};
  Scalar projected_contact_normal_n{0.0};
  Scalar friction_interaction_scale{1.0};
  std::string status{"inactive"};
};

struct CentralizerPlacementTorquePoint {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};
  Scalar measured_depth_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar axial_force_ratio{0.0};
  Scalar tangential_force_ratio{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar torsional_slip_indicator{0.0};
  Scalar torsional_tangential_demand_factor{1.0};
  Scalar bow_resultant_normal_n{0.0};
  Scalar bow_resultant_binormal_n{0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Scalar local_contact_direction_normal{0.0};
  Scalar local_contact_direction_binormal{0.0};
  Scalar effective_radial_direction_normal{0.0};
  Scalar effective_radial_direction_binormal{0.0};
  Scalar tangential_direction_normal{0.0};
  Scalar tangential_direction_binormal{0.0};
  Scalar tangential_friction_normal_n{0.0};
  Scalar tangential_friction_binormal_n{0.0};
  Scalar tangential_friction_magnitude_n{0.0};
  Scalar local_contact_weight{0.0};
  Scalar direction_alignment_cosine{0.0};
  Scalar projected_contact_normal_n{0.0};
  Scalar friction_interaction_scale{1.0};
  Scalar axial_friction_n{0.0};
  Scalar tangential_friction_n{0.0};
  Scalar local_torque_increment_n_m{0.0};
  Scalar cumulative_torque_n_m{0.0};
  std::string status{"inactive"};
};

struct LocalTangentialInteractionStatePoint {
  Scalar measured_depth_m{0.0};
  Scalar reduced_torsional_load_n_m{0.0};
  Scalar reduced_twist_rate_rad_per_m{0.0};
  Scalar body_effective_contact_radius_m{0.0};
  Scalar body_torsional_slip_indicator{0.0};
  Scalar body_tangential_mobilization{0.0};
  Scalar body_tangential_demand_factor{1.0};
  Scalar body_tangential_traction_indicator{0.0};
  std::string body_tangential_regime{"inactive"};
  bool body_feedback_applied{false};
  Scalar centralizer_effective_contact_radius_m{0.0};
  Scalar centralizer_torsional_slip_indicator{0.0};
  Scalar centralizer_tangential_mobilization{0.0};
  Scalar centralizer_tangential_demand_factor{1.0};
  Scalar centralizer_tangential_traction_indicator{0.0};
  std::string centralizer_tangential_regime{"inactive"};
  bool centralizer_feedback_applied{false};
  Scalar local_tangential_mobilization{0.0};
  Scalar local_tangential_traction_indicator{0.0};
  std::string local_tangential_regime{"inactive"};
  std::string body_status{"inactive"};
  std::string centralizer_status{"inactive"};
  std::string status{"inactive"};
};

struct TorqueDragResult {
  std::string status{"phase14-reduced-unified-local-tangential-torque-baseline"};
  std::string operation_mode{"run_in"};
  std::vector<AxialForcePoint> axial_force_run_in_profile;
  std::vector<AxialForcePoint> axial_force_pull_out_profile;
  std::vector<TorquePoint> torque_profile;
  std::vector<BodyFrictionPoint> body_axial_friction_profile;
  std::vector<TorquePoint> body_torque_profile;
  std::vector<CentralizerFrictionPoint> centralizer_axial_friction_profile;
  std::vector<CentralizerFrictionPoint> centralizer_tangential_friction_profile;
  std::vector<CentralizerTangentialDirectionPoint> centralizer_tangential_direction_profile;
  std::vector<CentralizerTangentialVectorContribution> centralizer_tangential_friction_vector_profile;
  std::vector<TorquePoint> centralizer_torque_profile;
  std::vector<std::vector<CentralizerPlacementTorquePoint>> centralizer_torque_breakdown_profile;
  std::vector<LocalTangentialInteractionStatePoint> local_tangential_interaction_state;
  CentralizerTorquePartitionSummary torque_partition_summary;
  Scalar hookload_run_in_n{0.0};
  Scalar hookload_pull_out_n{0.0};
  Scalar drag_run_in_n{0.0};
  Scalar drag_pull_out_n{0.0};
  std::optional<Scalar> estimated_surface_torque_n_m;
};

bool is_supported_operation_mode(const std::string& operation_mode);

// `reference_buoyant_hookload_n` is the frictionless surface hookload [N]
// obtained from buoyant tangential weight accumulation only. The pipe-body
// friction term still uses `body_normal_reaction_estimate_n` [N], i.e. a
// resultant segment reaction rather than a distributed load [N/m], so
// `mu * N_body` stays in [N] and `mu * N_body * r_body` stays in [N.m].
// Detailed centralizer drag/torque use the bow-spring resultant together with
// the local contact direction in the `n-b` frame. The reduced centralizer
// law projects the bow resultant onto an effective local radial direction and
// then applies separate axial/tangential ratios under a reduced combined
// friction budget. When a carried reduced torsional state is available, both
// pipe-body and centralizer tangential laws now pass through a shared reduced
// local tangential-state model built from |twist_rate| * r, bounded
// mobilization, and a traction-indicator proxy. Pipe-body axial drag remains
// the reduced `mu * N_body` law and is still reported separately from the
// centralizer contribution.
TorqueDragResult run_torque_drag_baseline(
    Scalar reference_buoyant_hookload_n,
    const std::vector<MechanicalSegmentResult>& mechanical_profile,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode,
    const std::vector<Scalar>& reduced_torsional_load_profile_n_m = {},
    const std::vector<Scalar>& reduced_twist_rate_profile_rad_per_m = {});

}  // namespace centraltd
