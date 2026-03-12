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

struct TorqueDragResult {
  std::string status{"phase11-reduced-vector-centralizer-torque-baseline"};
  std::string operation_mode{"run_in"};
  std::vector<AxialForcePoint> axial_force_run_in_profile;
  std::vector<AxialForcePoint> axial_force_pull_out_profile;
  std::vector<TorquePoint> torque_profile;
  std::vector<BodyFrictionPoint> body_axial_friction_profile;
  std::vector<TorquePoint> body_torque_profile;
  std::vector<CentralizerFrictionPoint> centralizer_axial_friction_profile;
  std::vector<CentralizerFrictionPoint> centralizer_tangential_friction_profile;
  std::vector<CentralizerTangentialVectorContribution> centralizer_tangential_friction_vector_profile;
  std::vector<TorquePoint> centralizer_torque_profile;
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
// Detailed centralizer drag/torque use the bow-spring resultant magnitude
// scaled by the nominal running/restoring-force ratio, which keeps the
// reduced formulation dimensionally consistent while remaining explicit
// about its current limitations.
TorqueDragResult run_torque_drag_baseline(
    Scalar reference_buoyant_hookload_n,
    const std::vector<MechanicalSegmentResult>& mechanical_profile,
    const std::vector<StringSection>& string_sections,
    const std::string& operation_mode);

}  // namespace centraltd
