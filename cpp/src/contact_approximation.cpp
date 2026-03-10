#include "centraltd/contact_approximation.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumPenaltyStiffnessNPerM = 1.0;
constexpr Scalar kBodyPenaltyMultiplier = 50.0;
constexpr Scalar kConvergenceToleranceM = 1.0e-9;
constexpr std::size_t kMaximumIterations = 16U;

Scalar local_contact_residual_n(
    Scalar eccentricity_estimate_m,
    const LateralEquilibriumState& lateral_state,
    const CentralizerSupportEffect& support_effect,
    Scalar base_lateral_stiffness_n_per_m,
    Scalar body_contact_penalty_n_per_m) {
  const Scalar support_contact_reaction_n =
      (support_effect.support_present &&
       eccentricity_estimate_m > support_effect.support_contact_clearance_m)
          ? support_effect.support_contact_penalty_n_per_m *
                (eccentricity_estimate_m - support_effect.support_contact_clearance_m)
          : 0.0;
  const Scalar body_contact_reaction_n =
      eccentricity_estimate_m > support_effect.pipe_body_clearance_m
          ? body_contact_penalty_n_per_m *
                (eccentricity_estimate_m - support_effect.pipe_body_clearance_m)
          : 0.0;

  // Residual of the reduced local equilibrium:
  // R(e) = (k_struct + k_center) e + R_support(e) + R_body(e) - F_lateral = 0.
  return (base_lateral_stiffness_n_per_m * eccentricity_estimate_m) +
         support_contact_reaction_n +
         body_contact_reaction_n -
         lateral_state.equivalent_lateral_force_n;
}

}  // namespace

ContactApproximation evaluate_contact_approximation(
    const MechanicalSegmentInput& segment,
    const LateralEquilibriumState& lateral_state,
    const CentralizerSupportEffect& support_effect) {
  ContactApproximation approximation;
  approximation.support_outer_diameter_m = support_effect.support_outer_diameter_m;
  approximation.pipe_body_clearance_m = support_effect.pipe_body_clearance_m;
  approximation.support_contact_clearance_m = support_effect.support_contact_clearance_m;
  approximation.centralizer_centering_stiffness_n_per_m =
      support_effect.centering_stiffness_n_per_m;
  approximation.support_contact_penalty_n_per_m =
      support_effect.support_contact_penalty_n_per_m;
  approximation.free_eccentricity_estimate_m = lateral_state.free_eccentricity_estimate_m;
  approximation.nearby_centralizer_count = support_effect.nearby_centralizer_count;

  const Scalar base_lateral_stiffness_n_per_m =
      lateral_state.structural_lateral_stiffness_n_per_m +
      support_effect.centering_stiffness_n_per_m;
  approximation.body_contact_penalty_n_per_m = kBodyPenaltyMultiplier * std::max(
      base_lateral_stiffness_n_per_m,
      kMinimumPenaltyStiffnessNPerM);

  if (segment.reference_hole_diameter_m <= 0.0) {
    approximation.eccentricity_estimate_m = lateral_state.free_eccentricity_estimate_m;
    approximation.contact_state = "hole-undefined";
    return approximation;
  }

  Scalar eccentricity_estimate_m = lateral_state.free_eccentricity_estimate_m;
  for (std::size_t iteration_index = 0; iteration_index < kMaximumIterations; ++iteration_index) {
    Scalar effective_stiffness_n_per_m = base_lateral_stiffness_n_per_m;
    Scalar constrained_force_n = lateral_state.equivalent_lateral_force_n;

    if (support_effect.support_present &&
        eccentricity_estimate_m > support_effect.support_contact_clearance_m) {
      effective_stiffness_n_per_m += support_effect.support_contact_penalty_n_per_m;
      constrained_force_n +=
          support_effect.support_contact_penalty_n_per_m *
          support_effect.support_contact_clearance_m;
    }

    if (eccentricity_estimate_m > support_effect.pipe_body_clearance_m) {
      effective_stiffness_n_per_m += approximation.body_contact_penalty_n_per_m;
      constrained_force_n +=
          approximation.body_contact_penalty_n_per_m *
          support_effect.pipe_body_clearance_m;
    }

    const Scalar updated_eccentricity_m =
        effective_stiffness_n_per_m > 0.0
            ? constrained_force_n / effective_stiffness_n_per_m
            : 0.0;
    approximation.final_update_norm_m =
        std::abs(updated_eccentricity_m - eccentricity_estimate_m);
    approximation.iteration_count = iteration_index + 1U;
    eccentricity_estimate_m = updated_eccentricity_m;

    if (approximation.final_update_norm_m <= kConvergenceToleranceM) {
      break;
    }
  }

  approximation.eccentricity_estimate_m = eccentricity_estimate_m;
  approximation.support_contact_penetration_m =
      support_effect.support_present
          ? std::max(
                0.0,
                eccentricity_estimate_m - support_effect.support_contact_clearance_m)
          : 0.0;
  approximation.body_contact_penetration_m =
      std::max(0.0, eccentricity_estimate_m - support_effect.pipe_body_clearance_m);
  approximation.support_normal_reaction_estimate_n =
      support_effect.support_contact_penalty_n_per_m *
      approximation.support_contact_penetration_m;
  approximation.body_normal_reaction_estimate_n =
      approximation.body_contact_penalty_n_per_m *
      approximation.body_contact_penetration_m;
  approximation.normal_reaction_estimate_n =
      approximation.support_normal_reaction_estimate_n +
      approximation.body_normal_reaction_estimate_n;
  approximation.normal_reaction_estimate_n_per_m =
      segment.segment_length_m > 0.0
          ? approximation.normal_reaction_estimate_n / segment.segment_length_m
          : 0.0;
  approximation.final_residual_n = std::abs(local_contact_residual_n(
      eccentricity_estimate_m,
      lateral_state,
      support_effect,
      base_lateral_stiffness_n_per_m,
      approximation.body_contact_penalty_n_per_m));

  if (support_effect.pipe_body_clearance_m > 0.0) {
    approximation.eccentricity_ratio =
        eccentricity_estimate_m / support_effect.pipe_body_clearance_m;
    approximation.standoff_estimate = std::clamp(
        1.0 - approximation.eccentricity_ratio,
        0.0,
        1.0);
  } else {
    approximation.eccentricity_ratio = 1.0;
    approximation.standoff_estimate = 0.0;
  }

  approximation.support_in_contact =
      approximation.support_normal_reaction_estimate_n > 0.0;
  approximation.pipe_body_in_contact =
      approximation.body_normal_reaction_estimate_n > 0.0;

  if (approximation.pipe_body_in_contact) {
    approximation.contact_state = "pipe-body-contact";
  } else if (approximation.support_in_contact) {
    approximation.contact_state = "support-contact";
  } else {
    approximation.contact_state = "free";
  }

  return approximation;
}

}  // namespace centraltd
