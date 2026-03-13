#include "centraltd/torque_drag_centralizer.hpp"

#include "centraltd/local_tangential_model.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace centraltd {
namespace {

constexpr char kPhase11PlacementStatus[] =
    "phase11-reduced-contact-informed-placement-vector-torque";
constexpr char kPhase11CentralizerStatus[] =
    "phase11-reduced-contact-informed-vector-centralizer-torque";
constexpr char kPhase14PlacementStatus[] =
    "phase14-reduced-local-tangential-placement-vector-torque";
constexpr char kPhase14CentralizerStatus[] =
    "phase14-reduced-local-tangential-vector-centralizer-torque";

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

Scalar dot_product(const Vector2& lhs, const Vector2& rhs) {
  return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]);
}

Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return std::max(lower, std::min(value, upper));
}

Vector2 normalize_or_zero(const Vector2& vector) {
  const Scalar magnitude = vector_norm(vector);
  if (magnitude <= 1.0e-12) {
    return {0.0, 0.0};
  }
  return {vector[0] / magnitude, vector[1] / magnitude};
}

Vector2 normalize_or_fallback(const Vector2& vector, const Vector2& fallback) {
  const Vector2 normalized = normalize_or_zero(vector);
  if (vector_norm(normalized) > 1.0e-12) {
    return normalized;
  }
  return normalize_or_zero(fallback);
}

Vector2 rotate_ccw_90(const Vector2& vector) {
  return {-vector[1], vector[0]};
}

struct PlacementTorqueInputs {
  std::string source_name;
  Scalar placement_measured_depth_m{0.0};
  Scalar effective_contact_radius_m{0.0};
  Scalar axial_force_ratio{0.0};
  Scalar tangential_force_ratio{0.0};
  Vector2 bow_resultant_vector_n_b{0.0, 0.0};
  Scalar bow_resultant_magnitude_n{0.0};
  Vector2 radial_direction_n_b{0.0, 0.0};
  Vector2 local_contact_direction_n_b{0.0, 0.0};
  Vector2 effective_radial_direction_n_b{0.0, 0.0};
  Vector2 tangential_direction_n_b{0.0, 0.0};
  Scalar local_contact_weight{0.0};
  Scalar direction_alignment_cosine{0.0};
  Scalar projected_contact_normal_n{0.0};
};

CentralizerPlacementTorqueContribution evaluate_placement_contribution(
    const PlacementTorqueInputs& inputs,
    Scalar reduced_torsional_load_n_m,
    Scalar reduced_twist_rate_rad_per_m,
    const char* active_status) {
  CentralizerPlacementTorqueContribution contribution;
  contribution.source_name = inputs.source_name;
  contribution.placement_measured_depth_m = inputs.placement_measured_depth_m;
  contribution.effective_contact_radius_m = inputs.effective_contact_radius_m;
  contribution.axial_force_ratio = inputs.axial_force_ratio;
  contribution.tangential_force_ratio = inputs.tangential_force_ratio;
  contribution.reduced_torsional_load_n_m = std::max(0.0, reduced_torsional_load_n_m);
  contribution.reduced_twist_rate_rad_per_m = std::abs(reduced_twist_rate_rad_per_m);
  contribution.bow_resultant_vector_n_b = inputs.bow_resultant_vector_n_b;
  contribution.bow_resultant_magnitude_n = inputs.bow_resultant_magnitude_n;
  contribution.radial_direction_n_b = inputs.radial_direction_n_b;
  contribution.local_contact_direction_n_b = inputs.local_contact_direction_n_b;
  contribution.effective_radial_direction_n_b = inputs.effective_radial_direction_n_b;
  contribution.tangential_direction_n_b = inputs.tangential_direction_n_b;
  contribution.local_contact_weight = inputs.local_contact_weight;
  contribution.direction_alignment_cosine = inputs.direction_alignment_cosine;
  contribution.projected_contact_normal_n = inputs.projected_contact_normal_n;
  const auto tangential_state = evaluate_local_tangential_state(
      LocalTangentialModelInputs{
          contribution.reduced_torsional_load_n_m,
          contribution.reduced_twist_rate_rad_per_m,
          contribution.effective_contact_radius_m,
          inputs.projected_contact_normal_n,
          inputs.tangential_force_ratio * inputs.projected_contact_normal_n,
      });
  contribution.torsional_slip_indicator =
      tangential_state.tangential_slip_indicator;
  contribution.tangential_mobilization =
      tangential_state.tangential_mobilization;
  contribution.torsional_tangential_demand_factor =
      tangential_state.tangential_demand_factor;
  contribution.tangential_traction_indicator =
      tangential_state.tangential_traction_indicator;
  contribution.tangential_regime = tangential_state.tangential_regime;

  if (inputs.bow_resultant_magnitude_n <= 0.0 ||
      inputs.projected_contact_normal_n <= 1.0e-12) {
    contribution.status = "inactive";
    return contribution;
  }

  const Scalar axial_friction_demand_n =
      inputs.axial_force_ratio * inputs.projected_contact_normal_n;
  const Scalar tangential_friction_demand_n =
      tangential_state.mobilized_tangential_demand_n;
  const Scalar combined_friction_demand_n =
      std::hypot(axial_friction_demand_n, tangential_friction_demand_n);
  contribution.friction_interaction_scale =
      (inputs.projected_contact_normal_n <= 1.0e-12 ||
       combined_friction_demand_n <= inputs.projected_contact_normal_n)
          ? 1.0
          : inputs.projected_contact_normal_n / combined_friction_demand_n;
  contribution.axial_friction_n =
      contribution.friction_interaction_scale * axial_friction_demand_n;
  contribution.tangential_friction_n =
      contribution.friction_interaction_scale * tangential_friction_demand_n;
  contribution.tangential_friction_vector_n_b = {
      contribution.tangential_friction_n * inputs.tangential_direction_n_b[0],
      contribution.tangential_friction_n * inputs.tangential_direction_n_b[1],
  };
  contribution.tangential_friction_vector_magnitude_n =
      vector_norm(contribution.tangential_friction_vector_n_b);
  contribution.torque_increment_n_m =
      contribution.tangential_friction_n * contribution.effective_contact_radius_m;
  contribution.active = true;
  contribution.status = active_status;
  return contribution;
}

CentralizerTorqueContribution aggregate_contributions(
    std::vector<CentralizerPlacementTorqueContribution> placement_contributions,
    const Vector2& fallback_bow_resultant_vector_n_b,
    const Vector2& fallback_local_contact_direction_n_b,
    const char* active_status) {
  CentralizerTorqueContribution result;
  result.placement_contributions = std::move(placement_contributions);
  result.local_contact_direction_n_b =
      normalize_or_zero(fallback_local_contact_direction_n_b);

  Vector2 bow_resultant_sum_n_b{0.0, 0.0};
  Vector2 effective_radial_sum_n_b{0.0, 0.0};
  Scalar projected_contact_normal_sum_n = 0.0;
  Scalar friction_interaction_weighted_sum_n = 0.0;
  Scalar torsional_slip_weighted_sum = 0.0;
  Scalar torsional_demand_factor_weighted_sum = 0.0;

  for (const auto& contribution : result.placement_contributions) {
    bow_resultant_sum_n_b[0] += contribution.bow_resultant_vector_n_b[0];
    bow_resultant_sum_n_b[1] += contribution.bow_resultant_vector_n_b[1];
    result.reduced_torsional_load_n_m = std::max(
        result.reduced_torsional_load_n_m,
        contribution.reduced_torsional_load_n_m);
    result.reduced_twist_rate_rad_per_m = std::max(
        result.reduced_twist_rate_rad_per_m,
        contribution.reduced_twist_rate_rad_per_m);
    if (vector_norm(result.local_contact_direction_n_b) <= 1.0e-12 &&
        vector_norm(contribution.local_contact_direction_n_b) > 1.0e-12) {
      result.local_contact_direction_n_b =
          normalize_or_zero(contribution.local_contact_direction_n_b);
    }

    if (!contribution.active) {
      continue;
    }

    result.active = true;
    result.axial_friction_n += contribution.axial_friction_n;
    result.tangential_friction_n += contribution.tangential_friction_n;
    result.tangential_friction_vector_n_b[0] +=
        contribution.tangential_friction_vector_n_b[0];
    result.tangential_friction_vector_n_b[1] +=
        contribution.tangential_friction_vector_n_b[1];
    result.torque_increment_n_m += contribution.torque_increment_n_m;
    result.effective_contact_radius_m = std::max(
        result.effective_contact_radius_m,
        contribution.effective_contact_radius_m);
    projected_contact_normal_sum_n += contribution.projected_contact_normal_n;
    friction_interaction_weighted_sum_n +=
        contribution.projected_contact_normal_n * contribution.friction_interaction_scale;
    torsional_slip_weighted_sum +=
        contribution.projected_contact_normal_n * contribution.torsional_slip_indicator;
    result.tangential_mobilization +=
        contribution.projected_contact_normal_n *
        contribution.tangential_mobilization;
    torsional_demand_factor_weighted_sum +=
        contribution.projected_contact_normal_n *
        contribution.torsional_tangential_demand_factor;
    result.tangential_traction_indicator +=
        contribution.projected_contact_normal_n *
        contribution.tangential_traction_indicator;
    effective_radial_sum_n_b[0] +=
        contribution.projected_contact_normal_n *
        contribution.effective_radial_direction_n_b[0];
    effective_radial_sum_n_b[1] +=
        contribution.projected_contact_normal_n *
        contribution.effective_radial_direction_n_b[1];
  }

  result.bow_resultant_vector_n_b = bow_resultant_sum_n_b;
  if (vector_norm(result.bow_resultant_vector_n_b) <= 1.0e-12) {
    result.bow_resultant_vector_n_b = fallback_bow_resultant_vector_n_b;
  }
  result.bow_resultant_magnitude_n = vector_norm(result.bow_resultant_vector_n_b);
  result.projected_contact_normal_n = projected_contact_normal_sum_n;
  result.friction_interaction_scale =
      projected_contact_normal_sum_n > 1.0e-12
          ? friction_interaction_weighted_sum_n / projected_contact_normal_sum_n
          : 1.0;
  result.torsional_slip_indicator =
      projected_contact_normal_sum_n > 1.0e-12
          ? torsional_slip_weighted_sum / projected_contact_normal_sum_n
          : 0.0;
  result.torsional_tangential_demand_factor =
      projected_contact_normal_sum_n > 1.0e-12
          ? torsional_demand_factor_weighted_sum / projected_contact_normal_sum_n
          : 1.0;
  result.tangential_mobilization =
      projected_contact_normal_sum_n > 1.0e-12
          ? result.tangential_mobilization / projected_contact_normal_sum_n
          : 0.0;
  result.tangential_traction_indicator =
      projected_contact_normal_sum_n > 1.0e-12
          ? result.tangential_traction_indicator / projected_contact_normal_sum_n
          : 0.0;
  if (!result.active) {
    result.tangential_regime = "inactive";
  } else if (result.tangential_traction_indicator < 0.25) {
    result.tangential_regime = "reduced-partial-adhesion-proxy";
  } else if (result.tangential_traction_indicator < 0.95) {
    result.tangential_regime = "reduced-mobilizing-traction";
  } else {
    result.tangential_regime = "reduced-slip-limited-traction";
  }
  result.effective_radial_direction_n_b = normalize_or_fallback(
      effective_radial_sum_n_b,
      result.bow_resultant_vector_n_b);
  result.tangential_friction_vector_magnitude_n =
      vector_norm(result.tangential_friction_vector_n_b);
  if (result.tangential_friction_vector_magnitude_n > 1.0e-12) {
    result.tangential_direction_n_b = {
        result.tangential_friction_vector_n_b[0] /
            result.tangential_friction_vector_magnitude_n,
        result.tangential_friction_vector_n_b[1] /
            result.tangential_friction_vector_magnitude_n,
    };
    result.status = active_status;
    return result;
  }

  if (vector_norm(result.effective_radial_direction_n_b) > 1.0e-12) {
    result.tangential_direction_n_b = rotate_ccw_90(result.effective_radial_direction_n_b);
    result.status = result.active ? active_status : "inactive";
    return result;
  }

  if (result.bow_resultant_magnitude_n > 1.0e-12) {
    result.effective_radial_direction_n_b =
        normalize_or_zero(result.bow_resultant_vector_n_b);
    result.tangential_direction_n_b = rotate_ccw_90(result.effective_radial_direction_n_b);
    result.status = result.active ? active_status : "inactive";
    return result;
  }

  result.status = "inactive";
  return result;
}

}  // namespace

CentralizerTorqueContribution evaluate_centralizer_torque_contribution(
    const BowSpringSegmentResult& bow_segment_result,
    const Vector2& local_contact_direction_n_b,
    const Vector2& body_normal_reaction_vector_n_b) {
  std::vector<CentralizerPlacementTorqueContribution> placement_contributions;
  placement_contributions.reserve(bow_segment_result.placement_resultant_details.size());
  const Vector2 normalized_contact_direction_n_b =
      normalize_or_zero(local_contact_direction_n_b);
  const Scalar body_normal_reaction_magnitude_n =
      vector_norm(body_normal_reaction_vector_n_b);

  for (const auto& placement_result : bow_segment_result.placement_resultant_details) {
    if (placement_result.bow_resultant_magnitude_n <= 0.0) {
      PlacementTorqueInputs inactive_inputs;
      inactive_inputs.source_name = placement_result.source_name;
      inactive_inputs.placement_measured_depth_m =
          placement_result.placement_measured_depth_m;
      inactive_inputs.effective_contact_radius_m =
          placement_result.effective_contact_radius_m;
      inactive_inputs.axial_force_ratio = placement_result.axial_force_ratio;
      inactive_inputs.tangential_force_ratio = placement_result.tangential_force_ratio;
      inactive_inputs.bow_resultant_vector_n_b = placement_result.bow_resultant_vector_n_b;
      inactive_inputs.bow_resultant_magnitude_n =
          placement_result.bow_resultant_magnitude_n;
      inactive_inputs.local_contact_direction_n_b = normalized_contact_direction_n_b;
      placement_contributions.push_back(evaluate_placement_contribution(
          inactive_inputs,
          0.0,
          0.0,
          kPhase11PlacementStatus));
      continue;
    }

    const Vector2 radial_direction_n_b = normalize_or_zero(
        placement_result.bow_resultant_vector_n_b);
    const Scalar local_contact_weight =
        body_normal_reaction_magnitude_n <= 1.0e-12
            ? 0.0
            : body_normal_reaction_magnitude_n /
                  (body_normal_reaction_magnitude_n + placement_result.bow_resultant_magnitude_n);
    const Vector2 effective_radial_direction_n_b =
        vector_norm(normalized_contact_direction_n_b) <= 1.0e-12
            ? radial_direction_n_b
            : normalize_or_fallback(
                  {
                      ((1.0 - local_contact_weight) * radial_direction_n_b[0]) +
                          (local_contact_weight * normalized_contact_direction_n_b[0]),
                      ((1.0 - local_contact_weight) * radial_direction_n_b[1]) +
                          (local_contact_weight * normalized_contact_direction_n_b[1]),
                  },
                  radial_direction_n_b);
    const Scalar direction_alignment_cosine =
        vector_norm(normalized_contact_direction_n_b) <= 1.0e-12
            ? 1.0
            : clamp_scalar(
                  dot_product(radial_direction_n_b, normalized_contact_direction_n_b),
                  -1.0,
                  1.0);
    const Vector2 tangential_direction_n_b = rotate_ccw_90(effective_radial_direction_n_b);
    PlacementTorqueInputs inputs;
    inputs.source_name = placement_result.source_name;
    inputs.placement_measured_depth_m = placement_result.placement_measured_depth_m;
    inputs.effective_contact_radius_m = placement_result.effective_contact_radius_m;
    inputs.axial_force_ratio = placement_result.axial_force_ratio;
    inputs.tangential_force_ratio = placement_result.tangential_force_ratio;
    inputs.bow_resultant_vector_n_b = placement_result.bow_resultant_vector_n_b;
    inputs.bow_resultant_magnitude_n = placement_result.bow_resultant_magnitude_n;
    inputs.radial_direction_n_b = radial_direction_n_b;
    inputs.local_contact_direction_n_b = normalized_contact_direction_n_b;
    inputs.effective_radial_direction_n_b = effective_radial_direction_n_b;
    inputs.tangential_direction_n_b = tangential_direction_n_b;
    inputs.local_contact_weight = local_contact_weight;
    inputs.direction_alignment_cosine = direction_alignment_cosine;
    inputs.projected_contact_normal_n = std::max(
        0.0,
        dot_product(
            placement_result.bow_resultant_vector_n_b,
            effective_radial_direction_n_b));
    placement_contributions.push_back(evaluate_placement_contribution(
        inputs,
        0.0,
        0.0,
        kPhase11PlacementStatus));
  }

  return aggregate_contributions(
      std::move(placement_contributions),
      bow_segment_result.bow_resultant_vector_n_b,
      normalized_contact_direction_n_b,
      kPhase11CentralizerStatus);
}

CentralizerTorqueContribution evaluate_centralizer_torque_contribution_from_details(
    const std::vector<CentralizerPlacementTorqueContribution>& placement_details,
    Scalar reduced_torsional_load_n_m,
    Scalar reduced_twist_rate_rad_per_m) {
  std::vector<CentralizerPlacementTorqueContribution> placement_contributions;
  placement_contributions.reserve(placement_details.size());
  Vector2 fallback_bow_resultant_vector_n_b{0.0, 0.0};
  Vector2 fallback_local_contact_direction_n_b{0.0, 0.0};

  for (const auto& placement_detail : placement_details) {
    fallback_bow_resultant_vector_n_b[0] += placement_detail.bow_resultant_vector_n_b[0];
    fallback_bow_resultant_vector_n_b[1] += placement_detail.bow_resultant_vector_n_b[1];
    if (vector_norm(fallback_local_contact_direction_n_b) <= 1.0e-12 &&
        vector_norm(placement_detail.local_contact_direction_n_b) > 1.0e-12) {
      fallback_local_contact_direction_n_b =
          normalize_or_zero(placement_detail.local_contact_direction_n_b);
    }

    PlacementTorqueInputs inputs;
    inputs.source_name = placement_detail.source_name;
    inputs.placement_measured_depth_m = placement_detail.placement_measured_depth_m;
    inputs.effective_contact_radius_m = placement_detail.effective_contact_radius_m;
    inputs.axial_force_ratio = placement_detail.axial_force_ratio;
    inputs.tangential_force_ratio = placement_detail.tangential_force_ratio;
    inputs.bow_resultant_vector_n_b = placement_detail.bow_resultant_vector_n_b;
    inputs.bow_resultant_magnitude_n = placement_detail.bow_resultant_magnitude_n;
    inputs.radial_direction_n_b = placement_detail.radial_direction_n_b;
    inputs.local_contact_direction_n_b = placement_detail.local_contact_direction_n_b;
    inputs.effective_radial_direction_n_b = placement_detail.effective_radial_direction_n_b;
    inputs.tangential_direction_n_b = placement_detail.tangential_direction_n_b;
    inputs.local_contact_weight = placement_detail.local_contact_weight;
    inputs.direction_alignment_cosine = placement_detail.direction_alignment_cosine;
    inputs.projected_contact_normal_n = placement_detail.projected_contact_normal_n;
    placement_contributions.push_back(evaluate_placement_contribution(
        inputs,
        reduced_torsional_load_n_m,
        reduced_twist_rate_rad_per_m,
        kPhase14PlacementStatus));
  }

  return aggregate_contributions(
      std::move(placement_contributions),
      fallback_bow_resultant_vector_n_b,
      fallback_local_contact_direction_n_b,
      kPhase14CentralizerStatus);
}

}  // namespace centraltd
