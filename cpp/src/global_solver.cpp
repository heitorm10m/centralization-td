#include "centraltd/global_solver.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kSolverToleranceM = 1.0e-8;
constexpr Scalar kMinimumPivot = 1.0e-12;

std::vector<Scalar> solve_dense_linear_system(
    std::vector<Scalar> matrix,
    std::vector<Scalar> rhs,
    std::size_t dimension) {
  for (std::size_t pivot_index = 0; pivot_index < dimension; ++pivot_index) {
    std::size_t best_row_index = pivot_index;
    Scalar best_pivot_value = std::abs(matrix[(pivot_index * dimension) + pivot_index]);
    for (std::size_t row_index = pivot_index + 1U; row_index < dimension; ++row_index) {
      const Scalar candidate_value =
          std::abs(matrix[(row_index * dimension) + pivot_index]);
      if (candidate_value > best_pivot_value) {
        best_pivot_value = candidate_value;
        best_row_index = row_index;
      }
    }

    if (best_pivot_value <= kMinimumPivot) {
      throw ValidationError("Global solver encountered a singular linear system.");
    }

    if (best_row_index != pivot_index) {
      for (std::size_t column_index = 0; column_index < dimension; ++column_index) {
        std::swap(
            matrix[(pivot_index * dimension) + column_index],
            matrix[(best_row_index * dimension) + column_index]);
      }
      std::swap(rhs[pivot_index], rhs[best_row_index]);
    }

    const Scalar pivot_value = matrix[(pivot_index * dimension) + pivot_index];
    for (std::size_t row_index = pivot_index + 1U; row_index < dimension; ++row_index) {
      const Scalar elimination_factor =
          matrix[(row_index * dimension) + pivot_index] / pivot_value;
      if (elimination_factor == 0.0) {
        continue;
      }

      for (std::size_t column_index = pivot_index; column_index < dimension; ++column_index) {
        matrix[(row_index * dimension) + column_index] -=
            elimination_factor * matrix[(pivot_index * dimension) + column_index];
      }
      rhs[row_index] -= elimination_factor * rhs[pivot_index];
    }
  }

  std::vector<Scalar> solution(dimension, 0.0);
  for (std::size_t reverse_index = dimension; reverse_index > 0U; --reverse_index) {
    const std::size_t row_index = reverse_index - 1U;
    Scalar back_substitution_sum = rhs[row_index];
    for (std::size_t column_index = row_index + 1U; column_index < dimension; ++column_index) {
      back_substitution_sum -=
          matrix[(row_index * dimension) + column_index] * solution[column_index];
    }
    solution[row_index] =
        back_substitution_sum / matrix[(row_index * dimension) + row_index];
  }

  return solution;
}

}  // namespace

GlobalSolverResult run_global_lateral_solver(
    const std::vector<GlobalNodeInput>& nodes,
    const DiscretizationSettings& settings) {
  GlobalSolverResult result;
  result.node_solutions.resize(nodes.size());
  result.contact_states.resize(nodes.size());

  std::vector<Scalar> displacements_m(nodes.size(), 0.0);
  for (std::size_t iteration_index = 0; iteration_index < settings.global_solver_max_iterations;
       ++iteration_index) {
    const auto system = assemble_global_linear_system(nodes, result.contact_states);
    const auto updated_displacements_m = solve_dense_linear_system(
        system.stiffness_matrix,
        system.load_vector,
        system.node_count);

    result.final_update_norm_m = 0.0;
    bool contact_state_changed = false;
    for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
      const Scalar updated_eccentricity_m = std::max(0.0, updated_displacements_m[node_index]);
      result.final_update_norm_m = std::max(
          result.final_update_norm_m,
          std::abs(updated_eccentricity_m - displacements_m[node_index]));

      const bool support_contact_active =
          nodes.at(node_index).support_contact_penalty_n_per_m > 0.0 &&
          updated_eccentricity_m > nodes.at(node_index).support_contact_clearance_m;
      const bool pipe_body_contact_active =
          updated_eccentricity_m > nodes.at(node_index).pipe_body_clearance_m;

      if (support_contact_active != result.contact_states.at(node_index).support_contact_active ||
          pipe_body_contact_active != result.contact_states.at(node_index).pipe_body_contact_active) {
        contact_state_changed = true;
      }

      result.contact_states.at(node_index).support_contact_active = support_contact_active;
      result.contact_states.at(node_index).pipe_body_contact_active = pipe_body_contact_active;
    }

    displacements_m = updated_displacements_m;
    for (auto& displacement_m : displacements_m) {
      displacement_m = std::max(0.0, displacement_m);
    }
    result.iteration_count = iteration_index + 1U;

    // Reduced global contact iteration: update the active penalty set from the
    // current nodal eccentricity estimate and reassemble until the active set
    // no longer changes and the displacement update is small.
    if (!contact_state_changed && result.final_update_norm_m <= kSolverToleranceM) {
      break;
    }
  }

  for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
    const auto& node = nodes.at(node_index);
    const Scalar eccentricity_estimate_m = std::max(0.0, displacements_m.at(node_index));
    const Scalar support_normal_reaction_estimate_n =
        std::max(
            0.0,
            node.support_contact_penalty_n_per_m *
                (eccentricity_estimate_m - node.support_contact_clearance_m));
    const Scalar body_normal_reaction_estimate_n =
        std::max(
            0.0,
            node.body_contact_penalty_n_per_m *
                (eccentricity_estimate_m - node.pipe_body_clearance_m));
    const Scalar total_normal_reaction_estimate_n =
        support_normal_reaction_estimate_n + body_normal_reaction_estimate_n;

    GlobalNodeSolution solution;
    solution.measured_depth_m = node.measured_depth_m;
    solution.eccentricity_estimate_m = eccentricity_estimate_m;
    solution.support_normal_reaction_estimate_n = support_normal_reaction_estimate_n;
    solution.body_normal_reaction_estimate_n = body_normal_reaction_estimate_n;
    solution.normal_reaction_estimate_n = total_normal_reaction_estimate_n;
    solution.normal_reaction_estimate_n_per_m =
        node.segment_length_m > 0.0 ? total_normal_reaction_estimate_n / node.segment_length_m : 0.0;
    solution.support_in_contact = support_normal_reaction_estimate_n > 0.0;
    solution.pipe_body_in_contact = body_normal_reaction_estimate_n > 0.0;

    if (node.pipe_body_clearance_m > 0.0) {
      solution.eccentricity_ratio = eccentricity_estimate_m / node.pipe_body_clearance_m;
      solution.standoff_estimate = std::clamp(
          1.0 - solution.eccentricity_ratio,
          0.0,
          1.0);
    } else {
      solution.eccentricity_ratio = 1.0;
      solution.standoff_estimate = 0.0;
    }

    if (solution.pipe_body_in_contact) {
      solution.contact_state = "pipe-body-contact";
    } else if (solution.support_in_contact) {
      solution.contact_state = "support-contact";
    } else {
      solution.contact_state = "free";
    }

    result.node_solutions.at(node_index) = solution;
  }

  return result;
}

}  // namespace centraltd
