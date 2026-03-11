#include "centraltd/vector_global_solver.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kSolverToleranceM = 1.0e-8;
constexpr Scalar kDirectionTolerance = 1.0e-6;
constexpr Scalar kMinimumPivot = 1.0e-12;

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

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
      throw ValidationError("Vector global solver encountered a singular linear system.");
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

VectorGlobalSolverResult run_vector_global_lateral_solver(
    const std::vector<VectorNodeInput>& nodes,
    const DiscretizationSettings& settings) {
  VectorGlobalSolverResult result;
  result.node_solutions.resize(nodes.size());
  result.contact_states.resize(nodes.size());

  std::vector<Vector2> displacements_n_b_m(nodes.size(), {0.0, 0.0});
  for (std::size_t iteration_index = 0; iteration_index < settings.global_solver_max_iterations;
       ++iteration_index) {
    const auto system = assemble_vector_global_linear_system(nodes, result.contact_states);
    const auto updated_solution = solve_dense_linear_system(
        system.stiffness_matrix,
        system.load_vector,
        system.dof_count);

    result.final_update_norm_m = 0.0;
    bool contact_state_changed = false;
    bool contact_direction_changed = false;
    for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
      const Vector2 updated_displacement_n_b_m{
          updated_solution[(2U * node_index)],
          updated_solution[(2U * node_index) + 1U],
      };
      const Vector2 displacement_update{
          updated_displacement_n_b_m[0] - displacements_n_b_m[node_index][0],
          updated_displacement_n_b_m[1] - displacements_n_b_m[node_index][1],
      };
      result.final_update_norm_m = std::max(
          result.final_update_norm_m,
          vector_norm(displacement_update));

      const auto contact_direction =
          select_contact_direction(updated_displacement_n_b_m, nodes.at(node_index).equivalent_lateral_force_n_b);
      const Scalar eccentricity_magnitude_m = vector_norm(updated_displacement_n_b_m);
      const bool support_contact_active =
          nodes.at(node_index).support_contact_penalty_n_per_m > 0.0 &&
          eccentricity_magnitude_m > nodes.at(node_index).support_contact_clearance_m;
      const bool pipe_body_contact_active =
          eccentricity_magnitude_m > nodes.at(node_index).pipe_body_clearance_m;
      const Vector2 direction_delta{
          contact_direction[0] - result.contact_states.at(node_index).contact_direction_n_b[0],
          contact_direction[1] - result.contact_states.at(node_index).contact_direction_n_b[1],
      };

      if (support_contact_active != result.contact_states.at(node_index).support_contact_active ||
          pipe_body_contact_active != result.contact_states.at(node_index).pipe_body_contact_active) {
        contact_state_changed = true;
      }
      if (vector_norm(direction_delta) > kDirectionTolerance) {
        contact_direction_changed = true;
      }

      result.contact_states.at(node_index).support_contact_active = support_contact_active;
      result.contact_states.at(node_index).pipe_body_contact_active = pipe_body_contact_active;
      result.contact_states.at(node_index).contact_direction_n_b = contact_direction;
      displacements_n_b_m[node_index] = updated_displacement_n_b_m;
    }

    result.iteration_count = iteration_index + 1U;
    if (!contact_state_changed &&
        !contact_direction_changed &&
        result.final_update_norm_m <= kSolverToleranceM) {
      break;
    }
  }

  for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
    const auto& node = nodes.at(node_index);
    const auto displacement_n_b_m = displacements_n_b_m.at(node_index);
    const auto contact_response = evaluate_vector_contact(node, displacement_n_b_m);

    VectorNodeSolution solution;
    solution.measured_depth_m = node.measured_depth_m;
    solution.lateral_displacement_normal_m = displacement_n_b_m[0];
    solution.lateral_displacement_binormal_m = displacement_n_b_m[1];
    solution.eccentricity_estimate_m = contact_response.eccentricity_magnitude_m;
    solution.eccentricity_ratio = contact_response.eccentricity_ratio;
    solution.standoff_estimate = contact_response.standoff_estimate;
    solution.contact_direction_n_b = contact_response.contact_direction_n_b;
    solution.support_normal_reaction_vector_n_b =
        contact_response.support_normal_reaction_vector_n_b;
    solution.body_normal_reaction_vector_n_b =
        contact_response.body_normal_reaction_vector_n_b;
    solution.normal_reaction_vector_n_b =
        contact_response.normal_reaction_vector_n_b;
    solution.support_normal_reaction_estimate_n =
        contact_response.support_normal_reaction_estimate_n;
    solution.body_normal_reaction_estimate_n =
        contact_response.body_normal_reaction_estimate_n;
    solution.normal_reaction_estimate_n = contact_response.normal_reaction_estimate_n;
    solution.normal_reaction_estimate_n_per_m =
        contact_response.normal_reaction_estimate_n_per_m;
    solution.support_in_contact = contact_response.support_in_contact;
    solution.pipe_body_in_contact = contact_response.pipe_body_in_contact;
    solution.contact_state = contact_response.contact_state;
    result.node_solutions.at(node_index) = solution;
  }

  return result;
}

}  // namespace centraltd
