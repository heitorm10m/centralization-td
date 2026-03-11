#include "centraltd/vector_global_assembly.hpp"

#include <algorithm>

namespace centraltd {
namespace {

constexpr Scalar kMinimumSpacingM = 1.0e-6;

std::size_t dof_index(std::size_t node_index, std::size_t component_index) {
  return (2U * node_index) + component_index;
}

void add_to_matrix(
    VectorLinearSystem* system,
    std::size_t row,
    std::size_t column,
    Scalar value) {
  system->stiffness_matrix[(row * system->dof_count) + column] += value;
}

void apply_centered_end_boundary(VectorLinearSystem* system, std::size_t node_index) {
  for (std::size_t component_index = 0; component_index < 2U; ++component_index) {
    const std::size_t boundary_dof = dof_index(node_index, component_index);
    for (std::size_t row_index = 0; row_index < system->dof_count; ++row_index) {
      system->stiffness_matrix[(row_index * system->dof_count) + boundary_dof] = 0.0;
    }
    for (std::size_t column_index = 0; column_index < system->dof_count; ++column_index) {
      system->stiffness_matrix[(boundary_dof * system->dof_count) + column_index] = 0.0;
    }
    system->stiffness_matrix[(boundary_dof * system->dof_count) + boundary_dof] = 1.0;
    system->load_vector[boundary_dof] = 0.0;
  }
}

}  // namespace

VectorLinearSystem assemble_vector_global_linear_system(
    const std::vector<VectorNodeInput>& nodes,
    const std::vector<VectorContactState>& contact_states) {
  if (nodes.size() != contact_states.size()) {
    throw ValidationError("Vector global assembly requires one contact state per node.");
  }

  VectorLinearSystem system;
  system.node_count = nodes.size();
  system.dof_count = 2U * system.node_count;
  system.stiffness_matrix.assign(system.dof_count * system.dof_count, 0.0);
  system.load_vector.assign(system.dof_count, 0.0);

  for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
    const auto& node = nodes.at(node_index);
    const auto& contact_state = contact_states.at(node_index);

    for (std::size_t component_index = 0; component_index < 2U; ++component_index) {
      const std::size_t component_dof = dof_index(node_index, component_index);
      system.load_vector[component_dof] += node.equivalent_lateral_force_n_b.at(component_index);
      add_to_matrix(
          &system,
          component_dof,
          component_dof,
          node.centralizer_centering_stiffness_n_per_m);
    }

    const auto add_contact_penalty = [&](Scalar penalty_n_per_m, Scalar clearance_m) {
      for (std::size_t row_component = 0; row_component < 2U; ++row_component) {
        for (std::size_t column_component = 0; column_component < 2U; ++column_component) {
          add_to_matrix(
              &system,
              dof_index(node_index, row_component),
              dof_index(node_index, column_component),
              penalty_n_per_m *
                  contact_state.contact_direction_n_b.at(row_component) *
                  contact_state.contact_direction_n_b.at(column_component));
        }
        system.load_vector[dof_index(node_index, row_component)] +=
            penalty_n_per_m * clearance_m * contact_state.contact_direction_n_b.at(row_component);
      }
    };

    if (contact_state.support_contact_active) {
      add_contact_penalty(node.support_contact_penalty_n_per_m, node.support_contact_clearance_m);
    }
    if (contact_state.pipe_body_contact_active) {
      add_contact_penalty(node.body_contact_penalty_n_per_m, node.pipe_body_clearance_m);
    }
  }

  for (std::size_t node_index = 1; node_index < nodes.size(); ++node_index) {
    const Scalar spacing_m = std::max(
        nodes.at(node_index).measured_depth_m - nodes.at(node_index - 1U).measured_depth_m,
        kMinimumSpacingM);
    const Scalar effective_axial_load_n = std::max(
        0.0,
        0.5 * (nodes.at(node_index).effective_axial_load_n +
               nodes.at(node_index - 1U).effective_axial_load_n));
    const Scalar axial_coefficient = effective_axial_load_n / spacing_m;

    for (std::size_t component_index = 0; component_index < 2U; ++component_index) {
      const std::size_t lower_dof = dof_index(node_index - 1U, component_index);
      const std::size_t upper_dof = dof_index(node_index, component_index);
      add_to_matrix(&system, lower_dof, lower_dof, axial_coefficient);
      add_to_matrix(&system, lower_dof, upper_dof, -axial_coefficient);
      add_to_matrix(&system, upper_dof, lower_dof, -axial_coefficient);
      add_to_matrix(&system, upper_dof, upper_dof, axial_coefficient);
    }
  }

  for (std::size_t node_index = 1; node_index + 1U < nodes.size(); ++node_index) {
    const Scalar spacing_left_m = std::max(
        nodes.at(node_index).measured_depth_m - nodes.at(node_index - 1U).measured_depth_m,
        kMinimumSpacingM);
    const Scalar spacing_right_m = std::max(
        nodes.at(node_index + 1U).measured_depth_m - nodes.at(node_index).measured_depth_m,
        kMinimumSpacingM);
    const Scalar average_spacing_m = 0.5 * (spacing_left_m + spacing_right_m);
    const Scalar bending_coefficient =
        nodes.at(node_index).bending_stiffness_n_m2 /
        std::max(average_spacing_m * average_spacing_m * average_spacing_m, kMinimumSpacingM);

    for (std::size_t component_index = 0; component_index < 2U; ++component_index) {
      const std::size_t lower_dof = dof_index(node_index - 1U, component_index);
      const std::size_t middle_dof = dof_index(node_index, component_index);
      const std::size_t upper_dof = dof_index(node_index + 1U, component_index);
      add_to_matrix(&system, lower_dof, lower_dof, bending_coefficient);
      add_to_matrix(&system, lower_dof, middle_dof, -2.0 * bending_coefficient);
      add_to_matrix(&system, lower_dof, upper_dof, bending_coefficient);
      add_to_matrix(&system, middle_dof, lower_dof, -2.0 * bending_coefficient);
      add_to_matrix(&system, middle_dof, middle_dof, 4.0 * bending_coefficient);
      add_to_matrix(&system, middle_dof, upper_dof, -2.0 * bending_coefficient);
      add_to_matrix(&system, upper_dof, lower_dof, bending_coefficient);
      add_to_matrix(&system, upper_dof, middle_dof, -2.0 * bending_coefficient);
      add_to_matrix(&system, upper_dof, upper_dof, bending_coefficient);
    }
  }

  if (system.node_count == 1U) {
    apply_centered_end_boundary(&system, 0U);
    return system;
  }

  apply_centered_end_boundary(&system, 0U);
  apply_centered_end_boundary(&system, system.node_count - 1U);
  return system;
}

}  // namespace centraltd
