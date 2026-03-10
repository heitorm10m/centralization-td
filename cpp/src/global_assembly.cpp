#include "centraltd/global_assembly.hpp"

#include <algorithm>

namespace centraltd {
namespace {

constexpr Scalar kMinimumSpacingM = 1.0e-6;

void add_to_matrix(
    GlobalLinearSystem* system,
    std::size_t row,
    std::size_t column,
    Scalar value) {
  system->stiffness_matrix[(row * system->node_count) + column] += value;
}

void apply_centered_end_boundary(GlobalLinearSystem* system, std::size_t boundary_index) {
  for (std::size_t row_index = 0; row_index < system->node_count; ++row_index) {
    system->stiffness_matrix[(row_index * system->node_count) + boundary_index] = 0.0;
  }
  for (std::size_t column_index = 0; column_index < system->node_count; ++column_index) {
    system->stiffness_matrix[(boundary_index * system->node_count) + column_index] = 0.0;
  }
  system->stiffness_matrix[(boundary_index * system->node_count) + boundary_index] = 1.0;
  system->load_vector[boundary_index] = 0.0;
}

}  // namespace

GlobalLinearSystem assemble_global_linear_system(
    const std::vector<GlobalNodeInput>& nodes,
    const std::vector<GlobalContactState>& contact_states) {
  if (nodes.size() != contact_states.size()) {
    throw ValidationError("Global assembly requires one contact state per node.");
  }

  GlobalLinearSystem system;
  system.node_count = nodes.size();
  system.stiffness_matrix.assign(system.node_count * system.node_count, 0.0);
  system.load_vector.assign(system.node_count, 0.0);

  for (std::size_t node_index = 0; node_index < nodes.size(); ++node_index) {
    const auto& node = nodes.at(node_index);
    system.load_vector[node_index] += node.equivalent_lateral_force_n;
    add_to_matrix(
        &system,
        node_index,
        node_index,
        node.centralizer_centering_stiffness_n_per_m);

    if (contact_states.at(node_index).support_contact_active) {
      add_to_matrix(
          &system,
          node_index,
          node_index,
          node.support_contact_penalty_n_per_m);
      system.load_vector[node_index] +=
          node.support_contact_penalty_n_per_m * node.support_contact_clearance_m;
    }

    if (contact_states.at(node_index).pipe_body_contact_active) {
      add_to_matrix(
          &system,
          node_index,
          node_index,
          node.body_contact_penalty_n_per_m);
      system.load_vector[node_index] +=
          node.body_contact_penalty_n_per_m * node.pipe_body_clearance_m;
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

    add_to_matrix(&system, node_index - 1U, node_index - 1U, axial_coefficient);
    add_to_matrix(&system, node_index - 1U, node_index, -axial_coefficient);
    add_to_matrix(&system, node_index, node_index - 1U, -axial_coefficient);
    add_to_matrix(&system, node_index, node_index, axial_coefficient);
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

    add_to_matrix(&system, node_index - 1U, node_index - 1U, bending_coefficient);
    add_to_matrix(&system, node_index - 1U, node_index, -2.0 * bending_coefficient);
    add_to_matrix(&system, node_index - 1U, node_index + 1U, bending_coefficient);
    add_to_matrix(&system, node_index, node_index - 1U, -2.0 * bending_coefficient);
    add_to_matrix(&system, node_index, node_index, 4.0 * bending_coefficient);
    add_to_matrix(&system, node_index, node_index + 1U, -2.0 * bending_coefficient);
    add_to_matrix(&system, node_index + 1U, node_index - 1U, bending_coefficient);
    add_to_matrix(&system, node_index + 1U, node_index, -2.0 * bending_coefficient);
    add_to_matrix(&system, node_index + 1U, node_index + 1U, bending_coefficient);
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
