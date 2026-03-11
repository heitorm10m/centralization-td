#pragma once

#include "centraltd/discretization.hpp"

#include <vector>

namespace centraltd {

struct VectorNodeInput {
  Index node_index{0};
  Scalar measured_depth_m{0.0};
  Scalar segment_length_m{0.0};
  Scalar effective_axial_load_n{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  Scalar bending_moment_n_m{0.0};
  Scalar bending_stress_pa{0.0};
  Scalar bending_strain_estimate{0.0};
  Vector2 gravity_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 curvature_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 equivalent_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 equivalent_lateral_force_n_b{0.0, 0.0};
  Scalar equivalent_lateral_load_magnitude_n_per_m{0.0};
  Scalar equivalent_lateral_force_magnitude_n{0.0};
  Scalar bending_lateral_stiffness_n_per_m{0.0};
  Scalar axial_tension_lateral_stiffness_n_per_m{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Scalar centralizer_centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  Scalar body_contact_penalty_n_per_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  std::size_t nearby_centralizer_count{0};
  Vector2 free_displacement_n_b_m{0.0, 0.0};
  StringSection section;
};

struct VectorContactState {
  bool support_contact_active{false};
  bool pipe_body_contact_active{false};
  Vector2 contact_direction_n_b{1.0, 0.0};
};

struct VectorLinearSystem {
  std::size_t node_count{0};
  std::size_t dof_count{0};
  std::vector<Scalar> stiffness_matrix;
  std::vector<Scalar> load_vector;
};

VectorLinearSystem assemble_vector_global_linear_system(
    const std::vector<VectorNodeInput>& nodes,
    const std::vector<VectorContactState>& contact_states);

}  // namespace centraltd
