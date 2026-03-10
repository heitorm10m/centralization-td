#pragma once

#include "centraltd/discretization.hpp"

#include <vector>

namespace centraltd {

struct GlobalNodeInput {
  Index node_index{0};
  Scalar measured_depth_m{0.0};
  Scalar segment_length_m{0.0};
  Scalar inclination_rad{0.0};
  Scalar curvature_rad_per_m{0.0};
  Scalar effective_line_weight_n_per_m{0.0};
  Scalar effective_axial_load_n{0.0};
  Scalar bending_stiffness_n_m2{0.0};
  Scalar bending_moment_n_m{0.0};
  Scalar bending_stress_pa{0.0};
  Scalar bending_strain_estimate{0.0};
  Scalar gravity_lateral_load_n_per_m{0.0};
  Scalar curvature_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_force_n{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Scalar centralizer_centering_stiffness_n_per_m{0.0};
  Scalar support_contact_penalty_n_per_m{0.0};
  Scalar body_contact_penalty_n_per_m{0.0};
  Scalar support_outer_diameter_m{0.0};
  Scalar pipe_body_clearance_m{0.0};
  Scalar support_contact_clearance_m{0.0};
  std::size_t nearby_centralizer_count{0};
  StringSection section;
};

struct GlobalContactState {
  bool support_contact_active{false};
  bool pipe_body_contact_active{false};
};

struct GlobalLinearSystem {
  std::size_t node_count{0};
  std::vector<Scalar> stiffness_matrix;
  std::vector<Scalar> load_vector;
};

// Reduced global scalar model used in Phase 5:
// K(e) u = f_lat + f_contact(e),
// where u is the nodal eccentricity-like displacement magnitude along MD.
// The assembled operator combines discrete bending, axial-tension geometric
// stiffening, nominal centralizer spring support, and simple penalty contact.
GlobalLinearSystem assemble_global_linear_system(
    const std::vector<GlobalNodeInput>& nodes,
    const std::vector<GlobalContactState>& contact_states);

}  // namespace centraltd
