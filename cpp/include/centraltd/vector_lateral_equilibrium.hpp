#pragma once

#include "centraltd/discretization.hpp"

namespace centraltd {

struct VectorLateralEquilibriumState {
  Vector2 gravity_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 curvature_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 equivalent_lateral_load_n_b_n_per_m{0.0, 0.0};
  Vector2 equivalent_lateral_force_n_b{0.0, 0.0};
  Scalar equivalent_lateral_load_magnitude_n_per_m{0.0};
  Scalar equivalent_lateral_force_magnitude_n{0.0};
  Scalar bending_lateral_stiffness_n_per_m{0.0};
  Scalar axial_tension_lateral_stiffness_n_per_m{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Vector2 free_displacement_n_b_m{0.0, 0.0};
};

VectorLateralEquilibriumState evaluate_vector_lateral_equilibrium(
    const MechanicalSegmentInput& segment,
    Scalar effective_axial_load_n);

}  // namespace centraltd
