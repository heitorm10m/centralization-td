#pragma once

#include "centraltd/discretization.hpp"

namespace centraltd {

struct LateralEquilibriumState {
  Scalar gravity_lateral_load_n_per_m{0.0};
  Scalar curvature_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_load_n_per_m{0.0};
  Scalar equivalent_lateral_force_n{0.0};
  Scalar bending_lateral_stiffness_n_per_m{0.0};
  Scalar axial_tension_lateral_stiffness_n_per_m{0.0};
  Scalar structural_lateral_stiffness_n_per_m{0.0};
  Scalar free_eccentricity_estimate_m{0.0};
};

// The local Phase 4/5 reduced model uses a scalar lateral load q [N/m] and a
// scalar lateral displacement e [m] per segment. The bending stiffness term
// uses the equivalent relation for a simply supported beam under uniform load:
// delta_max = 5 q L^4 / (384 E I), which implies an equivalent lateral
// stiffness F / delta = 384 E I / (5 L^3) with F = q L. This is a reduced
// structural hypothesis, not a full stiff-string formulation.
LateralEquilibriumState evaluate_lateral_equilibrium(
    const MechanicalSegmentInput& segment,
    Scalar effective_axial_load_n);

}  // namespace centraltd
