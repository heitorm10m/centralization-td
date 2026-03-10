#include "centraltd/solver_stub.hpp"

#include "centraltd/types.hpp"

#include <algorithm>
#include <numeric>

namespace centraltd {
namespace {

Scalar total_string_weight_n(const std::vector<StringSection>& sections) {
  return std::accumulate(
      sections.begin(), sections.end(), 0.0,
      [](Scalar total, const StringSection& section) {
        return total + (section.length_m * section.unit_weight_n_per_m);
      });
}

Scalar total_string_length_m(const std::vector<StringSection>& sections) {
  return std::accumulate(
      sections.begin(), sections.end(), 0.0,
      [](Scalar total, const StringSection& section) {
        return total + section.length_m;
      });
}

std::size_t total_centralizer_count(const std::vector<CentralizerSpec>& centralizers) {
  return std::accumulate(
      centralizers.begin(), centralizers.end(), std::size_t{0},
      [](std::size_t total, const CentralizerSpec& spec) {
        return total + spec.count;
      });
}

}  // namespace

void SolverStubInput::validate() const {
  well.validate();

  if (string_sections.empty()) {
    throw ValidationError("At least one string section is required.");
  }

  for (const auto& section : string_sections) {
    section.validate();
  }

  for (const auto& spec : centralizers) {
    spec.validate();
  }
}

SolverStubResult run_solver_stub(const SolverStubInput& input) {
  input.validate();

  const Scalar hookload_n = total_string_weight_n(input.string_sections);
  const Scalar string_length_m = total_string_length_m(input.string_sections);
  const Scalar final_md_m = input.well.final_measured_depth_m();
  const std::size_t centralizer_count = total_centralizer_count(input.centralizers);

  // Keep the stub deterministic so the repo can verify data flow before real mechanics land.
  const Scalar coverage_ratio =
      string_length_m > 0.0
          ? std::min<Scalar>(1.0, (static_cast<Scalar>(centralizer_count) * 15.0) / string_length_m)
          : 0.0;

  SolverStubResult result;
  result.status = "stub";
  result.message =
      "Phase 1 placeholder output. Values are deterministic scaffolding, not calibrated physics.";
  result.estimated_hookload_n = hookload_n;
  result.estimated_surface_torque_n_m = 0.02 * hookload_n * (1.0 + (final_md_m / 1000.0));
  result.minimum_standoff_ratio =
      centralizer_count == 0 ? 0.0 : std::min<Scalar>(0.95, 0.10 + coverage_ratio);
  result.contact_nodes = input.well.size() > 1 ? input.well.size() - 1 : 0;
  result.todos = {
      "TODO: implement stiff-string formulation.",
      "TODO: implement contact detection and reaction forces.",
      "TODO: implement standoff evaluation from contact geometry.",
      "TODO: implement detailed bow-spring centralizer model.",
      "TODO: implement torque and drag calculations.",
      "TODO: implement design-space optimization workflow.",
  };

  return result;
}

}  // namespace centraltd

