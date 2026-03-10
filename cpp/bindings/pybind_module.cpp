#include "centraltd/centralizer.hpp"
#include "centraltd/solver_stub.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/version.hpp"
#include "centraltd/well.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = "centraltd C++ core";
  m.attr("__version__") = centraltd::version_string();

  m.def("version_string", &centraltd::version_string);

  py::class_<centraltd::WellTrajectoryPoint>(m, "WellTrajectoryPoint")
      .def(py::init<>())
      .def_readwrite("measured_depth_m", &centraltd::WellTrajectoryPoint::measured_depth_m)
      .def_readwrite("inclination_rad", &centraltd::WellTrajectoryPoint::inclination_rad)
      .def_readwrite("azimuth_rad", &centraltd::WellTrajectoryPoint::azimuth_rad)
      .def_readwrite("tvd_m", &centraltd::WellTrajectoryPoint::tvd_m)
      .def_readwrite("northing_m", &centraltd::WellTrajectoryPoint::northing_m)
      .def_readwrite("easting_m", &centraltd::WellTrajectoryPoint::easting_m);

  py::class_<centraltd::WellTrajectory>(m, "WellTrajectory")
      .def(py::init<>())
      .def(py::init<std::vector<centraltd::WellTrajectoryPoint>>(), py::arg("points"))
      .def("validate", &centraltd::WellTrajectory::validate)
      .def("points", [](const centraltd::WellTrajectory& well) { return well.points(); })
      .def("size", &centraltd::WellTrajectory::size)
      .def("empty", &centraltd::WellTrajectory::empty)
      .def("final_measured_depth_m", &centraltd::WellTrajectory::final_measured_depth_m);

  py::class_<centraltd::StringSection>(m, "StringSection")
      .def(py::init<>())
      .def_readwrite("name", &centraltd::StringSection::name)
      .def_readwrite("length_m", &centraltd::StringSection::length_m)
      .def_readwrite("outer_diameter_m", &centraltd::StringSection::outer_diameter_m)
      .def_readwrite("inner_diameter_m", &centraltd::StringSection::inner_diameter_m)
      .def_readwrite("unit_weight_n_per_m", &centraltd::StringSection::unit_weight_n_per_m)
      .def_readwrite("grade", &centraltd::StringSection::grade)
      .def("validate", &centraltd::StringSection::validate);

  py::class_<centraltd::CentralizerSpec>(m, "CentralizerSpec")
      .def(py::init<>())
      .def_readwrite("name", &centraltd::CentralizerSpec::name)
      .def_readwrite("outer_diameter_m", &centraltd::CentralizerSpec::outer_diameter_m)
      .def_readwrite("start_md_m", &centraltd::CentralizerSpec::start_md_m)
      .def_readwrite("spacing_m", &centraltd::CentralizerSpec::spacing_m)
      .def_readwrite("count", &centraltd::CentralizerSpec::count)
      .def_readwrite("type", &centraltd::CentralizerSpec::type)
      .def("validate", &centraltd::CentralizerSpec::validate);

  py::class_<centraltd::SolverStubInput>(m, "SolverStubInput")
      .def(py::init<>())
      .def_readwrite("well", &centraltd::SolverStubInput::well)
      .def_readwrite("string_sections", &centraltd::SolverStubInput::string_sections)
      .def_readwrite("centralizers", &centraltd::SolverStubInput::centralizers)
      .def("validate", &centraltd::SolverStubInput::validate);

  py::class_<centraltd::SolverStubResult>(m, "SolverStubResult")
      .def(py::init<>())
      .def_readwrite("status", &centraltd::SolverStubResult::status)
      .def_readwrite("message", &centraltd::SolverStubResult::message)
      .def_readwrite("estimated_hookload_n", &centraltd::SolverStubResult::estimated_hookload_n)
      .def_readwrite(
          "estimated_surface_torque_n_m",
          &centraltd::SolverStubResult::estimated_surface_torque_n_m)
      .def_readwrite("minimum_standoff_ratio", &centraltd::SolverStubResult::minimum_standoff_ratio)
      .def_readwrite("contact_nodes", &centraltd::SolverStubResult::contact_nodes)
      .def_readwrite("todos", &centraltd::SolverStubResult::todos);

  m.def("run_solver_stub", &centraltd::run_solver_stub, py::arg("input"));
}

