#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

// Include all necessary headers from the vprim library
#include "include/utility.hh"
#include "include/CubicSpiral.hh"
#include "include/SpeedMatching.hh"
#include "include/SpeedMatchingSickness.hh"
#include "include/ComfortMatching.hh"

namespace py = pybind11;
using namespace vprim;

PYBIND11_MODULE(vprim_py, m) {
    m.doc() = "Python bindings for the Vehicle Primitives library";

    // From utility.hh
    py::class_<RootSolverOptions>(m, "RootSolverOptions")
        .def(py::init<>())
        .def_readwrite("max_iter", &RootSolverOptions::max_iter)
        .def_readwrite("alpha", &RootSolverOptions::alpha)
        .def_readwrite("step_tolerance", &RootSolverOptions::step_tolerance)
        .def_readwrite("func_tolerance", &RootSolverOptions::func_tolerance);

    py::class_<Polynomial>(m, "Polynomial")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("degree"))
        .def(py::init<std::initializer_list<double>>())
        .def_property_readonly("degree", &Polynomial::degree)
        .def("eval", &Polynomial::eval, py::arg("x"))
        .def("derivative", static_cast<Polynomial (Polynomial::*)() const>(&Polynomial::derivative))
        .def("derivative", static_cast<Polynomial (Polynomial::*)(int) const>(&Polynomial::derivative), py::arg("n"))
        .def("closest_root", &Polynomial::closest_root, py::arg("initial_guess"), py::arg("options") = RootSolverOptions{})
        .def("__getitem__", static_cast<const double& (Polynomial::*)(size_t) const>(&Polynomial::operator[]), py::is_operator())
        .def("__setitem__", static_cast<double& (Polynomial::*)(size_t)>(&Polynomial::operator[]), py::is_operator())
        .def("__repr__", [](const Polynomial &p) { return "<vprim.Polynomial of degree " + std::to_string(p.degree()) + ">"; });

    // From CubicSpiral.hh
    py::class_<SpatialState>(m, "SpatialState")
        .def(py::init<>())
        .def(py::init<double, double, double, double>(), py::arg("x"), py::arg("y"), py::arg("theta"), py::arg("kappa"))
        .def_readwrite("x", &SpatialState::x)
        .def_readwrite("y", &SpatialState::y)
        .def_readwrite("theta", &SpatialState::theta)
        .def_readwrite("kappa", &SpatialState::kappa)
        .def("__repr__", [](const SpatialState &s) {
            return "<vprim.SpatialState x=" + std::to_string(s.x) + ", y=" + std::to_string(s.y) + ", th=" + std::to_string(s.theta) + ">";
        });

    py::class_<CubicSpiral>(m, "CubicSpiral")
        .def(py::init<>())
        .def("build", &CubicSpiral::build, py::arg("start"), py::arg("goal"), "Builds the spiral from start to goal states.")
        .def("eval", &CubicSpiral::eval, py::arg("s"), "Evaluates the spiral at arc length 's'.")
        .def("get_cost", &CubicSpiral::get_cost, "Returns the total arc length of the spiral.")
        .def_property_readonly("length", &CubicSpiral::get_cost); // Pythonic alias for get_cost

    // From SpeedMatching.hh
    py::class_<LongitudinalState>(m, "LongitudinalState")
        .def(py::init<>())
        .def(py::init<double, double, double, double>(), py::arg("s"), py::arg("v"), py::arg("a"), py::arg("j"))
        .def_readwrite("s", &LongitudinalState::s)
        .def_readwrite("v", &LongitudinalState::v)
        .def_readwrite("a", &LongitudinalState::a)
        .def_readwrite("j", &LongitudinalState::j)
        .def("__repr__", [](const LongitudinalState &s) {
            return "<vprim.LongitudinalState s=" + std::to_string(s.s) + ", v=" + std::to_string(s.v) + ", a=" + std::to_string(s.a) + ">";
        });

    py::class_<SpeedMatching>(m, "SpeedMatching")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("weight_time"))
        .def("set_time_weight", &SpeedMatching::set_time_weight, py::arg("wT"))
        .def("set_jerk_limit", &SpeedMatching::set_jerk_limit, py::arg("jmax"))
        .def("get_T", &SpeedMatching::get_T)
        .def_property_readonly("T", &SpeedMatching::get_T) // Pythonic alias
        .def("is_ok", &SpeedMatching::is_ok)
        .def("build", &SpeedMatching::build, py::arg("start"), py::arg("goal"))
        .def("eval", &SpeedMatching::eval, py::arg("t"))
        .def("get_cost", &SpeedMatching::get_cost);

    // From SpeedMatchingSickness.hh
    py::class_<LTIFilter>(m, "LTIFilter")
        .def(py::init<>())
        .def_readwrite("n", &LTIFilter::n)
        .def_readwrite("A", &LTIFilter::A)
        .def_readwrite("B", &LTIFilter::B)
        .def_readwrite("C", &LTIFilter::C);

    m.def("default_motion_sickness_filter", &default_motion_sickness_filter,
          "Default 2nd-order ISO 2631-1 Wf approximation used throughout "
          "motion_sickness_filter_minT_notes.md.");

    py::class_<SpeedMatchingSickness>(m, "SpeedMatchingSickness")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("weight_time"))
        .def("set_time_weight", &SpeedMatchingSickness::set_time_weight, py::arg("wT"))
        .def("set_accel_scale", &SpeedMatchingSickness::set_accel_scale, py::arg("scale"))
        .def("set_filter", &SpeedMatchingSickness::set_filter, py::arg("filter"))
        .def("set_filter_initial_state", &SpeedMatchingSickness::set_filter_initial_state, py::arg("q0"))
        .def("set_search_bounds", &SpeedMatchingSickness::set_search_bounds,
             py::arg("Tmin"), py::arg("Tmax"), py::arg("n_points") = 400)
        .def("get_T", &SpeedMatchingSickness::get_T)
        .def_property_readonly("T", &SpeedMatchingSickness::get_T)
        .def("is_ok", &SpeedMatchingSickness::is_ok)
        .def("build", &SpeedMatchingSickness::build, py::arg("start"), py::arg("goal"))
        .def("eval", &SpeedMatchingSickness::eval, py::arg("t"))
        .def("get_cost", &SpeedMatchingSickness::get_cost)
        .def("AW", &SpeedMatchingSickness::AW, py::arg("t"));

    // From ComfortMatching.hh
    py::class_<ComfortMatching>(m, "ComfortMatching")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("weight_time"))
        .def("set_time_weight", &ComfortMatching::set_time_weight, py::arg("wT"))
        .def("set_jerk_weight", &ComfortMatching::set_jerk_weight, py::arg("w_j"))
        .def("set_jerk_limit", &ComfortMatching::set_jerk_limit, py::arg("jmax"))
        .def("set_accel_weight", &ComfortMatching::set_accel_weight, py::arg("w_a"))
        .def("set_accel_limit", &ComfortMatching::set_accel_limit, py::arg("a_max"))
        .def("set_filter", &ComfortMatching::set_filter, py::arg("filter"))
        .def("set_filter_initial_state", &ComfortMatching::set_filter_initial_state, py::arg("q0"))
        .def("set_search_bounds", &ComfortMatching::set_search_bounds,
             py::arg("Tmin"), py::arg("Tmax"), py::arg("n_points") = 400)
        .def("get_T", &ComfortMatching::get_T)
        .def_property_readonly("T", &ComfortMatching::get_T)
        .def("is_ok", &ComfortMatching::is_ok)
        .def("build", &ComfortMatching::build, py::arg("start"), py::arg("goal"))
        .def("eval", &ComfortMatching::eval, py::arg("t"))
        .def("get_cost", &ComfortMatching::get_cost)
        .def("AW", &ComfortMatching::AW, py::arg("t"));
}