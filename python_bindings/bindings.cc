#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

// Include all necessary headers from the vprim library
#include "include/utility.hh"
#include "include/CubicSpiral.hh"
#include "include/SpeedMatching.hh"

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
        .def("get_T", &SpeedMatching::get_T)
        .def_property_readonly("T", &SpeedMatching::get_T) // Pythonic alias
        .def("is_ok", &SpeedMatching::is_ok)
        .def("build", &SpeedMatching::build, py::arg("start"), py::arg("goal"))
        .def("eval", &SpeedMatching::eval, py::arg("t"))
        .def("get_cost", &SpeedMatching::get_cost);
}