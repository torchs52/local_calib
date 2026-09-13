#include "ui_interface/status_mmap.h"
#include <pybind11/chrono.h> // For handling time conversions
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // For handling STL types like std::string

namespace py = pybind11;

void bind_status_mmap_interface(py::module& m)
{
    py::module m_status_mmap = m.def_submodule("status_mmap");
    py::enum_<StatusCode>(m_status_mmap, "StatusCode")
        .value("INIT", StatusCode::INIT)
        .value("REBOOT", StatusCode::REBOOT)
        .value("BOOTING", StatusCode::BOOTING)
        .value("RUNNING", StatusCode::RUNNING)
        .value("ERROR", StatusCode::ERROR)
        .value("SHUTDOWN", StatusCode::SHUTDOWN);

    py::class_<StatusMMAP>(m_status_mmap, "StatusMMAP")
        .def(py::init<const std::string&, bool>(), py::arg("path"), py::arg("create") = false)
        .def("write_status", py::overload_cast<int>(&StatusMMAP::write_status, py::const_))
        .def("write_status", py::overload_cast<StatusCode>(&StatusMMAP::write_status, py::const_))
        .def("read_status", &StatusMMAP::read_status)
        // NOTE closeにしたとき、Cのcloseと名前が被るため、Python側のみ変更
        .def("close", &StatusMMAP::close_mmap)
        .def_static("is_recent", &StatusMMAP::is_recent, py::arg("timeout") = 5.0)
        .def_static("get_status_name", &StatusMMAP::get_status_name);
}