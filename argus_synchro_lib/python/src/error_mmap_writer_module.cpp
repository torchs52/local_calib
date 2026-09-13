#include "error_mmap_writer/error_mmap_writer.h"
#include "logger/py_logger.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/buffer_info.h>
#include <pybind11/functional.h>

namespace py = pybind11;

static void write_state(ErrorMMapWriter& self, py::buffer b)
{
    py::buffer_info info = b.request();
    if (info.itemsize != 1)
    {
        throw std::invalid_argument("state bytes: itemsize must be 1");
    }
    if (info.ndim != 1)
    {
        throw std::invalid_argument("state bytes must be 1-dimensional");
    }

    const auto* p = static_cast<const uint8_t*>(info.ptr);
    const size_t n = static_cast<size_t>(info.size);

    self.writeStateError(p, n);
}

static void write_action(ErrorMMapWriter& self, py::buffer b)
{
    py::buffer_info info = b.request();
    if (info.itemsize != 1)
    {
        throw std::invalid_argument("action bytes: itemsize must be 1");
    }
    if (info.ndim != 1)
    {
        throw std::invalid_argument("action bytes must be 1-dimensional");
    }

    const auto* p = static_cast<const uint8_t*>(info.ptr);
    const size_t n = static_cast<size_t>(info.size);

    self.writeActionError(p, n);
}

static void write_status(ErrorMMapWriter& self, py::buffer b)
{
    py::buffer_info info = b.request();
    if (info.itemsize != 1)
    {
        throw std::invalid_argument("status bytes: itemsize must be 1");
    }
    if (info.ndim != 1)
    {
        throw std::invalid_argument("status bytes must be 1-dimensional");
    }

    const auto* p = static_cast<const uint8_t*>(info.ptr);
    const size_t n = static_cast<size_t>(info.size);

    self.writeStatus(p, n);
}

void bind_error_mmap_writer(py::module& m)
{
    py::module error_mmap_writer = m.def_submodule("error_mmap_writer");
    error_mmap_writer.doc() = "pybind11 bindings for error_mmap_writer";

    py::class_<ErrorMMapWriter>(error_mmap_writer, "ErrorMMapWriter")
        .def(py::init<const std::vector<std::string>&, LoggerFunc>(), py::arg("paths"), py::arg("logfunc"))
        .def("init", &ErrorMMapWriter::init)
        .def("start_write", &ErrorMMapWriter::start_write)
        .def("rotate_if_busy", &ErrorMMapWriter::rotate_if_busy)
        .def("write_state_error", &write_state, py::arg("data"))
        .def("write_action_error", &write_action, py::arg("data"))
        .def("write_status", &write_status, py::arg("data"))
        .def("close", &ErrorMMapWriter::close);
}