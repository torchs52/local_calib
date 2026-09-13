#pragma once
#include <pybind11/pybind11.h>

namespace py = pybind11;
void bind_status_mmap_interface(py::module& m);