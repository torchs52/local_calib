#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_detect3d(py::module& m);
