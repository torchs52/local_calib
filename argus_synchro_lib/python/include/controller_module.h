#pragma once

#include "octotree/controller.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_controller(py::module& m);