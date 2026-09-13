#pragma once

#include "octotree/OctoNode.h"
#include "octotree/OctoTree.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_octotree(py::module& m);
