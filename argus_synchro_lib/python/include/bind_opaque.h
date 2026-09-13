#pragma once

#include "octotree/AbstractCollisionDetector.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoNode.h"

#include <map>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <tuple>
#include <variant>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::map<std::variant<int, std::string>, std::map<std::tuple<int, int, int>, OctoNode>>);
PYBIND11_MAKE_OPAQUE(std::map<std::string, std::variant<OctoNode, LiDARCoord, double>>);
