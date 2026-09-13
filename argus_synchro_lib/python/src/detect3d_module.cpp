#include "detect3d_module.h"
#include "detect3d/detect3d.h"
#include "detect3d/utils.h"
#include "logger/py_logger.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

void bind_detect3d(py::module& m)
{
    py::module m_detect3d = m.def_submodule("detect3d");

    m_detect3d.def("main_accum", &main_accum, py::arg("xyz"), py::arg("debug_log"), py::arg("eps"),
                   py::arg("min_samples"), py::arg("logfunc"));

    m_detect3d.def("dbscan", &dbscan, py::arg("matrix_pc"), py::arg("eps"), py::arg("min_samples"));

    m_detect3d.def("bounding_box", &bounding_box, py::arg("mtx_pc"), py::arg("unique_labels"), py::arg("labels"));
}
