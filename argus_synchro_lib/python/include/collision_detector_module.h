#pragma once

#include "octotree/AbstractCollisionDetector.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// 抽象クラスをpybind11で使用するために必要
template <typename T> class PyAbstractCollisionDetector : public AbstractCollisionDetector<T>
{
  public:
    /* Inherit the constructors */
    using AbstractCollisionDetector<T>::AbstractCollisionDetector;

    /* Trampoline (need one for each virtual function) */
    std::set<T> create_dialation_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes, int window) override
    {
        PYBIND11_OVERRIDE_PURE(std::set<T>, AbstractCollisionDetector<T>, create_dialation_coord, nodes, window);
    }

    std::set<T> create_dest_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes, int window) override
    {
        PYBIND11_OVERRIDE_PURE(std::set<T>, AbstractCollisionDetector<T>, create_dest_coord, nodes, window);
    }

    bool collision_detection(const std::set<T>& src_coords, const std::set<T>& dest_coords) override
    {
        PYBIND11_OVERRIDE_PURE(bool, AbstractCollisionDetector<T>, collision_detection, src_coords, dest_coords);
    }
};

template <typename T> void bind_abstract_collision_detector(py::module& m_collision_detector, const char* name)
{
    py::class_<AbstractCollisionDetector<T>, PyAbstractCollisionDetector<T>>(m_collision_detector, name)
        .def_readwrite("coord_method", &AbstractCollisionDetector<T>::coord_method)
        .def_readwrite("octotree_obj", &AbstractCollisionDetector<T>::octotree_obj)
        .def(py::init<CoordMethod>(), py::arg("coord_method") = CoordMethod::VOX_MED)
        .def("assign_octotree", &AbstractCollisionDetector<T>::assign_octotree, py::arg("octotree_obj"))
        .def("detect_collided_object", &AbstractCollisionDetector<T>::detect_collided_object,
             py::arg("src_detect_label"), py::arg("src_measure_label"), py::arg("dest_labels") = py::none(),
             py::arg("window") = 2, py::arg("detect_window") = Eigen::Vector3d(3.0, 3.0, 3.0),
             py::arg("distance_threshold") = 10.0, py::arg("metric") = py::none())
        .def("_keynode2array", &AbstractCollisionDetector<T>::_keynode2array, py::arg("keynodes"))
        .def("_mean2array", &AbstractCollisionDetector<T>::_mean2array, py::arg("keynodes"))
        .def("_far2array", &AbstractCollisionDetector<T>::_far2array, py::arg("keynodes"))
        .def("_near2array", &AbstractCollisionDetector<T>::_near2array, py::arg("keynodes"))
        .def("_quantile2array", &AbstractCollisionDetector<T>::_quantile2array, py::arg("keynodes"))
        .def("_get_vox_w_pairs", &AbstractCollisionDetector<T>::_get_vox_w_pairs, py::arg("src_nodes"),
             py::arg("dest_nodes"))
        .def("find_minimum_node_pair", &AbstractCollisionDetector<T>::find_minimum_node_pair, py::arg("src_nodes"),
             py::arg("dest_nodes"), py::arg("distance_threshold"), py::arg("metric") = py::none())
        .def("create_dialation_coord", &AbstractCollisionDetector<T>::create_dialation_coord, py::arg("nodes"),
             py::arg("window") = 2)
        .def("create_dest_coord", &AbstractCollisionDetector<T>::create_dest_coord, py::arg("nodes"),
             py::arg("window") = 2)
        .def("collision_detection", &AbstractCollisionDetector<T>::collision_detection, py::arg("src_coords"),
             py::arg("dest_coords"));
}

void bind_collision_detector(py::module& m);
