#include "collision_detector_module.h"
#include "octotree/AbstractCollisionDetector.h"
#include "octotree/LayerBasedCollisionDetector.h"
#include "octotree/NeighborBasedCollisionDetector.h"
#include "octotree/OctoNode.h"
#include "bind_opaque.h"

#include <pybind11/stl.h>

py::dict CollisionDetResultmap_to_pydict(const std::map<std::string, std::variant<OctoNode, LiDARCoord, double>>& d)
{
    py::dict py_d;
    py_d["src_node"] = d.at("src_node");
    py_d["dest_node"] = d.at("dest_node");
    py_d["src_coord"] = d.at("src_coord");
    py_d["dest_coord"] = d.at("dest_coord");
    py_d["src_dest_dist"] = d.at("src_dest_dist");
    return py_d;
}

void bind_collision_detector(py::module& m)
{
    py::module m_collision_detector = m.def_submodule("collision_detector");

    py::class_<CollisionDetResult>(m_collision_detector, "CollisionDetResult")
        .def(py::init<OctoNode, OctoNode, LiDARCoord, LiDARCoord, double>())
        .def_readwrite("src_node", &CollisionDetResult::src_node)
        .def_readwrite("dest_node", &CollisionDetResult::dest_node)
        .def_readwrite("src_coord", &CollisionDetResult::src_coord)
        .def_readwrite("dest_coord", &CollisionDetResult::dest_coord)
        .def_readwrite("src_dest_dist", &CollisionDetResult::src_dest_dist)
        .def("to_dict", [](const CollisionDetResult& c) { return CollisionDetResultmap_to_pydict(c.to_dict()); })
        .def("to_tuple", &CollisionDetResult::to_tuple)
        .def("__repr__", [](const CollisionDetResult& c) { return CollisionDetResultmap_to_pydict(c.to_dict()); });

    py::enum_<CoordMethod>(m_collision_detector, "CoordMethod")
        .value("VOX_MED", CoordMethod::VOX_MED)
        .value("MEAN", CoordMethod::MEAN)
        .value("FAR_POINT", CoordMethod::FAR_POINT)
        .value("NEAR_POINT", CoordMethod::NEAR_POINT)
        .value("QUANTILE", CoordMethod::QUANTILE)
        .export_values()
        .def_static("from_string", &coord_method_from_string, py::arg("name"));

    // AbstractCollisionDetector
    bind_abstract_collision_detector<int>(m_collision_detector, "AbstractCollisionDetectorBaseInt");
    bind_abstract_collision_detector<std::tuple<int, int, int>>(m_collision_detector,
                                                                "AbstractCollisionDetectorBaseTupleIntIntInt");

    // MachineCollisionBase
    py::class_<LayerBasedCollisionDetector, AbstractCollisionDetector<int>>(m_collision_detector,
                                                                            "LayerBasedCollisionDetector")
        .def(py::init<CoordMethod>(),
             // py::arg("distance_threshold") = 10.0,
             py::arg("coord_method") = CoordMethod::VOX_MED);

    // MachineCollisionBase
    py::class_<NeighborBasedCollisionDetector, AbstractCollisionDetector<std::tuple<int, int, int>>>(
        m_collision_detector, "NeighborBasedCollisionDetector")
        .def(py::init<CoordMethod>(),
             // py::arg("distance_threshold") = 10.0,
             py::arg("coord_method") = CoordMethod::VOX_MED);
}
