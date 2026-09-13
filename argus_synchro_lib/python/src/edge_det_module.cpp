#include "edge_det_module.h"
#include "octotree/transform.h"
#include "octotree/edge_utility.h"
#include "octotree/edge_detector.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

void bind_edge_det(py::module& m)
{
    py::module m_edge_det = m.def_submodule("edge_det");
    py::class_<edge_det::EdgeDetectionResult>(m_edge_det, "EdgeDetectionResult")
        .def(py::init<int, double, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi>(), py::arg("frame"),
             py::arg("time"), py::arg("edge_points"), py::arg("edge_lines"), py::arg("edge_lenth"))
        .def_property_readonly("frame", [](const edge_det::EdgeDetectionResult& r) { return r.frame; })
        .def_property_readonly("time", [](const edge_det::EdgeDetectionResult& r) { return r.time; })
        .def_property_readonly("edge_points", [](const edge_det::EdgeDetectionResult& r) { return r.edge_points; })
        .def_property_readonly("edge_lines", [](const edge_det::EdgeDetectionResult& r) { return r.edge_lines; })
        .def_property_readonly("edge_length", [](const edge_det::EdgeDetectionResult& r) { return r.edge_length; })
        .def("get_edge_points_on_ground", &edge_det::EdgeDetectionResult::get_edge_points_on_ground)
        .def("get_edge_cluster", &edge_det::EdgeDetectionResult::get_edge_cluster);

    py::class_<edge_det::EdgeDetectionConfig>(m_edge_det, "EdgeDetectionConfig")
        .def(py::init<float>(), py::arg("target_edge_dist_th") = 20.0)
        .def_property_readonly("target_edge_dist_th",
                               [](const edge_det::EdgeDetectionConfig& r) { return r.target_edge_dist_th; });

    py::class_<edge_det::EdgeDetectorCpp>(m_edge_det, "EdgeDetector")
        .def(py::init<>())
        .def("main", &edge_det::EdgeDetectorCpp::main, py::arg("octotree_obj"), py::arg("target_entities"))
        .def("update", &edge_det::EdgeDetectorCpp::update, py::arg("edge_conf"));

    py::enum_<edge_det::DediscretizeMethod>(m_edge_det, "DediscretizeMethod")
        .value("MED", edge_det::DediscretizeMethod::MED)
        .value("MIN", edge_det::DediscretizeMethod::MIN)
        .value("MAX", edge_det::DediscretizeMethod::MAX)
        .export_values();

    py::enum_<edge_det::BevCoord>(m_edge_det, "BevCoord")
        .value("CARTESIAN", edge_det::BevCoord::CARTESIAN)
        .value("POLAR", edge_det::BevCoord::POLAR)
        .export_values();

    py::enum_<edge_det::AggName>(m_edge_det, "AggName")
        .value("MEAN", edge_det::AggName::MEAN)
        .value("MAX", edge_det::AggName::MAX)
        .value("MIN", edge_det::AggName::MIN)
        .value("LAST", edge_det::AggName::LAST)
        .export_values();

    m_edge_det.def("scale_value", &edge_det::scale_value, py::arg("mat"), py::arg("from_min"), py::arg("from_max"),
                   py::arg("to_min") = 0.00, py::arg("to_max") = 255.0);

    m_edge_det.def("octotree2bev", &edge_det::octotree2bev, py::arg("octotree_obj"), py::arg("fwd_range"),
                   py::arg("side_range"), py::arg("grid_size"), py::arg("bev_shape"), py::arg("target_entities"),
                   py::arg("bev_depth") = std::nullopt, py::arg("bev_coord") = edge_det::BevCoord::POLAR,
                   py::arg("agg_method") = edge_det::AggName::MAX, py::arg("coord_origin") = std::make_tuple(0, 0, 0),
                   py::arg("discrete_origin") = std::make_tuple(0, -1 * edge_det::PI), py::arg("scaled") = true,
                   py::arg("min_scale_z") = -1.88, py::arg("max_scale_z") = -0.88, py::arg("min_bev_val") = 0.0,
                   py::arg("max_bev_val") = 255.0, py::arg("nan_fill_value") = 0);

    m_edge_det.def("real_to_grid", &edge_det::real_to_grid, py::arg("real_coords_2d"), py::arg("grid_size"),
                   py::arg("real_offset") = std::make_tuple(0.0, 0.0), py::arg("grid_offset") = std::make_tuple(0, 0));

    m_edge_det.def("grid_to_real", &edge_det::grid_to_real, py::arg("grid_coords_2d"), py::arg("grid_size"),
                   py::arg("grid_offset") = std::make_tuple(0, 0), py::arg("real_offset") = std::make_tuple(0.0, 0.0),
                   py::arg("repr_method") = edge_det::DediscretizeMethod::MED);

    m_edge_det.def("cartesian_to_polar", &edge_det::cartesian_to_polar, py::arg("points"), py::arg("polar_origin"));

    m_edge_det.def("polar_to_cartesian", &edge_det::polar_to_cartesian, py::arg("real_polar_coords"),
                   py::arg("from_polar_origin"));

    m_edge_det.def("mask_img_by_value", &edge_det::mask_img_by_value_py, py::arg("src"), py::arg("low_value") = 50.0f,
                   py::arg("high_value") = 200.0f);
}
