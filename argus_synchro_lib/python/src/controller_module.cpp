#include "controller_module.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

void bind_controller(py::module& m)
{
    py::module m_controller = m.def_submodule("controller");

    py::class_<OctotreeCollisionConfig>(m_controller, "OctotreeCollisionConfig")
        .def(py::init())
        .def_property(
            "octotree_obj",
            [](const OctotreeCollisionConfig& cfg) -> OctoTree& {
                if (cfg.octotree_obj == nullptr)
                {
                    throw py::value_error("octotree_obj is not set");
                }
                return *cfg.octotree_obj;
            },
            [](OctotreeCollisionConfig& cfg, OctoTree& tree) { cfg.octotree_obj = &tree; })
        .def_readwrite("src_measure_entities", &OctotreeCollisionConfig::src_measure_entities)
        .def_readwrite("src_detect_entities", &OctotreeCollisionConfig::src_detect_entities)
        .def_readwrite("dest_entity", &OctotreeCollisionConfig::dest_entity)
        .def_readwrite("src_labels", &OctotreeCollisionConfig::src_labels)
        .def_readwrite("dest_labels", &OctotreeCollisionConfig::dest_labels)
        .def_readwrite("dialate_point_size", &OctotreeCollisionConfig::dialate_point_size)
        .def_readwrite("detect_window", &OctotreeCollisionConfig::detect_window)
        .def_readwrite("roll_angle", &OctotreeCollisionConfig::roll_angle)
        .def_readwrite("pitch_angle", &OctotreeCollisionConfig::pitch_angle)
        .def_readwrite("yaw_angle", &OctotreeCollisionConfig::yaw_angle)
        .def_readwrite("distance_threshold", &OctotreeCollisionConfig::distance_threshold)
        .def_readwrite("metric", &OctotreeCollisionConfig::metric);

    py::class_<OctotreeCollisionConfigBuilder>(m_controller, "OctotreeCollisionConfigBuilder")
        .def(py::init<NodeEntity>(), py::arg("dest_entity") = NodeEntity::OTHER)
        .def("setOctotree",
             [](OctotreeCollisionConfigBuilder& builder, OctoTree& octotree_obj) {
                 return builder.setOctotree(octotree_obj);
             },
             py::arg("octotree_obj"), py::return_value_policy::reference_internal)
        .def("setSrcMeasureEntities", &OctotreeCollisionConfigBuilder::setSrcMeasureEntities,
             py::arg("src_measure_entities"), py::return_value_policy::reference_internal)
        .def("setSrcDetectEntities", &OctotreeCollisionConfigBuilder::setSrcDetectEntities,
             py::arg("src_detect_entities"), py::return_value_policy::reference_internal)
        .def("setSrcLabels", &OctotreeCollisionConfigBuilder::setSrcLabels, py::arg("src_labels") = std::nullopt,
             py::return_value_policy::reference_internal)
        .def("setDestLabels", &OctotreeCollisionConfigBuilder::setDestLabels, py::arg("dest_labels") = std::nullopt,
             py::return_value_policy::reference_internal)
        .def("setDialatePointSize", &OctotreeCollisionConfigBuilder::setDialatePointSize,
             py::arg("dialate_point_size") = 5, py::return_value_policy::reference_internal)
        .def("setDistanceThreshold", &OctotreeCollisionConfigBuilder::setDistanceThreshold,
             py::arg("distance_threshold") = 10.0, py::return_value_policy::reference_internal)
        .def("setAngles", &OctotreeCollisionConfigBuilder::setAngles, py::arg("roll_rad") = 0.0,
             py::arg("pitch_rad") = 0.0, py::arg("yaw_rad") = 0.0, py::return_value_policy::reference_internal)
        .def("setDetectWindow", &OctotreeCollisionConfigBuilder::setDetectWindow,
             py::arg("detect_window") = Eigen::Vector3d(3.0, 3.0, 3.0), py::return_value_policy::reference_internal)
        .def("build", &OctotreeCollisionConfigBuilder::build);

    m_controller.def("rotate_x", &rotate_x, py::arg("theta"));
    m_controller.def("rotate_y", &rotate_y, py::arg("theta"));
    m_controller.def("rotate_z", &rotate_z, py::arg("theta"));
    m_controller.def("rotate_xyz", &rotate_xyz, py::arg("roll_rad_angle"), py::arg("pitch_rad_angle"),
                     py::arg("yaw_rad_angle"));
    m_controller.def("octotree_accum_points", &octotree_accum_points, py::arg("pcd_points"), py::arg("octotree_pcd"),
                     py::arg("target_entity") = NodeEntity::OTHER, py::arg("point_depth") = py::none());
    m_controller.def("transfer_movable_octotree", &transfer_movable_octotree, py::arg("octotree_obj"),
                     py::arg("octotree_points"), py::arg("roll_angle") = 0.0, py::arg("pitch_angle") = 0.0,
                     py::arg("yaw_angle") = 0.0);
    m_controller.def("remove_machine_points", &remove_machine_points, py::arg("pcd_points"), py::arg("l_machine_col"),
                     py::arg("remove_dist") = py::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0.0,
                     py::arg("pitch_angle") = 0.0, py::arg("yaw_angle") = 0.0);
    m_controller.def(
        "octotree_collision_detection",
        py::overload_cast<OctoTree&, OctoTree&, OctoTree&, const Eigen::Ref<const Eigen::MatrixXd>&, OctoTree&,
                          OctoTree&, const Eigen::Ref<const Eigen::MatrixXd>&, LayerBasedCollisionDetector&,
                          const std::optional<std::vector<int>>&, int, const std::optional<Eigen::Vector3d>&, double,
                          double, double, std::optional<double>,
                          const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>>>(
            &octotree_collision_detection),
        py::arg("octotree_pcd"), py::arg("octotree_machine_mobile_detect"), py::arg("octotree_machine_immobile_detect"),
        py::arg("machine_mobile_points_detect"), py::arg("octotree_machine_mobile_measure"),
        py::arg("octotree_machine_immobile_measure"), py::arg("machine_mobile_points_measure"),
        py::arg("collision_detector"), py::arg("dest_labels") = py::none(), py::arg("dialate_point_size") = 2,
        py::arg("detect_window") = Eigen::Vector3d(3.0, 3.0, 3.0), py::arg("roll_angle") = 0.0,
        py::arg("pitch_angle") = 0.0, py::arg("yaw_angle") = 0.0, py::arg("distance_threshold") = 10.0,
        py::arg("metric") = std::nullopt);
    m_controller.def(
        "octotree_collision_detection",
        py::overload_cast<OctoTree&, OctoTree&, OctoTree&, const Eigen::Ref<const Eigen::MatrixXd>&, OctoTree&,
                          OctoTree&, const Eigen::Ref<const Eigen::MatrixXd>&, NeighborBasedCollisionDetector&,
                          const std::optional<std::vector<int>>&, int, const std::optional<Eigen::Vector3d>&, double,
                          double, double, std::optional<double>,
                          const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>>>(
            &octotree_collision_detection),
        py::arg("octotree_pcd"), py::arg("octotree_machine_mobile_detect"), py::arg("octotree_machine_immobile_detect"),
        py::arg("machine_mobile_points_detect"), py::arg("octotree_machine_mobile_measure"),
        py::arg("octotree_machine_immobile_measure"), py::arg("machine_mobile_points_measure"),
        py::arg("collision_detector"), py::arg("dest_labels") = py::none(), py::arg("dialate_point_size") = 2,
        py::arg("detect_window") = Eigen::Vector3d(3.0, 3.0, 3.0), py::arg("roll_angle") = 0.0,
        py::arg("pitch_angle") = 0.0, py::arg("yaw_angle") = 0.0, py::arg("distance_threshold") = 10.0,
        py::arg("metric") = std::nullopt);
    m_controller.def("update_movable_entity", &update_movable_entity, py::arg("octotree_obj"),
                     py::arg("octotree_points"), py::arg("transfered_entity"), py::arg("entity_replace") = true,
                     py::arg("roll_angle") = 0.0, py::arg("pitch_angle") = 0.0, py::arg("yaw_angle") = 0.0);
    m_controller.def("octotree_collision_detection_entities",
                     py::overload_cast<LayerBasedCollisionDetector&, const OctotreeCollisionConfig&>(
                         &octotree_collision_detection_entities),
                     py::arg("collision_detector"), py::arg("cfg"));
    m_controller.def("octotree_collision_detection_entities",
                     py::overload_cast<NeighborBasedCollisionDetector&, const OctotreeCollisionConfig&>(
                         &octotree_collision_detection_entities),
                     py::arg("collision_detector"), py::arg("cfg"));
    m_controller.def("cluster_col_map_to_py", &cluster_col_map_to_py, py::arg("clusters"));
}
