#include "SceneModule.h"
#include "dataclass/camera.h"
#include "scene/common.h"
#include "scene/SceneDesc.h"
#include "scene/app_config.h"
#include "octotree/OctoTree.h"
#include "octotree/NodeEntity.h"
#include "logger/py_logger.h"

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

void bind_sceneModule(py::module& m)
{
    py::module m_scene = m.def_submodule("scene");
    py::class_<SceneDescriptionConf>(m_scene, "SceneDescriptionConf")
        .def(py::init<>())
        .def_readwrite("coarse_lo", &SceneDescriptionConf::coarse_lo)
        .def_readwrite("coarse_hi", &SceneDescriptionConf::coarse_hi)
        .def_readwrite("k_min", &SceneDescriptionConf::k_min)
        .def_readwrite("h_ref_px", &SceneDescriptionConf::h_ref_px)
        .def_readwrite("lo_gain", &SceneDescriptionConf::lo_gain)
        .def_readwrite("hi_gain", &SceneDescriptionConf::hi_gain)
        .def_readwrite("lo_floor", &SceneDescriptionConf::lo_floor)
        .def_readwrite("hi_ceil", &SceneDescriptionConf::hi_ceil)
        .def_readwrite("vertical_w_iou", &SceneDescriptionConf::vertical_w_iou)
        .def_readwrite("vertical_w_scale", &SceneDescriptionConf::vertical_w_scale)
        .def_readwrite("vertical_w_phi", &SceneDescriptionConf::vertical_w_phi)
        .def_readwrite("final_threshold", &SceneDescriptionConf::final_threshold)
        .def_readwrite("use_human_gate", &SceneDescriptionConf::use_human_gate)
        .def_readwrite("H_min", &SceneDescriptionConf::H_min)
        .def_readwrite("H_max", &SceneDescriptionConf::H_max)
        .def_readwrite("W_min", &SceneDescriptionConf::W_min)
        .def_readwrite("W_max", &SceneDescriptionConf::W_max)
        .def_readwrite("D_min", &SceneDescriptionConf::D_min)
        .def_readwrite("D_max", &SceneDescriptionConf::D_max)
        .def_readwrite("tall_ratio_min", &SceneDescriptionConf::tall_ratio_min)
        .def(py::pickle(
            [](const SceneDescriptionConf& s_c_conf)
            {
                py::dict d;
                d["coarse_lo"] = s_c_conf.coarse_lo;
                d["coarse_hi"] = s_c_conf.coarse_hi;
                d["k_min"] = s_c_conf.k_min;
                d["h_ref_px"] = s_c_conf.h_ref_px;
                d["lo_gain"] = s_c_conf.lo_gain;
                d["hi_gain"] = s_c_conf.hi_gain;
                d["lo_floor"] = s_c_conf.lo_floor;
                d["hi_ceil"] = s_c_conf.hi_ceil;
                d["vertical_w_iou"] = s_c_conf.vertical_w_iou;
                d["vertical_w_scale"] = s_c_conf.vertical_w_scale;
                d["vertical_w_phi"] = s_c_conf.vertical_w_phi;
                d["final_threshold"] = s_c_conf.final_threshold;
                d["use_human_gate"] = s_c_conf.use_human_gate;
                d["H_min"] = s_c_conf.H_min;
                d["H_max"] = s_c_conf.H_max;
                d["W_min"] = s_c_conf.W_min;
                d["W_max"] = s_c_conf.W_max;
                d["D_min"] = s_c_conf.D_min;
                d["D_max"] = s_c_conf.D_max;
                d["tall_ratio_min"] = s_c_conf.tall_ratio_min;

                return d;
            },
            [](py::dict d)
            {
                SceneDescriptionConf s_c_conf;
                s_c_conf.coarse_lo = d["coarse_lo"].cast<double>();
                s_c_conf.coarse_hi = d["coarse_hi"].cast<double>();
                s_c_conf.k_min = d["k_min"].cast<double>();
                s_c_conf.h_ref_px = d["h_ref_px"].cast<int>();
                s_c_conf.lo_gain = d["lo_gain"].cast<double>();
                s_c_conf.hi_gain = d["hi_gain"].cast<double>();
                s_c_conf.lo_floor = d["lo_floor"].cast<double>();
                s_c_conf.hi_ceil = d["hi_ceil"].cast<double>();
                s_c_conf.vertical_w_iou = d["vertical_w_iou"].cast<double>();
                s_c_conf.vertical_w_scale = d["vertical_w_scale"].cast<double>();
                s_c_conf.vertical_w_phi = d["vertical_w_phi"].cast<double>();
                s_c_conf.final_threshold = d["final_threshold"].cast<double>();
                s_c_conf.use_human_gate = d["use_human_gate"].cast<bool>();
                s_c_conf.H_min = d["H_min"].cast<double>();
                s_c_conf.H_max = d["H_max"].cast<double>();
                s_c_conf.W_min = d["W_min"].cast<double>();
                s_c_conf.W_max = d["W_max"].cast<double>();
                s_c_conf.D_min = d["D_min"].cast<double>();
                s_c_conf.D_max = d["D_max"].cast<double>();
                s_c_conf.tall_ratio_min = d["tall_ratio_min"].cast<double>();

                return s_c_conf;
            }));

    py::class_<Scene>(m_scene, "Scene")
        .def(py::init<SceneDescriptionConf, LoggerFunc>(), py::arg("scene_conf"), py::arg("logfunc"))
        .def("update", &Scene::update, py::arg("scene_conf"), py::arg("logfunc"))
        .def("integrate2d3d", &Scene::integrate2d3d, py::arg("camera"), py::arg("bb_box_data"), py::arg("boxes"),
             py::arg("minmax"), py::arg("valid_detects"), py::arg("from_entity") = NodeEntity::OTHER,
             py::arg("method") = "center")
        .def("aggregate2d3d_results", &Scene::aggregate2d3d_results, py::arg("camera_cluster2entities"),
             py::arg("octotree_obj"), py::arg("from_entity") = NodeEntity::OTHER)
        .def("append_distance_info", &Scene::append_distance_info, py::arg("collision_clusters"), py::arg("minmax"),
             py::arg("origin") = Eigen::Vector3d(0.0, 0.0, 0.0))
        .def(py::pickle(
            [](const Scene& scene)
            {
                py::dict d;
                d["coarse_lo"] = scene.coarse_lo;
                d["coarse_hi"] = scene.coarse_hi;
                d["k_min"] = scene.k_min;
                d["h_ref_px"] = scene.h_ref_px;
                d["lo_gain"] = scene.lo_gain;
                d["hi_gain"] = scene.hi_gain;
                d["lo_floor"] = scene.lo_floor;
                d["hi_ceil"] = scene.hi_ceil;
                d["vertical_w_iou"] = scene.vertical_w_iou;
                d["vertical_w_scale"] = scene.vertical_w_scale;
                d["vertical_w_phi"] = scene.vertical_w_phi;
                d["final_threshold"] = scene.final_threshold;
                d["use_human_gate"] = scene.use_human_gate;
                d["H_min"] = scene.H_min;
                d["H_max"] = scene.H_max;
                d["W_min"] = scene.W_min;
                d["W_max"] = scene.W_max;
                d["D_min"] = scene.D_min;
                d["D_max"] = scene.D_max;
                d["tall_ratio_min"] = scene.tall_ratio_min;

                return d;
            },
            [](py::dict d)
            {
                Scene scene;
                scene.coarse_lo = d["coarse_lo"].cast<double>();
                scene.coarse_hi = d["coarse_hi"].cast<double>();
                scene.k_min = d["k_min"].cast<double>();
                scene.h_ref_px = d["h_ref_px"].cast<int>();
                scene.lo_gain = d["lo_gain"].cast<double>();
                scene.hi_gain = d["hi_gain"].cast<double>();
                scene.lo_floor = d["lo_floor"].cast<double>();
                scene.hi_ceil = d["hi_ceil"].cast<double>();
                scene.vertical_w_iou = d["vertical_w_iou"].cast<double>();
                scene.vertical_w_scale = d["vertical_w_scale"].cast<double>();
                scene.vertical_w_phi = d["vertical_w_phi"].cast<double>();
                scene.final_threshold = d["final_threshold"].cast<double>();
                scene.use_human_gate = d["use_human_gate"].cast<bool>();
                scene.H_min = d["H_min"].cast<double>();
                scene.H_max = d["H_max"].cast<double>();
                scene.W_min = d["W_min"].cast<double>();
                scene.W_max = d["W_max"].cast<double>();
                scene.D_min = d["D_min"].cast<double>();
                scene.D_max = d["D_max"].cast<double>();
                scene.tall_ratio_min = d["tall_ratio_min"].cast<double>();
                return scene;
            }));
}
