#include "ui_interface/GeneralConf.h"
#include "ui_interface/UIIFConf.h"
#include "ui_interface/ui_interface.h"
#include "logger/py_logger.h"

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

void bind_ui_interface(py::module& m)
{
    py::module m_ui_interface = m.def_submodule("ui_interface");
    m_ui_interface.doc() = "pybind11 bindings for UI_interface and UIIFConf";

    py::enum_<CLIFF_LEVEL>(m_ui_interface, "CLIFF_LEVEL")
        .value("NORMAL", CLIFF_LEVEL::NORMAL)
        .value("ATTENTION", CLIFF_LEVEL::ATTENTION)
        .value("WARNING", CLIFF_LEVEL::WARNING);

    py::class_<UIIFConf>(m_ui_interface, "UIIFConf")
        .def(py::init<bool, int, double, const std::vector<std::string>&, const std::vector<std::string>&, bool, double,
                      double, double, double, double, bool, bool>(),
             py::arg("damp_out"), py::arg("bbox_3d_num"), py::arg("bbox_3d_distance"), py::arg("UI_mmap"),
             py::arg("damp_mmap"), py::arg("show_unk"), py::arg("collision_depict_dist"),
             py::arg("collision_attention_dist"), py::arg("collision_warning_dist"), py::arg("cliff_attention_dist"),
             py::arg("cliff_warning_dist"), py::arg("draw_bbox_3d"), py::arg("draw_collision"))
        .def_readwrite("damp_out", &UIIFConf::damp_out)
        .def_readwrite("bbox_3d_num", &UIIFConf::bbox_3d_num)
        .def_readwrite("bbox_3d_distance", &UIIFConf::bbox_3d_distance)
        .def_readwrite("UI_mmap", &UIIFConf::UI_mmap)
        .def_readwrite("damp_mmap", &UIIFConf::damp_mmap)
        .def_readwrite("show_unk", &UIIFConf::show_unk)
        .def_readwrite("collision_depict_dist", &UIIFConf::collision_depict_dist)
        .def_readwrite("collision_attention_dist", &UIIFConf::collision_attention_dist)
        .def_readwrite("collision_warning_dist", &UIIFConf::collision_warning_dist)
        .def_readwrite("cliff_attention_dist", &UIIFConf::cliff_attention_dist)
        .def_readwrite("cliff_warning_dist", &UIIFConf::cliff_warning_dist)
        .def_readwrite("draw_bbox_3d", &UIIFConf::draw_bbox_3d)
        .def_readwrite("draw_collision", &UIIFConf::draw_collision);

    py::class_<GeneralConf>(m_ui_interface, "GeneralConf")
        .def(py::init<bool, int, bool, double, double, double, double, const std::string&>(), py::arg("in_factory"),
             py::arg("operation_mode"), py::arg("has_external_guard"), py::arg("external_guard_offset"),
             py::arg("ground_height"), py::arg("ground_height_margin"), py::arg("rotation_radius"),
             py::arg("initial_transform_file"))
        .def_readonly("in_factory", &GeneralConf::in_factory)
        .def_readonly("operation_mode", &GeneralConf::operation_mode)
        .def_readonly("has_external_guard", &GeneralConf::has_external_guard)
        .def_readonly("external_guard_offset", &GeneralConf::external_guard_offset)
        .def_readonly("ground_height", &GeneralConf::ground_height)
        .def_readonly("ground_height_margin", &GeneralConf::ground_height_margin)
        .def_readonly("rotation_radius", &GeneralConf::rotation_radius)
        .def_readonly("initial_transform_file", &GeneralConf::initial_transform_file);

    py::class_<UI_interface>(m_ui_interface, "UI_interface")
        .def(py::init<UIIFConf, int, double, int, bool, double, const std::string&, LoggerFunc>(), py::arg("ui_if"),
             py::arg("s_frame"), py::arg("rotation_radius"), py::arg("camera_num"), py::arg("has_external_guard"),
             py::arg("external_guard_offset"), py::arg("status_mmap_path"), py::arg("logfunc"))
        .def("set_cliff_info_by_octreee", &UI_interface::set_cliff_info_by_octreee, py::arg("octotree_obj"))
        .def("preprocess_info", &UI_interface::preprocess_info)
           .def("error_info", &UI_interface::error_info, py::arg("isslow"))
        .def("octotree_info", &UI_interface::octotree_info, py::arg("octotree_obj"))
        .def("machine_info", &UI_interface::machine_info, py::arg("angle_deg"))
        .def("collision_info", &UI_interface::collision_info)
        .def("zero_padding", &UI_interface::zero_padding, py::arg("n_byte"))
        .def("cliff_info", &UI_interface::cliff_info)
        .def("postprocess_info", &UI_interface::postprocess_info, py::arg("ret_t"), py::arg("process_time_ms"))
        .def("damp_info", &UI_interface::damp_info)
        .def("close_mmap", &UI_interface::close_mmap);
}
