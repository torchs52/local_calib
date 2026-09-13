#include "machine_collision_module.h"
#include "octotree/CircumcenterProcessor.h"
#include "octotree/MachineCollisionBase.h"
#include "octotree/MachineCollisionImmobileCuboid.h"
#include "octotree/MachineCollisionImmobileRoundCuboid.h"
#include "octotree/MachineCollisionImmobileHexaPrism.h"
#include "octotree/MachineCollisionMobileCuboid.h"
#include "octotree/machine_collision.h"

#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <nlohmann/json.hpp>

using arrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;
using json = nlohmann::json;

// 抽象クラスをpybind11で使用するために必要
class PyMachineCollisionBase : public MachineCollisionBase
{
  public:
    /* Inherit the constructors */
    using MachineCollisionBase::MachineCollisionBase;
};

class PyBaseRange : public machine_rm::BaseRange
{
  public:
    using machine_rm::BaseRange::BaseRange;
};

void bind_machine_collision(py::module& m)
{
    py::module m_MachineCollision = m.def_submodule("machine_collision");

    // CircumcenterProcessor
    py::class_<CircumcenterProcessor>(m_MachineCollision, "CircumcenterProcessor")
        .def_readwrite("center", &CircumcenterProcessor::center)
        .def_readwrite("radius", &CircumcenterProcessor::radius)
        .def_readwrite("points", &CircumcenterProcessor::points)
        .def(py::init<>())
        .def("fit", &CircumcenterProcessor::fit, py::arg("points"))
        .def("score", &CircumcenterProcessor::score, py::arg("points"), py::arg("center"), py::arg("radius"));

    // MachineCollisionBase
    py::class_<MachineCollisionBase, PyMachineCollisionBase>(m_MachineCollision, "MachineCollisionBase")
        .def_readwrite("machine_pcd_points", &MachineCollisionBase::machine_pcd_points)
        .def_readwrite("machine_form_points", &MachineCollisionBase::machine_form_points)
        .def_readwrite("machine_info", &MachineCollisionBase::machine_info)
        .def_readwrite("pcd_points_file", &MachineCollisionBase::pcd_points_file)
        .def_readwrite("offsets_initial", &MachineCollisionBase::offsets_initial)
        .def_readwrite("reverse_initial", &MachineCollisionBase::reverse_initial)
        .def(py::init<MachineConf, std::string, std::optional<std::tuple<double, double, double>>,
                      std::optional<std::tuple<bool, bool, bool>>, std::string>(),
             py::arg("machine_info"), py::arg("filename"), py::arg("initial_offsets") = py::none(),
             py::arg("reverse") = py::none(), py::arg("machine_form_extension") = ".csv")
        .def("check_pcd_on_self", &MachineCollisionBase::check_pcd_on_self, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0,
             py::arg("pitch_angle") = 0, py::arg("yaw_angle") = 0, py::arg("transform_mat") = 0)
        .def("__str__", &MachineCollisionBase::to_string);

    // MachineCollisionImmobileCuboid
    py::class_<MachineCollisionImmobileCuboid, MachineCollisionBase>(m_MachineCollision,
                                                                     "MachineCollisionImmobileCuboid")
        .def(py::init<MachineConf, std::string, std::optional<std::tuple<double, double, double>>,
                      std::optional<std::tuple<bool, bool, bool>>>(),
             py::arg("machine_info"), py::arg("filename"), py::arg("initial_offsets") = py::none(),
             py::arg("reverse") = py::none())
        .def("check_pcd_on_self", &MachineCollisionImmobileCuboid::check_pcd_on_self, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0,
             py::arg("pitch_angle") = 0, py::arg("yaw_angle") = 0, py::arg("transform_mat") = py::none());

    // MachineCollisionImmobileRoundCuboid
    py::class_<MachineCollisionImmobileRoundCuboid, MachineCollisionBase>(m_MachineCollision,
                                                                          "MachineCollisionImmobileRoundCuboid")
        .def(py::init<MachineConf, std::string, std::optional<std::tuple<double, double, double>>,
                      std::optional<std::tuple<bool, bool, bool>>>(),
             py::arg("machine_info"), py::arg("filename"), py::arg("initial_offsets") = py::none(),
             py::arg("reverse") = py::none())
        .def("check_pcd_on_self", &MachineCollisionImmobileRoundCuboid::check_pcd_on_self, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0,
             py::arg("pitch_angle") = 0, py::arg("yaw_angle") = 0, py::arg("transform_mat") = py::none());

    // MachineCollisionMobileCuboid
    py::class_<MachineCollisionMobileCuboid, MachineCollisionBase>(m_MachineCollision, "MachineCollisionMobileCuboid")
        .def(py::init<MachineConf, std::string, std::optional<std::tuple<double, double, double>>,
                      std::optional<std::tuple<bool, bool, bool>>>(),
             py::arg("machine_info"), py::arg("filename"), py::arg("initial_offsets") = py::none(),
             py::arg("reverse") = py::none())
        .def("check_pcd_on_self", &MachineCollisionMobileCuboid::check_pcd_on_self, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0,
             py::arg("pitch_angle") = 0, py::arg("yaw_angle") = 0, py::arg("transform_mat") = py::none());

    // MachineCollisionImmobileHexaPrism
    py::class_<machine_rm::MachineCollisionImmobileHexaPrism, MachineCollisionBase>(m_MachineCollision,
                                                                                    "MachineCollisionImmobileHexaPrism")
        .def_property_readonly("hex_base_ranges",
                               [](const machine_rm::MachineCollisionImmobileHexaPrism& self)
                               {
                                   py::list pylist;
                                   for (auto& ptr : self.hex_base_ranges)
                                   {
                                       pylist.append(ptr);
                                   }
                                   return pylist;
                               })
        .def(py::init<MachineConf, std::string, std::optional<std::tuple<double, double, double>>,
                      std::optional<std::tuple<bool, bool, bool>>, std::string>(),
             py::arg("machine_info"), py::arg("filename"), py::arg("initial_offsets") = py::none(),
             py::arg("reverse") = py::none(), py::arg("machine_form_extension") = ".jsonc")
        .def("check_pcd_on_self", &machine_rm::MachineCollisionImmobileHexaPrism::check_pcd_on_self, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.11, 0.11, 0.11), py::arg("roll_angle") = 0,
             py::arg("pitch_angle") = 0, py::arg("yaw_angle") = 0, py::arg("transform_mat") = py::none());
    //.def_readwrite("hex_base_ranges", &machine_rm::MachineCollisionImmobileHexaPrism::hex_base_ranges
    //)

    // BaseRange
    py::class_<machine_rm::BaseRange, PyBaseRange, std::shared_ptr<machine_rm::BaseRange>>(m_MachineCollision,
                                                                                           "BaseRange")
        .def("check_pcd", &machine_rm::BaseRange::check_pcd, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.12, 0.12, 0.04));

    // CuboidRange
    py::class_<machine_rm::CuboidRange, machine_rm::BaseRange, std::shared_ptr<machine_rm::CuboidRange>>(
        m_MachineCollision, "CuboidRange")
        .def(py::init<std::tuple<double, double>, std::tuple<double, double>, std::tuple<double, double>,
                      std::tuple<double, double>, std::tuple<double, double>, std::tuple<double, double>>(),
             py::arg("x_min_max"), py::arg("y_min_max"), py::arg("z_min_max"),
             py::arg("x_range_ratio") = std::make_tuple(-1.0, 1.0),
             py::arg("y_range_ratio") = std::make_tuple(-1.0, 1.0),
             py::arg("z_range_ratio") = std::make_tuple(-1.0, 1.0))
        .def_readwrite("x_minmax", &machine_rm::CuboidRange::x_minmax)
        .def_readwrite("y_minmax", &machine_rm::CuboidRange::y_minmax)
        .def_readwrite("z_minmax", &machine_rm::CuboidRange::z_minmax)
        .def_readwrite("x_range_ratio", &machine_rm::CuboidRange::x_range_ratio)
        .def_readwrite("y_range_ratio", &machine_rm::CuboidRange::y_range_ratio)
        .def_readwrite("z_range_ratio", &machine_rm::CuboidRange::z_range_ratio)
        .def("__repr__",
             [](const machine_rm::CuboidRange& self)
             {
                 std::ostringstream oss;
                 oss << "CuboidRange("
                     << "x_minmax = (" << std::get<0>(self.x_minmax) << ", " << std::get<1>(self.x_minmax) << "), "
                     << "y_minmax = (" << std::get<0>(self.y_minmax) << ", " << std::get<1>(self.y_minmax) << "), "
                     << "z_minmax = (" << std::get<0>(self.z_minmax) << ", " << std::get<1>(self.z_minmax) << "), "
                     << "x_range_ratio = (" << std::get<0>(self.x_range_ratio) << ", "
                     << std::get<1>(self.x_range_ratio) << "), "
                     << "y_range_ratio =" << std::get<0>(self.y_range_ratio) << ", " << std::get<1>(self.y_range_ratio)
                     << "), "
                     << "z_range_ratio =" << std::get<0>(self.z_range_ratio) << ", " << std::get<1>(self.z_range_ratio)
                     << ")";
                 return oss.str();
             })
        .def("check_pcd", &machine_rm::CuboidRange::check_pcd, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.12, 0.12, 0.04));

    // TriPillar
    py::class_<machine_rm::TriPillar, machine_rm::BaseRange, std::shared_ptr<machine_rm::TriPillar>>(m_MachineCollision,
                                                                                                     "TriPillar")
        .def(py::init<std::tuple<double, double>, std::vector<std::tuple<double, double>>, std::tuple<double, double>,
                      std::tuple<double, double>, std::tuple<double, double>, bool>(),
             py::arg("z_minmax"), py::arg("tri_points"), py::arg("x_remove_offset_ratio"),
             py::arg("y_remove_offset_ratio"), py::arg("z_remove_offset_ratio"), py::arg("vec_is_reverse") = false)
        .def_readwrite("z_minmax", &machine_rm::TriPillar::z_minmax)
        .def_readwrite("tri_points", &machine_rm::TriPillar::tri_points)
        .def_readwrite("x_remove_offset_ratio", &machine_rm::TriPillar::x_remove_offset_ratio)
        .def_readwrite("y_remove_offset_ratio", &machine_rm::TriPillar::y_remove_offset_ratio)
        .def_readwrite("z_remove_range_ratio", &machine_rm::TriPillar::z_remove_offset_ratio)
        .def_readwrite("vec_is_reverse", &machine_rm::TriPillar::vec_is_reverse)
        .def("__repr__",
             [](const machine_rm::TriPillar& self)
             {
                 std::ostringstream oss;
                 std::string bool_string = self.vec_is_reverse ? "true" : "false";
                 oss << "TriPillar("
                     << "z_minmax= (" << std::get<0>(self.z_minmax) << "," << std::get<1>(self.z_minmax) << "), "
                     << "tri_points = [";
                 for (auto& tri_point : self.tri_points)
                 {
                     oss << "(" << std::get<0>(tri_point) << ", " << std::get<1>(tri_point) << "), ";
                 }
                 oss << "], ";
                 oss << "x_remove_offset_ratio = (" << std::get<0>(self.x_remove_offset_ratio) << ", "
                     << std::get<1>(self.x_remove_offset_ratio) << "), "
                     << "y_remove_offset_ratio = (" << std::get<0>(self.y_remove_offset_ratio) << ", "
                     << std::get<1>(self.y_remove_offset_ratio) << "), "
                     << "z_remove_offset_ratio = (" << std::get<0>(self.z_remove_offset_ratio) << ", "
                     << std::get<1>(self.z_remove_offset_ratio) << "), "
                     << "vec_is_reverse = " << bool_string << ")";
                 return oss.str();
             })
        .def("check_pcd", &machine_rm::TriPillar::check_pcd, py::arg("xyz"),
             py::arg("remove_dist") = std::make_tuple(0.12, 0.12, 0.04));

    // "MachineConf
    py::class_<MachineConf>(m_MachineCollision, "MachineConf")
        .def_readwrite("pcd_points_file", &MachineConf::pcd_points_file)
        .def_readwrite("load_order", &MachineConf::load_order)
        .def_readwrite("instance_name", &MachineConf::instance_name)
        .def_readwrite("offsets", &MachineConf::offsets)
        .def_readwrite("reverse", &MachineConf::reverse)
        .def_readwrite("is_mobile", &MachineConf::is_mobile)
        .def_readwrite("form_points_pattern", &MachineConf::form_points_pattern)
        .def(py::init<std::string, int, std::string, std::tuple<double, double, double>, std::tuple<bool, bool, bool>,
                      bool, std::string>(),
             py::arg("pcd_points_file"), py::arg("load_order"), py::arg("instance_name"),
             py::arg("offsets") = std::make_tuple(0, 0, 0), py::arg("reverse") = std::make_tuple(false, true, false),
             py::arg("is_mobile") = false, py::arg("form_points_pattern") = "cuboid_points")
        .def("get_form_points_filename", &MachineConf::get_form_points_filename, py::arg("filename"),
             py::arg("extension") = ".csv");
    m_MachineCollision.def("create_machine_collision_list", &create_machine_collision_list, py::arg("file_dir"),
                           py::arg("initial_offsets"), py::arg("l_col_machine_conf"),
                           py::arg("reverse") = py::make_tuple(true, true, false));

    m_MachineCollision.def("create_base_range_from_json",
                           [](const std::string& json_str)
                           {
                               json entry = json::parse(json_str);
                               return std::shared_ptr<machine_rm::BaseRange>(create_base_range_from_json(entry));
                           });

    m_MachineCollision.def("load_base_ranges_from_json", &load_base_ranges_from_json, py::arg("json_path"));
}
