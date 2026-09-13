#include "octotree_module.h"
#include "octotree/OctoNode.h"
#include "octotree/OctoTree.h"
#include "octotree/VoxStats.h"
#include "bind_opaque.h"

#include <memory>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

void bind_octotree(py::module& m)
{
    py::module m_octotree = m.def_submodule("octotree");

    py::class_<Point>(m_octotree, "Point")
        .def(py::init())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y)
        .def_readwrite("z", &Point::z)
        .def_readwrite("is_null", &Point::is_null);

    py::class_<VoxStats>(m_octotree, "VoxStats")
        .def(py::init())
        .def_readwrite("first_moment", &VoxStats::first_moment)
        .def_readwrite("second_moment", &VoxStats::second_moment)
        .def_readwrite("counts", &VoxStats::counts)
        .def_readwrite("far_point", &VoxStats::far_point)
        .def_readwrite("near_point", &VoxStats::near_point)
        .def_readwrite("quantile", &VoxStats::quantile)
        .def_readwrite("far_dist", &VoxStats::far_dist)
        .def_readwrite("near_dist", &VoxStats::near_dist);

    py::enum_<NodeEntity>(m_octotree, "NodeEntity")
        .value("UNK", NodeEntity::UNK)
        .value("HUMAN", NodeEntity::HUMAN)
        .value("CRANE", NodeEntity::CRANE)
        .value("CRANE_MOBILE", NodeEntity::CRANE_MOBILE)
        .value("CRANE_IMMOBILE", NodeEntity::CRANE_IMMOBILE)
        .value("CRANE_MOBILE_FOR_DET", NodeEntity::CRANE_MOBILE_FOR_DET)
        .value("CRANE_IMMOBILE_FOR_DET", NodeEntity::CRANE_IMMOBILE_FOR_DET)
        .value("CRANE_EXTERNAL_GUARD", NodeEntity::CRANE_EXTERNAL_GUARD)
        .value("CLIFF", NodeEntity::CLIFF)
        .value("HIGH_3D", NodeEntity::HIGH_3D)
        .value("LOW_3D", NodeEntity::LOW_3D)
        .value("OTHER", NodeEntity::OTHER)
        .export_values();

    py::class_<NodeClusterKey>(m_octotree, "NodeClusterKey")
        .def(py::init<NodeEntity, std::optional<int>>(), py::arg("entity"), py::arg("cluster_id") = py::none())
        .def_readwrite("entity", &NodeClusterKey::entity)
        .def_readwrite("cluster_id", &NodeClusterKey::cluster_id)
        .def("__eq__", &NodeClusterKey::operator==)
        .def("__hash__",
             [](const NodeClusterKey& k)
             {
                 size_t h1 = std::hash<int>{}(static_cast<int>(k.entity));
                 size_t h2 = k.cluster_id ? std::hash<int>{}(*k.cluster_id) : 0;
                 return h1 ^ (h2 << 1);
             })
        .def("__repr__",
             [](const NodeClusterKey& k)
             {
                 if (k.cluster_id)
                     return "<NodeClusterKey entity=" + std::to_string((int)k.entity) +
                            ", cluster=" + std::to_string(*k.cluster_id) + ">";
                 else
                     return "<NodeClusterKey entity=" + std::to_string((int)k.entity) + ", cluster=None>";
             });

    py::class_<OctoNode>(m_octotree, "OctoNode")
        .def(py::init<std::tuple<int, int, int>, NodeEntity, std::shared_ptr<VoxStats>>(), py::arg("vox_coord"),
             py::arg("entity") = NodeEntity::UNK, py::arg("node_stats") = py::none())
        .def_readwrite("entity", &OctoNode::entity)
        .def_readwrite("morton_code", &OctoNode::morton_code)
        .def_readwrite("node_stats", &OctoNode::node_stats)
        .def_readwrite("cluster_label", &OctoNode::cluster_label)
        .def_static("morton_encode_3d", &OctoNode::morton_encode_3d, py::arg("x"), py::arg("y"), py::arg("z"))
        .def_static("morton_decode_3d", &OctoNode::morton_decode_3d, py::arg("morton_code"))
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def("__str__", &OctoNode::to_string)
        .def("get_cluster_label", &OctoNode::get_cluster_label)
        .def("set_cluster_label", &OctoNode::set_cluster_label)
        .def("free_vox_stats", &OctoNode::free_vox_stats)
        .def("get_mean", &OctoNode::get_mean)
        .def("get_far_point", &OctoNode::get_far_point)
        .def("get_near_point", &OctoNode::get_near_point)
        .def("get_quantile", &OctoNode::get_quantile);

    py::bind_map<std::map<std::variant<int, std::string>, std::map<std::tuple<int, int, int>, OctoNode>>>(
        m_octotree, "LabeledOctoNodes", py::module_local(false));

    py::class_<OctoTree>(m_octotree, "OctoTree")
        .def(py::init<std::optional<Eigen::MatrixXd>, Eigen::Vector3d, Eigen::Vector3d, double, NodeEntity, bool,
                      std::optional<float>, std::optional<Eigen::Vector3d>>(),
             py::arg("xyz") = py::none(), py::arg("max_xyz") = Eigen::Vector3d(18, 20, 20),
             py::arg("min_xyz") = Eigen::Vector3d(-12, -10, -10), py::arg("max_tree_depth") = 7,
             py::arg("xyz_entity") = NodeEntity::UNK, py::arg("use_node_stats") = false,
             py::arg("quantile") = py::none(), py::arg("origin_w2oct") = py::none())
        .def_readwrite("max_xyz", &OctoTree::max_xyz)
        .def_readwrite("min_xyz", &OctoTree::min_xyz)
        .def_readwrite("max_tree_depth", &OctoTree::max_tree_depth)
        .def_readwrite("cell_interval", &OctoTree::cell_interval)
        .def_readwrite("use_node_stats", &OctoTree::use_node_stats)
        .def_readwrite("quantile", &OctoTree::quantile)
        .def_readwrite("labeled_octo_nodes", &OctoTree::labeled_octo_nodes)
        .def_readwrite("unlabeled_octo_nodes", &OctoTree::unlabeled_octo_nodes)
        .def_readwrite("cluster_vox2deepest_vox", &OctoTree::cluster_vox2deepest_vox)
        .def_readwrite("origin_w2oct", &OctoTree::origin_w2oct)
        .def_readwrite("entity_octonodes", &OctoTree::entity_octonodes)
        .def_readonly_static("MACHINE_DETECT_LABEL", &OctoTree::MACHINE_DETECT_LABEL)
        .def_readonly_static("MACHINE_MEASURE_LABEL", &OctoTree::MACHINE_MEASURE_LABEL)
        .def("create_octonodes", &OctoTree::create_octonodes, py::arg("xyz"), py::arg("entity") = NodeEntity::UNK,
             py::arg("removed_vox_min_points") = py::none(), py::arg("removed_vox_max_points") = py::none(),
             py::arg("remove_dist") = 1)
        .def("insert_or_create_octonodes", &OctoTree::insert_or_create_octonodes, py::arg("xyz"),
             py::arg("entity") = NodeEntity::UNK, py::arg("removed_vox_min_points") = py::none(),
             py::arg("removed_vox_max_points") = py::none(), py::arg("remove_dist") = 1)
        .def_static("get_cuboid_boundary", &OctoTree::get_cuboid_boundary, py::arg("min_x"), py::arg("min_y"),
                    py::arg("min_z"), py::arg("max_x"), py::arg("max_y"), py::arg("max_z"), py::arg("step") = 2)
        .def("vox2oct_coords", &OctoTree::vox2oct_coords, py::arg("vox_coords"), py::arg("tree_depth") = py::none())
        .def("w2vox_coords", &OctoTree::w2vox_coords, py::arg("xyz"), py::arg("tree_depth") = py::none(),
             py::arg("remove_dep") = true)
        .def("oct2vox_coords", &OctoTree::oct2vox_coords, py::arg("oct_coords"), py::arg("tree_depth") = py::none(),
             py::arg("remove_dep") = true)
        .def("vox2w_coords", &OctoTree::vox2w_coords, py::arg("vox_coords"), py::arg("tree_depth") = py::none())
        .def("get_octonodes_vox_coord_unlabled", &OctoTree::get_octonodes_vox_coord_unlabled,
             py::arg("tree_depth") = py::none())
        .def("get_octonodes_oct_coord_unlabeled", &OctoTree::get_octonodes_oct_coord_unlabeled,
             py::arg("tree_depth") = py::none())
        .def("get_octonodes_np_coord_unlabeled", &OctoTree::get_octonodes_np_coord_unlabeled,
             py::arg("tree_depth") = py::none())
        .def("get_octonodes_np_coord_labeled_v2", &OctoTree::get_octonodes_np_coord_labeled_v2,
             py::arg("target_labels") = py::none(), py::arg("tree_depth") = py::none())
        .def("get_octonodes_np_coord_labeled", &OctoTree::get_octonodes_np_coord_labeled,
             py::arg("target_labels") = py::none(), py::arg("tree_depth") = py::none())
        .def("get_octonodes_np_coord_entity", &OctoTree::get_octonodes_np_coord_entity, py::arg("target_entities"),
             py::arg("tree_depth") = py::none())
        .def("w2oct_coords", &OctoTree::w2oct_coords, py::arg("xyz"))
        .def("oct2w_coords", &OctoTree::oct2w_coords, py::arg("oct_coords"))
        .def("insert_entity_result", &OctoTree::insert_entity_result, py::arg("cluster2entity"))
        .def("insert_entity_result_v2", &OctoTree::insert_entity_result_v2, py::arg("cluster2entity"))
        .def("insert_clustering_result", &OctoTree::insert_clustering_result, py::arg("clustered_data"),
             py::arg("labels"))
        .def("insert_clustering_result_v2", &OctoTree::insert_clustering_result_v2, py::arg("clustered_data"),
             py::arg("labels"))
        .def("insert_or_entity_octonodes", &OctoTree::insert_or_entity_octonodes, py::arg("xyz"),
             py::arg("entity") = NodeEntity::OTHER, py::arg("entity_replace") = false, py::arg("is_order") = false,
             py::arg("removed_vox_min_points") = py::none(), py::arg("removed_vox_max_points") = py::none(),
             py::arg("remove_dist") = 1)
        .def("insert_or_entity_octonodes_with_labels", &OctoTree::insert_or_entity_octonodes_with_labels,
             py::arg("xyz"), py::arg("labels"), py::arg("entity") = NodeEntity::CLIFF,
             py::arg("entity_replace") = false, py::arg("is_order") = true,
             py::arg("removed_vox_min_points") = py::none(), py::arg("removed_vox_max_points") = py::none(),
             py::arg("remove_dist") = 1)
        .def("get_vox_from_entity_octonodes_by_chunk", &OctoTree::get_vox_from_entity_octonodes_by_chunk,
             py::arg("target_entities"), py::arg("tree_depth") = py::none())
        .def("get_np_from_entity_octonodes_by_chunk", &OctoTree::get_np_from_entity_octonodes_by_chunk,
             py::arg("target_entities"), py::arg("tree_depth") = py::none())
        .def("get_np_from_entity_octonodes_by_list", &OctoTree::get_np_from_entity_octonodes_by_list,
             py::arg("target_entities"), py::arg("tree_depth") = py::none())
        .def("get_clustering_data_by_entity", &OctoTree::get_clustering_data_by_entity, py::arg("cluster_entity"),
             py::arg("cluster_depth") = py::none())
        .def("erase_nodes_for_entities", &OctoTree::erase_nodes_for_entities, py::arg("entities"))
        .def("erase_nodes_for_entities_noret", &OctoTree::erase_nodes_for_entities_noret, py::arg("entities"))
        .def("insert_labeles_and_move_in_octonodes", &OctoTree::insert_labeles_and_move_in_octonodes,
             py::arg("clustered_data"), py::arg("labels"), py::arg("cluster_entity"))
        .def("replace_entities_in_octonodes", &OctoTree::replace_entities_in_octonodes, py::arg("from_entity"),
             py::arg("cluster_transfered_entity"))
        .def_static("limit_pcd_range", &OctoTree::limit_pcd_range, py::arg("pcd_data"), py::arg("min_data"),
                    py::arg("max_data"))
        .def("calc_vox_statistics", &OctoTree::calc_vox_statistics, py::arg("vox_xyz"), py::arg("w_xyz"))
        .def("calc_vox_quantile", &OctoTree::calc_vox_quantile, py::arg("vox_xyz"), py::arg("w_xyz"),
             py::arg("quantile"));
}
