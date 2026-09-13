#include "ui_interface/vis_octotree.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoTree.h"
#include <Eigen/Core>
#include <memory>
// #include <open3d/Open3D.h>
#include <algorithm>
#include <optional>
#include <tuple>
#include <vector>

// using namespace open3d;

/**trans_vecを起点として、xyz方向にscaleの長さを持たせたboxをcolor_listの色で生成する関数
 * + 入力:
 *      1. color_list: boxの色, Noneの場合黒色になる
 *      2. trans_vec: 原点からどれだけ並進させるか,
 *         Noneの場合原点を起点にboxを作る
 *      3. scale: 各辺xyz方向にどれだけ伸ばすか, Noneの場合長さ1のboxを作る
 * + 出力:
 *      設定値に基づいたLineSet
 **/
// std::shared_ptr<open3d::geometry::LineSet> create_unit_bbox_by_lineset(
//     const std::optional<std::tuple<double, double, double>> &color_list,
//     std::optional<Eigen::MatrixXd> trans_vec,
//     std::optional<Eigen::MatrixXd> scale) {

//   Eigen::MatrixXd unit_points(8, 3);
//   unit_points << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1,
//   1,
//       0, 1, 1;
//   if (scale.has_value()) {
//     unit_points = scale.value().array() * unit_points.array();
//   }
//   if (trans_vec.has_value()) {
//     unit_points = unit_points.array() + trans_vec.value().array();
//   }
//   std::vector<Eigen::Vector3d> points;

//   points.reserve(8);
//   for (int i = 0; i < unit_points.rows(); i++) {
//     points.push_back(unit_points.row(i));
//   }
//   auto unit_lineset = std::make_shared<geometry::LineSet>();
//   unit_lineset->points_ = points;
//   unit_lineset->lines_ = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5},
//                           {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};
//   if (color_list.has_value()) {
//     Eigen::Vector3d eigen_color_list = {std::get<0>(color_list.value()),
//                                         std::get<1>(color_list.value()),
//                                         std::get<2>(color_list.value())};
//     unit_lineset->PaintUniformColor(eigen_color_list);
//   } else {
//     unit_lineset->PaintUniformColor({0, 0, 0});
//   };
//   return unit_lineset;
// }

/***
 * 離散座標vox_coordsを幅がw_min_rangeからw_max_rangeの中で実座標に変換する
 * 変換に際して、offsetが必要な場合はoffsetを行う
 * いくつかの箇所で同じ処理をしているので関数化
 */
std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
_vox2w_coords(const Eigen::Ref<const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>>& vox_coords,
              int tree_depth, const Eigen::Ref<const Eigen::VectorXd>& w_max_range,
              const Eigen::Ref<const Eigen::VectorXd>& w_min_range, std::tuple<double, double, double> offset)
{
    Eigen::VectorXd cell_size = (w_max_range - w_min_range) / (pow(2.0, tree_depth));

    if (vox_coords.size() > 0)
    {
        Eigen::Vector3d eigen_offset(3);
        eigen_offset << std::get<0>(offset), std::get<1>(offset), std::get<2>(offset);
        return {(((vox_coords.cast<double>().rowwise() + eigen_offset.transpose()).array().rowwise() *
                  cell_size.transpose().array())
                     .rowwise() +
                 w_min_range.transpose().array())
                    .matrix(),
                cell_size};
    }
    return {Eigen::MatrixXd::Zero(0, 0), cell_size};
}

/**
 * sourceとdestの共通部分とそれ以外を分ける計算
 */
std::tuple<Eigen::Matrix<int64_t, Eigen::Dynamic, 3>, Eigen::Matrix<int64_t, Eigen::Dynamic, 3>,
           Eigen::Matrix<int64_t, Eigen::Dynamic, 3>>
_split_intersection(const Eigen::Ref<const Eigen::MatrixXi>& np_source,
                    const Eigen::Ref<const Eigen::MatrixXi>& np_dest)
{
    // 重なりを計算
    std::set<std::tuple<int, int, int>> source_set;
    for (int i = 0; i < np_source.rows(); i++)
    {
        source_set.insert({np_source(i, 0), np_source(i, 1), np_source(i, 2)});
    }
    std::set<std::tuple<int, int, int>> dest_set;
    for (int i = 0; i < np_dest.rows(); i++)
    {
        dest_set.insert({np_dest(i, 0), np_dest(i, 1), np_dest(i, 2)});
    }
    std::set<std::tuple<int, int, int>> intersection_set;
    std::set_intersection(source_set.begin(), source_set.end(), dest_set.begin(), dest_set.end(),
                          std::inserter(intersection_set, intersection_set.begin()));

    // 機体と被っている点群
    std::set<std::tuple<int, int, int>> source_only_set;
    std::set_difference(source_set.begin(), source_set.end(), intersection_set.begin(), intersection_set.end(),
                        std::inserter(source_only_set, source_only_set.begin()));
    std::set<std::tuple<int, int, int>> dest_only_set;
    std::set_difference(dest_set.begin(), dest_set.end(), intersection_set.begin(), intersection_set.end(),
                        std::inserter(dest_only_set, dest_only_set.begin()));

    Eigen::Matrix<int64_t, Eigen::Dynamic, 3> np_intersection(intersection_set.size(), 3);
    Eigen::Matrix<int64_t, Eigen::Dynamic, 3> np_source_only(source_only_set.size(), 3);
    Eigen::Matrix<int64_t, Eigen::Dynamic, 3> np_dest_only(dest_only_set.size(), 3);

    int i = 0;
    for (const auto& elem : intersection_set)
    {
        np_intersection(i, 0) = static_cast<int64_t>(std::get<0>(elem));
        np_intersection(i, 1) = static_cast<int64_t>(std::get<1>(elem));
        np_intersection(i, 2) = static_cast<int64_t>(std::get<2>(elem));
        i++;
    }
    i = 0;
    for (const auto& elem : source_only_set)
    {
        np_source_only(i, 0) = static_cast<int64_t>(std::get<0>(elem));
        np_source_only(i, 1) = static_cast<int64_t>(std::get<1>(elem));
        np_source_only(i, 2) = static_cast<int64_t>(std::get<2>(elem));
        i++;
    }
    i = 0;
    for (const auto& elem : dest_only_set)
    {
        np_dest_only(i, 0) = static_cast<int64_t>(std::get<0>(elem));
        np_dest_only(i, 1) = static_cast<int64_t>(std::get<1>(elem));
        np_dest_only(i, 2) = static_cast<int64_t>(std::get<2>(elem));
        i++;
    }
    return {np_intersection, np_source_only, np_dest_only};
}

/**
 * 接触可能性探索のLiDAR点群が占める部分, 機体点群が占める部分,
 * 接触部分が占める部分の 中心座標を計算する
 */
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> create_voxmed_existing_cell_by_entity(
    int vis_tree_depth, OctoTree octotree_obj, const Eigen::Ref<const Eigen::VectorXd>& w_max_range,
    const Eigen::Ref<const Eigen::VectorXd>& w_min_range, const std::vector<NodeEntity>& src_entities,
    const std::vector<NodeEntity>& dest_entities)
{
    Eigen::MatrixXi vox_points_src = octotree_obj.get_vox_from_entity_octonodes_by_chunk(src_entities, vis_tree_depth);

    Eigen::MatrixXi vox_points_dest =
        octotree_obj.get_vox_from_entity_octonodes_by_chunk(dest_entities, vis_tree_depth);

    const auto& [vox_points_intersection, vox_points_machine_only, vox_points_pcd_only] =
        _split_intersection(vox_points_src, vox_points_dest);

    // 離散座標から中心座標を計算
    Eigen::MatrixXd intersection_result;
    std::tie(intersection_result, std::ignore) =
        _vox2w_coords(vox_points_intersection, vis_tree_depth, w_max_range, w_min_range, {0.5, 0.5, 0.5});
    Eigen::MatrixXd machine_only_result;
    std::tie(machine_only_result, std::ignore) =
        _vox2w_coords(vox_points_machine_only, vis_tree_depth, w_max_range, w_min_range, {0.5, 0.5, 0.5});
    Eigen::MatrixXd pcd_only_result;
    std::tie(pcd_only_result, std::ignore) =
        _vox2w_coords(vox_points_pcd_only, vis_tree_depth, w_max_range, w_min_range, {0.5, 0.5, 0.5});
    return {intersection_result, machine_only_result, pcd_only_result};
}