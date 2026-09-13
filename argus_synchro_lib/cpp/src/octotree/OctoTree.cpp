#include "octotree/OctoTree.h"
#include "octotree/NodeEntity.h"
#include "cpp_helper_lib/eigen_operator.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>

// 衝突判定で用いる期待ラベルを表す定数
// 接触可能性探索と距離計算で点群を分ける必要が出てきたので、それぞれで別のラベルを用意
// entity_octonodesを使うことにしてからこれらは使っていない
const std::string OctoTree::MACHINE_DETECT_LABEL = "machine_detect";
const std::string OctoTree::MACHINE_MEASURE_LABEL = "machine_measure";

/**
 * @brief 八分木を表すクラス 八分木の各ノードを持っているが、衝突判定はこのクラスでは行わない
 * @param xyz : 前工程から与えられる点群の座標, インスタンス生成時に入れることもできる
 * @param max_xyz : 計算対象となる点群座標の最大値
 * @param min_xyz : 計算対象となる点群座標の最小値
 * @param max_treed_epth : 木の深さ
 * @param xyz_entity xyzのNodeEntityを表す
 * @param use_node_stats 八分木ノードの中の点群に対して統計量を計算するかどうか
 * @param quantile 統計量としてquantileを計算する場合の何パーセントのquantileを用いるかを表す
 * @param origin_w2oct 八分木原点, nullの場合はLiDAR座標と同じ
 */
OctoTree::OctoTree(const std::optional<Eigen::MatrixXd>& xyz, const Eigen::Ref<const Eigen::Vector3d>& max_xyz,
                   const Eigen::Ref<const Eigen::Vector3d>& min_xyz, double max_tree_depth, NodeEntity xyz_entity,
                   bool use_node_stas, std::optional<float> quantile,
                   const std::optional<Eigen::Vector3d>& origin_w2oct)
    : max_xyz(max_xyz), min_xyz(min_xyz), max_tree_depth(max_tree_depth), labeled_octo_nodes(std::nullopt),
      _clustering_tree_depth(std::nullopt), use_node_stats(use_node_stas), quantile(quantile),
      origin_w2oct(origin_w2oct), entity_octonodes(EntityMap())
{
    this->cell_interval = (this->max_xyz - this->min_xyz) / pow(2, max_tree_depth);

    if (xyz.has_value())
    {
        this->create_octonodes(xyz.value(), xyz_entity);
    }
    else
    {
        this->unlabeled_octo_nodes = std::nullopt;
    }
};

/**
 * @brief tree_depthがmax_tree_depthに対して、どれくらい上の階層か計算して、必要であれば例外を出す
 *
 * @param tree_depth 一番下の階層から何階層登るか
 * @return int 計算で用いる階層
 */
int OctoTree::_get_tree_diff(std::optional<int> tree_depth) const
{
    auto _tree_depth = (tree_depth != std::nullopt) ? tree_depth.value() : max_tree_depth;
    if (_tree_depth > this->max_tree_depth)
    {
        throw std::invalid_argument("tree_depth should be less than  " + std::to_string(max_tree_depth) + ".");
    }

    int diff_tree_depth = max_tree_depth - _tree_depth;
    return diff_tree_depth;
}

/**
 * @brief Create a octonodes object,
 *
 * @param xyz
 * @param entity
 * @param removed_vox_min_points
 * @param removed_vox_max_points
 * @param remove_dist
 * @return OctoTree&
 */
OctoTree&
OctoTree::create_octonodes(const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity,
                           const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                               removed_vox_min_points,
                           const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                               removed_vox_max_points,
                           int remove_dist)
{
    if (this->use_node_stats)
    {
        this->unlabeled_octo_nodes = this->_gen_octonodes_with_stats<OctoMap>(
            xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist, this->quantile);
    }
    else
    {
        this->unlabeled_octo_nodes = this->_gen_octonodes_without_stats<OctoMap>(xyz, entity, removed_vox_min_points,
                                                                                 removed_vox_max_points, remove_dist);
    }
    return *this;
};

/**
 * @brief unlabeled_octonodesが存在しなければ与えられたxyz座標を基にunlabeled_octonodesを生成,
 * 存在すれば、与えられたxyz座標から生成されるoctonodesと同じkeyを持つものは置き換えられる
 * unlabeled_octonodesのあるkeyに対するentityは一定で、新しい方を使う
 * @deprecated entity_octonodesに変えてから使っていない
 */
OctoTree& OctoTree::insert_or_create_octonodes(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist)
{
    OctoMap new_octonodes;
    if (this->use_node_stats)
    {
        new_octonodes = this->_gen_octonodes_with_stats<OctoMap>(xyz, entity, removed_vox_min_points,
                                                                 removed_vox_max_points, remove_dist, this->quantile);
    }
    else
    {
        new_octonodes = this->_gen_octonodes_without_stats<OctoMap>(xyz, entity, removed_vox_min_points,
                                                                    removed_vox_max_points, remove_dist);
    }

    /* unlabeled_octonodesがnullの場合は、new_octonodesを割り当てて、
    そうでない場合は、new_octonodesと同じkeyを持つunlabeled_octonodesのkeyをnew_octonodesの方で上書きする
  */
    if (!this->unlabeled_octo_nodes)
    {
        this->unlabeled_octo_nodes = std::move(new_octonodes);
    }
    else
    {
        for (auto& [key, value] : new_octonodes)
        {
            this->unlabeled_octo_nodes->insert_or_assign(key, std::move(value));
        }
    }
    return *this;
};

std::unordered_set<VoxelCoord, TupleHash> OctoTree::get_cuboid_boundary(int min_x, int min_y, int min_z, int max_x,
                                                                        int max_y, int max_z, int step)
{
    // Initialize a set to store the boundary coordinates
    std::unordered_set<VoxelCoord, TupleHash> boundary_coords;

    // 元のプログラムでは、リストを使って計算をしていたが、同じ要素を持っていた場合もsetに入れる段階でなくなるため、予めset型で定義
    std::unordered_set<int> x_set;
    for (int x = min_x; x < max_x + 1; x += step)
    {
        x_set.insert(x);
    }
    x_set.insert(max_x);

    std::unordered_set<int> y_set;
    for (int y = min_y; y < max_y + 1; y += step)
    {
        y_set.insert(y);
    }
    y_set.insert(max_y);

    std::unordered_set<int> z_set;
    for (int z = min_z + 1; z < max_z; z += step)
    {
        z_set.insert(z);
    }
    z_set.insert(max_z);

    // Iterate over the edges of the cuboid
    for (const auto& x : x_set)
    {
        for (const auto& y : y_set)
        {
            // Add the coordinates on the bottom and top faces
            boundary_coords.insert({x, y, min_z});
            boundary_coords.insert({x, y, max_z});
        }
    }
    for (const auto& x : x_set)
    {
        for (const auto& z : z_set)
        {
            // Add the coordinates on the front and back faces
            boundary_coords.insert({x, min_y, z});
            boundary_coords.insert({x, max_y, z});
        }
    }
    for (const auto& y : y_set)
    {
        for (const auto& z : z_set)
        {
            // Add the coordinates on the left and right faces
            boundary_coords.insert({min_x, y, z});
            boundary_coords.insert({max_x, y, z});
        }
    }
    return boundary_coords;
}

/**
 * @brief 八分木ノードを生成するために、範囲外の点群を除去して除去後の離散座標と実座標のペアを返すメソッド
 *
 * @param xyz 対象となる点群
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param removed_vox_max_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param remove_dist removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param remove_dup 同じ離散座標を持つ点を除外するかどうか
 * @return std::pair<Eigen::MatrixXi, Eigen::MatrixXd 離散座標と実座標
 */
std::pair<Eigen::MatrixXi, Eigen::MatrixXd> OctoTree::_gen_vox_for_octonodes(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist, bool remove_dup)
{
    // 八分木座標に変換する
    Eigen::MatrixXd oct_coords = this->w2oct_coords(xyz);

    // 八分木座標を絞る
    Eigen::VectorXi limitted_ind = this->limit_pcd_range(oct_coords, this->min_xyz, this->max_xyz);

    Eigen::MatrixXd limitted_oct = oct_coords(limitted_ind, Eigen::all);

    // Note:
    // 統計量の計算では、リストの各要素の実座標が欲しいので重複削除は行わない
    Eigen::MatrixXi vox_coords = this->oct2vox_coords(limitted_oct, std::nullopt, remove_dup);

    // removed_vox_pointsに近い点を除去する
    if (removed_vox_min_points != std::nullopt && removed_vox_max_points != std::nullopt)
    {
        int removed_vox_points_size = removed_vox_min_points.value().size();
        int vox_coords_size = vox_coords.rows();
        Eigen::MatrixXi tmp_remove_ind(removed_vox_points_size, vox_coords_size);
        for (int i = 0; i < removed_vox_points_size; i++)
        {
            auto vox_min_points_parts = removed_vox_min_points.value().at(i);
            auto vox_max_points_parts = removed_vox_max_points.value().at(i);
            Eigen::VectorXd vox_coords_x = vox_coords(Eigen::all, 0).cast<double>();
            Eigen::VectorXd vox_coords_y = vox_coords(Eigen::all, 1).cast<double>();
            Eigen::VectorXd vox_coords_z = vox_coords(Eigen::all, 2).cast<double>();
            arrayXb row = ((vox_coords_x.array() >= vox_min_points_parts(0) - remove_dist &&
                            vox_coords_x.array() <= vox_max_points_parts(0) + remove_dist) &&
                           (vox_coords_y.array() >= vox_min_points_parts(1) - remove_dist &&
                            vox_coords_y.array() <= vox_max_points_parts(1) + remove_dist) &&
                           (vox_coords_z.array() >= vox_min_points_parts(2) - remove_dist &&
                            vox_coords_z.array() <= vox_max_points_parts(2) + remove_dist));

            for (int j = 0; j < vox_coords_size; j++)
            {
                tmp_remove_ind(i, j) = row(j);
            }
        }
        Eigen::VectorXi remove_ind = tmp_remove_ind.colwise().any();
        Eigen::VectorXi not_remove_ind = (remove_ind.array() != 1).cast<int>().matrix();

        not_remove_ind = helper::nonzero(not_remove_ind);

        // 不要なindexを除去する
        vox_coords = vox_coords(not_remove_ind, Eigen::all);
        limitted_oct = limitted_oct(not_remove_ind, Eigen::all);
    }

    return std::make_pair(vox_coords, limitted_oct);
}

/**
 * @brief 点群を八分木に入れるための形式に変換するメソッド
 * @details 八分木ノードを作る過程で、各ノードの実座標における統計量も計算する
 * @remarks
 * 引数で統計量の計算の有無を切り替えられるようにしていたが、統計量の計算をしない場合の処理速度が遅くなったので、呼ぶ関数を切り替えることで処理を行うようにした,
 * @remarks
 * 統計量の計算をしないほうが早いので、統計量を使って後続処理をしない場合は、_gen_octonodes_without_statsを読んだ方が良い
 * @remarks
 * genericsを使っているのは、ノードをmapで保持する場合と、queueで保持する場合があって、入れ物が違う以外は同じ処理になるため
 *
 * @param xyz 八分木に入れる点群
 * @param entity xyzに紐づけるNodeEntity
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param quantile 統計量としてquantileを計算する場合の設定する, nullだと計算いない
 * @return Container OctoMapやOctoQueueを想定したもの, 離散座標と八分木のノードの組のコレクション
 */
template <typename Container>
Container OctoTree::_gen_octonodes_with_stats(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist, std::optional<float> quantile)
{
    // 必要な範囲に点群を絞る
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> res =
        this->_gen_vox_for_octonodes(xyz, removed_vox_min_points, removed_vox_max_points, remove_dist, false);
    Eigen::MatrixXi vox_coords = res.first;
    Eigen::MatrixXd limitted_xyz = res.second;

    // 統計量の計算
    auto vox_stats = this->calc_vox_statistics(vox_coords, limitted_xyz);

    // quantileは処理が重いので必要な時だけ別途計算する
    if (quantile.has_value())
    {
        VoxelCoordMap<std::tuple<float, float, float>> vox2quantile =
            this->calc_vox_quantile(vox_coords, limitted_xyz, quantile.value());

        for (const auto& [vox_coord, quantile] : vox2quantile)
        {
            auto stats_it = vox_stats.find(vox_coord);
            if (stats_it != vox_stats.end() && stats_it->second)
            {
                VoxStats& target_vox_stats = *stats_it->second;
                target_vox_stats.quantile = {std::get<0>(quantile), std::get<1>(quantile), std::get<2>(quantile),
                                             false};
            }
        }
    }

    // 葉ノードの数の八分木ノードを作成する
    Container octonodes;
    for (int i = 0; i < vox_coords.rows(); i++)
    {
        const auto key = std::make_tuple(vox_coords(i, 0), vox_coords(i, 1), vox_coords(i, 2));
        std::shared_ptr<VoxStats> stats_ptr;
        auto stats_it = vox_stats.find(key);
        if (stats_it != vox_stats.end())
        {
            stats_ptr = stats_it->second;
        }
        auto node = OctoNode(key, entity, stats_ptr);
        if constexpr (std::is_same_v<Container, OctoMap>)
        {
            octonodes.try_emplace(key, node);
        }
        else if constexpr (std::is_same_v<Container, OctoQueue>)
        {
            octonodes.emplace_back(key, node);
        }
        else
        {
            throw std::invalid_argument("map or deque is supproted for this function");
        }
    }
    return octonodes;
}

/**
 * @brief 点群を八分木に入れるための形式に変換するメソッド
 * @remarks
 * 統計量の計算をしないほうが早いので、統計量を使って後続処理をしない場合は、_gen_octonodes_without_statsを読んだ方が良い
 * @remarks
 * genericsを使っているのは、ノードをmapで保持する場合と、queueで保持する場合があって、入れ物が違う以外は同じ処理になるため
 *
 * @param xyz 八分木に入れる点群
 * @param entity xyzに紐づけるNodeEntity
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @param removed_vox_min_points
 * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
 * nullの場合はこの条件で除外しない
 * @return Container OctoMapやOctoQueueを想定したもの, 離散座標と八分木のノードの組のコレクション
 */
template <typename Container>
Container OctoTree::_gen_octonodes_without_stats(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist)
{
    // 必要な範囲に点群を絞る
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> res =
        this->_gen_vox_for_octonodes(xyz, removed_vox_min_points, removed_vox_max_points, remove_dist, true);
    Eigen::MatrixXi vox_coords = res.first;

    // 葉ノードの数の八分木ノードを作成する
    Container octonodes;
    for (int i = 0; i < vox_coords.rows(); i++)
    {
        const auto key = std::make_tuple(vox_coords(i, 0), vox_coords(i, 1), vox_coords(i, 2));
        if constexpr (std::is_same_v<Container, OctoMap>)
        {
            octonodes.try_emplace(key, OctoNode(key, entity));
        }
        else if constexpr (std::is_same_v<Container, OctoQueue>)
        {
            octonodes.emplace_back(key, OctoNode(key, entity));
        }
        else
        {
            throw std::invalid_argument("map or deque is supproted for this function");
        }
    }
    return octonodes;
}

Eigen::MatrixXd OctoTree::vox2oct_coords(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords,
                                         std::optional<int> tree_depth)
{
    if (vox_coords.size() == 0)
    {
        return Eigen::MatrixXd();
    }

    Eigen::Vector3d _cell_interval;
    if (tree_depth == std::nullopt)
    {
        _cell_interval = this->cell_interval;
    }
    else
    {
        if (tree_depth > this->max_tree_depth)
        {
            throw std::invalid_argument("tree_depth should be less than " + std::to_string(this->max_tree_depth) + ".");
        }
        _cell_interval = (this->max_xyz - this->min_xyz) / pow(2, tree_depth.value());
    }
    // NOTE MatrixとVectorの演算は以下の様に行う
    // mat.array().rowwise() - vec.transpose().array()
    Eigen::MatrixXd center = vox_coords.cast<double>().array() + 0.5;
    center = (center.array().rowwise() * _cell_interval.transpose().array()).matrix();
    center = center.array().rowwise() + this->min_xyz.transpose().array();

    return center;
}

Eigen::MatrixXi OctoTree::w2vox_coords(const Eigen::Ref<const Eigen::MatrixXd>& xyz, std::optional<int> tree_depth,
                                       bool remove_dep)
{
    Eigen::MatrixXd oct_coords = this->w2oct_coords(xyz);
    return oct2vox_coords(oct_coords, tree_depth, remove_dep);
}

Eigen::MatrixXi OctoTree::oct2vox_coords(const Eigen::Ref<const Eigen::MatrixXd>& oct_coords,
                                         std::optional<int> tree_depth, bool remove_dep) const
{
    Eigen::Vector3d _cell_interval;

    if (tree_depth == std::nullopt)
    {
        _cell_interval = this->cell_interval;
    }
    else
    {
        if (tree_depth > this->max_tree_depth)
        {
            throw std::invalid_argument("tree_depth should be less than " + std::to_string(this->max_tree_depth) + ".");
        }
        _cell_interval = (this->max_xyz - this->min_xyz) / pow(2, tree_depth.value());
    }

    // 八分木座標を離散座標に変換する

    Eigen::MatrixXi vox_coords =
        ((oct_coords.array().rowwise() - this->min_xyz.transpose().array()).matrix().array().rowwise() /
         _cell_interval.transpose().array())
            .cast<int>()
            .floor()
            .matrix();

    if (remove_dep && vox_coords.rows() != 0)
    {
        return helper::calc_unique_matrixxi(vox_coords);
    }

    return vox_coords;
}

Eigen::MatrixXd OctoTree::vox2w_coords(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords,
                                       std::optional<int> tree_depth)
{
    // 離散座標を八分木座標に変換する
    Eigen::MatrixXd oct_coords = this->vox2oct_coords(vox_coords, tree_depth);

    // 実座標に変換する
    return this->oct2w_coords(oct_coords);
}

Eigen::MatrixXi OctoTree::get_octonodes_vox_coord_unlabled(std::optional<int> tree_depth) const
{
    auto diff_tree_depth = this->_get_tree_diff(tree_depth);

    std::unordered_set<VoxelCoord, TupleHash> target_vox2deepest_vox;

    if (diff_tree_depth > 0)
    {
        for (const auto& [key, octonode] : unlabeled_octo_nodes.value())
        {
            auto ancestor_morton_code = OctoNode::morton_decode_3d(octonode.morton_code >> 3 * diff_tree_depth);
            target_vox2deepest_vox.insert(ancestor_morton_code);
        }

        Eigen::MatrixXi unlabeld_octo_nodes_keys({target_vox2deepest_vox.size(), size_t(3)});
        int i = 0;
        for (const auto& value : target_vox2deepest_vox)
        {
            unlabeld_octo_nodes_keys(i, 0) = std::get<0>(value);
            unlabeld_octo_nodes_keys(i, 1) = std::get<1>(value);
            unlabeld_octo_nodes_keys(i, 2) = std::get<2>(value);
            i++;
        }

        return unlabeld_octo_nodes_keys;
    }
    else
    {
        Eigen::MatrixXi unlabeld_octo_nodes_keys({this->unlabeled_octo_nodes.value().size(), size_t(3)});
        int i = 0;
        for (const auto& [key, value] : this->unlabeled_octo_nodes.value())
        {
            unlabeld_octo_nodes_keys(i, 0) = std::get<0>(key);
            unlabeld_octo_nodes_keys(i, 1) = std::get<1>(key);
            unlabeld_octo_nodes_keys(i, 2) = std::get<2>(key);
            i++;
        }

        return unlabeld_octo_nodes_keys;
    }
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] Eigen::MatrixXd
OctoTree::get_octonodes_oct_coord_unlabeled(std::optional<int> tree_depth)
{
    this->_clustering_tree_depth = tree_depth != std::nullopt ? tree_depth : std::optional<int>(this->max_tree_depth);
    auto diff_tree_depth = this->_get_tree_diff(tree_depth);

    // octo_nodeから離散座標を取得する
    // 最下層よりも上位ノードでクラスタリングを行う場合は、該当階層の離散座標と最下層の離散座標の対応表がデータ生成及び、クラスタリング結果の反映で必要になるため、前もって対応表を作る
    this->cluster_vox2deepest_vox.clear();
    if (diff_tree_depth > 0)
    {
        for (const auto& [key, octonode] : this->unlabeled_octo_nodes.value())
        {
            auto ancestor_morton_code = OctoNode::morton_decode_3d(octonode.morton_code >> 3 * diff_tree_depth);
            this->cluster_vox2deepest_vox.try_emplace(ancestor_morton_code, std::vector<std::tuple<int, int, int>>());
            this->cluster_vox2deepest_vox.at(ancestor_morton_code).push_back(key);
        }

        Eigen::MatrixXi vox_coords({this->cluster_vox2deepest_vox.size(), size_t(3)});
        int i = 0;
        for (const auto& [key, value] : this->cluster_vox2deepest_vox)
        {
            vox_coords(i, 0) = std::get<0>(key);
            vox_coords(i, 1) = std::get<1>(key);
            vox_coords(i, 2) = std::get<2>(key);
            i++;
        }

        return this->vox2oct_coords(vox_coords, tree_depth);
    }
    else
    {
        Eigen::MatrixXi vox_coords({this->unlabeled_octo_nodes.value().size(), size_t(3)});
        int i = 0;
        for (const auto& [key, value] : this->unlabeled_octo_nodes.value())
        {
            vox_coords(i, 0) = std::get<0>(key);
            vox_coords(i, 1) = std::get<1>(key);
            vox_coords(i, 2) = std::get<2>(key);
            i++;
        }

        return this->vox2oct_coords(vox_coords, tree_depth);
    }
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] Eigen::MatrixXd
OctoTree::get_octonodes_np_coord_unlabeled(std::optional<int> tree_depth)
{
    this->_clustering_tree_depth = tree_depth != std::nullopt ? tree_depth : std::optional<int>(this->max_tree_depth);
    auto diff_tree_depth = this->_get_tree_diff(tree_depth);

    // octo_nodeから離散座標を取得する
    // 最下層よりも上位ノードでクラスタリングを行う場合は、該当階層の離散座標と最下層の離散座標の対応表がデータ生成及び、クラスタリング結果の反映で必要になるため、前もって対応表を作る
    this->cluster_vox2deepest_vox.clear();
    if (diff_tree_depth > 0)
    {
        for (const auto& [key, octonode] : this->unlabeled_octo_nodes.value())
        {
            auto ancestor_morton_code = OctoNode::morton_decode_3d(octonode.morton_code >> 3 * diff_tree_depth);
            this->cluster_vox2deepest_vox.try_emplace(ancestor_morton_code, std::vector<std::tuple<int, int, int>>());
            this->cluster_vox2deepest_vox.at(ancestor_morton_code).push_back(key);
        }

        Eigen::MatrixXi vox_coords(this->cluster_vox2deepest_vox.size(), size_t(3));
        int i = 0;
        for (const auto& [key, value] : this->cluster_vox2deepest_vox)
        {
            vox_coords(i, 0) = std::get<0>(key);
            vox_coords(i, 1) = std::get<1>(key);
            vox_coords(i, 2) = std::get<2>(key);
            i++;
        }

        return this->vox2w_coords(vox_coords, tree_depth);
    }
    else
    {
        Eigen::MatrixXi vox_coords(this->unlabeled_octo_nodes.value().size(), size_t(3));
        int i = 0;
        for (const auto& [key, value] : this->unlabeled_octo_nodes.value())
        {
            vox_coords(i, 0) = std::get<0>(key);
            vox_coords(i, 1) = std::get<1>(key);
            vox_coords(i, 2) = std::get<2>(key);
            i++;
        }

        return this->vox2w_coords(vox_coords, tree_depth);
    }
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] Eigen::MatrixXd
OctoTree::get_octonodes_np_coord_labeled(const std::optional<std::vector<int>>& target_labels,
                                         std::optional<int> tree_depth)
{
    if (!this->labeled_octo_nodes.has_value() || this->labeled_octo_nodes.value().empty())
    {
        return Eigen::MatrixXd();
    }

    std::vector<std::variant<int, std::string>> _target_labels;
    if (target_labels != std::nullopt)
    {
        for (const auto& value : target_labels.value())
        {
            _target_labels.push_back(value);
        }
    }
    else
    {
        for (const auto& [key, value] : this->labeled_octo_nodes.value())
        {
            _target_labels.push_back(key);
        }
    }

    auto _tree_depth = (tree_depth != std::nullopt) ? tree_depth.value() : this->max_tree_depth;

    if (_tree_depth > this->max_tree_depth)
    {
        throw std::invalid_argument("tree_depth should be less than" + std::to_string(this->max_tree_depth) + ".");
    }

    // 保持しているモートン順序よりもいくつ上位の階層の座標が欲しいかを計算
    auto diff_tree_depth = this->max_tree_depth - _tree_depth;

    Eigen::MatrixXi vox_coords({_target_labels.size(), size_t(3)});
    int i = 0;
    for (const auto& target_label : _target_labels)
    {
        if (diff_tree_depth > 0)
        {
            // 保持しているモートン順序と欲しい階層が異なる場合は、欲しい階層におけるモートン順序を取得する
            //  集合で保持して、重複を取り除く
            for (const auto& [key, octo_node] : this->labeled_octo_nodes.value().at(target_label))
            {
                // 重複を除いた後に、モートン順序を離散座標に戻す
                auto value = OctoNode::morton_decode_3d(octo_node.morton_code >> (3 * diff_tree_depth));
                vox_coords(i, 0) = std::get<0>(value);
                vox_coords(i, 1) = std::get<1>(value);
                vox_coords(i, 2) = std::get<2>(value);
            }
        }
        else
        {
            for (const auto& [key, octo_node] : this->labeled_octo_nodes.value().at(target_label))
            {
                vox_coords(i, 0) = std::get<0>(key);
                vox_coords(i, 1) = std::get<1>(key);
                vox_coords(i, 2) = std::get<2>(key);
            }
        }
        i++;
    }

    return this->vox2w_coords(vox_coords, tree_depth);
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] Eigen::MatrixXd
OctoTree::get_octonodes_np_coord_labeled_v2(const std::optional<std::vector<int>>& target_labels,
                                            std::optional<int> tree_depth)
{
    if (!this->unlabeled_octo_nodes.has_value() || this->unlabeled_octo_nodes.value().empty())
    {
        return Eigen::MatrixXd();
    }

    auto _tree_depth = (tree_depth != std::nullopt) ? tree_depth.value() : this->max_tree_depth;

    if (_tree_depth > this->max_tree_depth)
    {
        throw std::invalid_argument("tree_depth should be less than" + std::to_string(this->max_tree_depth) + ".");
    }
    // 保持しているモートン順序よりもいくつ上位の階層の座標が欲しいかを計算
    auto diff_tree_depth = this->max_tree_depth - _tree_depth;

    // ここで必要なoctonodeを取り出して、後続処理に渡した方が効率が良さそうなので、そのように修正する
    std::vector<OctoNode> _chosen_octonodes;
    Eigen::MatrixXi vox_coords({0, size_t(3)});
    // int i = 0;
    for (const auto& [key, octonode] : this->unlabeled_octo_nodes.value())
    {
        if (octonode.get_cluster_label().has_value())
        {
            bool is_append =
                false; // 条件が複雑なので、条件内に無理やりvox_coordsへの追加処理を入れ外側で追加処理を入れる
            if (target_labels.has_value())
            {
                std::vector<int> _target_labels = target_labels.value();
                auto it = std::find(_target_labels.begin(), _target_labels.end(), octonode.get_cluster_label().value());
                if (it != _target_labels.end())
                {
                    // octonodeのクラスタラベルがtarget_labelsに含まれる場合
                    is_append = true;
                }
            }
            else
            {
                // target_labelsが設定されていない場合
                is_append = true;
            }

            if (is_append)
            {
                std::tuple<int, int, int> value = diff_tree_depth > 0
                                                      ? (std::tuple<int, int, int>)OctoNode::morton_decode_3d(
                                                            octonode.morton_code >> (3 * diff_tree_depth))
                                                      : key;
                // auto value = OctoNode::morton_decode_3d(octonode.morton_code >> (3 *
                // diff_tree_depth));
                vox_coords.conservativeResize(vox_coords.rows() + 1, Eigen::NoChange);
                vox_coords.row(vox_coords.rows() - 1) << std::get<0>(value), std::get<1>(value), std::get<2>(value);
                // vox_coords(i, 0) = std::get<0>(value);
                // vox_coords(i, 1) = std::get<1>(value);
                // vox_coords(i, 2) = std::get<2>(value);
                // i++;
            }
        }
    }

    return this->vox2w_coords(vox_coords, tree_depth);
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] Eigen::MatrixXd
OctoTree::get_octonodes_np_coord_entity(const std::vector<NodeEntity>& target_entities, std::optional<int> tree_depth)
{
    if (!this->labeled_octo_nodes.has_value() || this->labeled_octo_nodes.value().empty())
    {
        return Eigen::MatrixXd();
    }

    auto diff_tree_depth = this->_get_tree_diff(tree_depth);
    std::vector<std::tuple<int, int, int>> vox_coords;

    for (const auto& target_entity : target_entities)
    {
        std::map<std::tuple<int, int, int>, OctoNode> target_octonodes;
        for (const auto& [key, value] : this->unlabeled_octo_nodes.value())
        {
            if (value.entity == target_entity)
            {
                target_octonodes.insert_or_assign(key, value);
            }
        }

        if (target_octonodes.size() > 0)
        {
            // 該当するtarget_entityを持つものがいる場合のみリストに追加
            if (diff_tree_depth > 0)
            {
                // 保持しているモートン順序と欲しい階層が異なる場合は、欲しい階層におけるモートン順序を取得する
                // 集合で保持して、重複を取り除く
                for (const auto& [key, octo_node] : target_octonodes)
                {
                    // 重複を除いた後に、モートン順序を離散座標に戻す
                    vox_coords.push_back(OctoNode::morton_decode_3d(octo_node.morton_code >> (3 * diff_tree_depth)));
                }
            }
            else
            {
                for (const auto& [key, value] : target_octonodes)
                {
                    vox_coords.push_back(key);
                }
            }
        }
    }

    Eigen::MatrixXi np_vox_coords({vox_coords.size(), size_t(3)});
    int i = 0;
    for (const auto& value : vox_coords)
    {
        np_vox_coords(i, 0) = std::get<0>(value);
        np_vox_coords(i, 1) = std::get<1>(value);
        np_vox_coords(i, 2) = std::get<2>(value);
        i++;
    }

    return this->vox2w_coords(np_vox_coords, tree_depth);
}

void OctoTree::_check_data_label_consistency(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                             const Eigen::Ref<const Eigen::VectorXi>& labels) const
{
    if (clustered_data.rows() != labels.size())
    {
        throw std::invalid_argument(
            "クラスタリングで使ったデータとラベルの数が異なります: データ数:" + std::to_string(clustered_data.rows()) +
            ", ラベル数:" + std::to_string(labels.size()));
    }

    if (!this->_clustering_tree_depth)
    {
        throw std::logic_error("_clustering_tree_depthがnullになっており,クラスタリング結果が既に書き込まれているか, "
                               "クラスタリング前にメソッドが呼ばれています");
    }
}

Eigen::MatrixXd OctoTree::w2oct_coords(const Eigen::Ref<const Eigen::MatrixXd>& xyz) const
{
    if (xyz.rows() > 0 && this->origin_w2oct.has_value())
    {
        return xyz.array().rowwise() - this->origin_w2oct.value().transpose().array();
    }

    return xyz;
}

Eigen::MatrixXd OctoTree::oct2w_coords(const Eigen::Ref<const Eigen::MatrixXd>& oct_coords) const
{
    if (oct_coords.rows() > 0 && this->origin_w2oct.has_value())
    {
        return oct_coords.array().rowwise() + this->origin_w2oct.value().transpose().array();
    }
    return oct_coords;
}

OctoTree& OctoTree::_insert_clustering_result_in_deepest_layer(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                               const Eigen::Ref<const Eigen::MatrixXi>& labels)
{
    // クラスタリングで使った実座標データを八分木座標に変換する
    Eigen::MatrixXd oct_coords(this->w2oct_coords(clustered_data));

    // 八分木座標を離散座標に変換する
    Eigen::MatrixXi vox_coords(this->oct2vox_coords(oct_coords, std::nullopt, false));

    this->labeled_octo_nodes =
        std::map<std::variant<int, std::string>, std::map<std::tuple<int, int, int>, OctoNode>>();

    Eigen::VectorXi unique_labels = helper::calc_unique_vectorxi(labels);

    for (int i = 0; i < unique_labels.rows(); i++)
    {
        int label = unique_labels(i);
        // labelsでlabelと一致するものを1としたVector
        Eigen::VectorXi labels_ind = (labels.array() == label).cast<int>();
        Eigen::VectorXi ind = helper::nonzero(labels_ind);
        Eigen::MatrixXi coords = vox_coords(ind, Eigen::all);

        std::map<std::tuple<int, int, int>, OctoNode> m;
        for (int j = 0; j < coords.rows(); j++)
        {
            const auto& elem = std::make_tuple(coords(j, 0), coords(j, 1), coords(j, 2));
            const auto& value = this->unlabeled_octo_nodes.value().at(elem);
            m.insert_or_assign(elem, value);
        }
        this->labeled_octo_nodes.value().insert_or_assign(label, m);
    }

    return *this;
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] OctoTree&
OctoTree::insert_entity_result(const std::map<int, NodeEntity>& cluster2entity)
{
    if (!this->labeled_octo_nodes.has_value() || this->labeled_octo_nodes.value().empty())
    {
        throw std::invalid_argument("クラスタリング結果が付与される前にOctoNodeに属性を付与しようとして"
                                    "います。先にクラスタリングを行ってください。");
    }

    for (const auto& [label, entity] : cluster2entity)
    {
        if (this->labeled_octo_nodes.value().find(label) == this->labeled_octo_nodes.value().end())
        {
            continue;
        }

        for (auto& [key, node] : this->labeled_octo_nodes.value().at(label))
        {
            node.entity = entity;
            this->unlabeled_octo_nodes.value().at(key).entity = entity;
        }
    }

    return *this;
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] OctoTree&
OctoTree::insert_clustering_result(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                   const Eigen::Ref<const Eigen::VectorXi>& labels)
{
    // クラスタリング結果のチェック
    this->_check_data_label_consistency(clustered_data, labels);

    // クラスタリングデータの深さを基に処理を分ける
    auto diff_tree_depth = this->_get_tree_diff(this->_clustering_tree_depth);

    if (diff_tree_depth == 0)
    {
        // 八分木の最下層のデータでクラスタリングした場合は、それ専用のメソッドで処理を行う
        return this->_insert_clustering_result_in_deepest_layer(clustered_data, labels);
    }
    //  クラスタリングで用いたデータを離散座標に変換する
    Eigen::MatrixXi clustered_vox_coords = this->w2vox_coords(clustered_data, this->_clustering_tree_depth, false);

    // クラスタリングデータに対応する各ラベルを取り出して、max_tree_depthにおけるoctonodeに紐づける
    this->labeled_octo_nodes =
        std::map<std::variant<int, std::string>, std::map<std::tuple<int, int, int>, OctoNode>>();

    Eigen::VectorXi unique_labels = helper::calc_unique_matrixxi(labels);

    for (int i = 0; i < unique_labels.size(); i++)
    {
        int label = unique_labels(i);
        Eigen::VectorXi labels_idx = (labels.array() == label).cast<int>();
        labels_idx = helper::nonzero(labels_idx);
        Eigen::MatrixXi labeled_clustered_vox_coords = clustered_vox_coords(labels_idx, Eigen::all);

        std::map<std::tuple<int, int, int>, OctoNode> contains_octo_nodes;
        for (int j = 0; j < labeled_clustered_vox_coords.rows(); j++)
        {
            const auto cluster_vox =
                std::make_tuple(labeled_clustered_vox_coords(j, 0), labeled_clustered_vox_coords(j, 1),
                                labeled_clustered_vox_coords(j, 2));
            this->cluster_vox2deepest_vox.try_emplace(cluster_vox);
            for (const auto& deepest_vox : this->cluster_vox2deepest_vox.at(cluster_vox))
            {
                const auto& value = this->unlabeled_octo_nodes.value().at(deepest_vox);
                contains_octo_nodes.insert_or_assign(deepest_vox, value);
            }
        }
        this->labeled_octo_nodes.value().insert_or_assign(label, contains_octo_nodes);
    }

    this->_clustering_tree_depth = std::nullopt;
    return *this;
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] OctoTree&
OctoTree::_insert_clustering_result_in_deepest_layer_v2(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                        const Eigen::Ref<const Eigen::MatrixXi>& labels)
{
    // クラスタリングで使った実座標データを八分木座標に変換する
    Eigen::MatrixXd oct_coords(this->w2oct_coords(clustered_data));

    // 八分木座標を離散座標に変換する
    Eigen::MatrixXi vox_coords(this->oct2vox_coords(oct_coords, std::nullopt, false));

    // this->labeled_octo_nodes =
    //     std::map<std::variant<int, std::string>,
    //              std::map<std::tuple<int, int, int>, OctoNode>>();

    Eigen::VectorXi unique_labels = helper::calc_unique_vectorxi(labels);

    for (int i = 0; i < unique_labels.rows(); i++)
    {
        int label = unique_labels(i);
        // labelsでlabelと一致するものを1としたVector
        Eigen::VectorXi labels_ind = (labels.array() == label).cast<int>();
        Eigen::VectorXi ind = helper::nonzero(labels_ind);
        Eigen::MatrixXi coords = vox_coords(ind, Eigen::all);

        std::map<std::tuple<int, int, int>, OctoNode> m;
        for (int j = 0; j < coords.rows(); j++)
        {
            const auto& elem = std::make_tuple(coords(j, 0), coords(j, 1), coords(j, 2));
            // const auto& value = this->unlabeled_octo_nodes.value().at(elem);
            this->unlabeled_octo_nodes.value().at(elem).set_cluster_label(label);
            // m.insert_or_assign(elem, value);
        }
        // this->labeled_octo_nodes.value().insert_or_assign(label, m);
    }

    return *this;
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] OctoTree&
OctoTree::insert_clustering_result_v2(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                      const Eigen::Ref<const Eigen::VectorXi>& labels)
{
    // クラスタリング結果のチェック
    this->_check_data_label_consistency(clustered_data, labels);

    // クラスタリングデータの深さを基に処理を分ける
    auto diff_tree_depth = this->_get_tree_diff(this->_clustering_tree_depth);

    if (diff_tree_depth == 0)
    {
        // 八分木の最下層のデータでクラスタリングした場合は、それ専用のメソッドで処理を行う
        return this->_insert_clustering_result_in_deepest_layer_v2(clustered_data, labels);
    }
    //  クラスタリングで用いたデータを離散座標に変換する
    Eigen::MatrixXi clustered_vox_coords = this->w2vox_coords(clustered_data, this->_clustering_tree_depth, false);

    // クラスタリングデータに対応する各ラベルを取り出して、max_tree_depthにおけるoctonodeに紐づける
    // this->labeled_octo_nodes =
    //    std::map<std::variant<int, std::string>,
    //             std::map<std::tuple<int, int, int>, OctoNode>>();

    Eigen::VectorXi unique_labels = helper::calc_unique_matrixxi(labels);

    for (int i = 0; i < unique_labels.size(); i++)
    {
        int label = unique_labels(i);
        Eigen::VectorXi labels_idx = (labels.array() == label).cast<int>();
        labels_idx = helper::nonzero(labels_idx);
        Eigen::MatrixXi labeled_clustered_vox_coords = clustered_vox_coords(labels_idx, Eigen::all);

        std::map<std::tuple<int, int, int>, OctoNode> contains_octo_nodes;
        for (int j = 0; j < labeled_clustered_vox_coords.rows(); j++)
        {
            const auto cluster_vox =
                std::make_tuple(labeled_clustered_vox_coords(j, 0), labeled_clustered_vox_coords(j, 1),
                                labeled_clustered_vox_coords(j, 2));
            this->cluster_vox2deepest_vox.try_emplace(cluster_vox);
            for (const auto& deepest_vox : this->cluster_vox2deepest_vox.at(cluster_vox))
            {
                this->unlabeled_octo_nodes.value().at(deepest_vox).set_cluster_label(label);
                // const auto& value =
                //     this->unlabeled_octo_nodes.value().at(deepest_vox);
                // contains_octo_nodes.insert_or_assign(deepest_vox, value);
            }
        }
        // this->labeled_octo_nodes.value().insert_or_assign(label,
        //                                                   contains_octo_nodes);
    }

    this->_clustering_tree_depth = std::nullopt;
    return *this;
}

[[deprecated("This method should not be necessary to handle the latest "
             "octotree")]] OctoTree&
OctoTree::insert_entity_result_v2(const std::map<int, NodeEntity>& cluster2entity)
{
    for (auto& [key, octonode] : this->unlabeled_octo_nodes.value())
    {
        std::optional<int> cluster_label = octonode.get_cluster_label();
        // octonode: OctoNode型の変数
        if (cluster_label.has_value() && cluster2entity.find(cluster_label.value()) != cluster2entity.end())
        {
            // octonodeのクラスタラベルがnullで、cluster2entityに該当するクラスタ番号があれば、octonodeのentityを更新
            octonode.entity = cluster2entity.at(cluster_label.value());
        }
    }

    return *this;
}

/*
NodeEntityのみでentity_octonodesに格納する
格納された点群は(null, entity)をkeyとするentity_octonodesに格納される
立体物点群などを入れるので、その設定をデフォルト引数にしている
+ 入力:
    1. xyz: entity_octonodesに入れられる点群
    2. entity: 格納するNodeEntity
    3. entity_replace: 該当するentityのkeyを新しくするかどうか, trueの場合、(*,
entity)に該当するkeyは全て削除して、xyzが格納される, falseの場合、(null, entity)のkeyだけxyzに更新される
    4. is_order: trueの場合、xyzの行順にデータが保持される
    5. vox_min/max_points: それぞれnullでない場合、min/maxの範囲離散座標が入っていなければ除外される,
どちらかがnullの場合は除外する処理は行われない
    6. remove_dist: 除外する大きさ, 離散座標における大きさになっている
+ 出力:
    八分木インスタンスそのもののメモリ番地を返す
*/
OctoTree& OctoTree::insert_or_entity_octonodes(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, const NodeEntity& entity, bool entity_replace, bool is_order,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist)
{

    ChunkOctoNodes new_octonodes;

    // 統計量の計算が必要かどうかで処理が分かれる
    if (this->use_node_stats)
    {
        if (is_order)
        {
            new_octonodes = this->_gen_octonodes_with_stats<OctoQueue>(
                xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist, this->quantile);
        }
        else
        {
            new_octonodes = this->_gen_octonodes_with_stats<OctoMap>(
                xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist, this->quantile);
        }
    }
    else
    {
        if (is_order)
        {
            new_octonodes = this->_gen_octonodes_without_stats<OctoQueue>(xyz, entity, removed_vox_min_points,
                                                                          removed_vox_max_points, remove_dist);
        }
        else
        {
            new_octonodes = this->_gen_octonodes_without_stats<OctoMap>(xyz, entity, removed_vox_min_points,
                                                                        removed_vox_max_points, remove_dist);
        }
    }

    if (entity_replace)
    {
        this->erase_nodes_for_entities_noret({entity});
    }
    this->entity_octonodes.insert_or_assign(NodeClusterKey{entity, std::nullopt}, new_octonodes);
    return *this;
}

/*
NodeEntity + クラスタ番号毎にOctoNodeをentity_octonodesに格納する
現状は崖点群を入れるのに使うので、その設定をデフォルト引数にしている
+ 入力:
    1. xyz: entity_octonodesに入れられる点群, n*3行列
    2. labels: 各点群のクラスタ番号, nベクトル
    3. entity: 格納するNodeEntity
    4. entity_replace: 該当するentityのkeyを新しくするかどうか, trueの場合、(*,
entity)に該当するkeyは全て削除して、xyzが格納される, falseの場合、(null, entity)のkeyだけxyzに更新される
    5. is_order: trueの場合、xyzの行順にデータが保持される
    6. vox_min/max_points: それぞれnullでない場合、min/maxの範囲離散座標が入っていなければ除外される,
どちらかがnullの場合は除外する処理は行われない
    7. remove_dist: 除外する大きさ, 離散座標における大きさになっている
*/
OctoTree& OctoTree::insert_or_entity_octonodes_with_labels(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, const Eigen::Ref<const Eigen::VectorXi>& labels,
    const NodeEntity& entity, bool entity_replace, bool is_order,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_min_points,
    const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
        removed_vox_max_points,
    int remove_dist)
{

    Eigen::VectorXi unique_labels = helper::calc_unique_vectorxi(labels);

    if (entity_replace)
    {
        this->erase_nodes_for_entities_noret({entity});
    }

    for (int i = 0; i < unique_labels.rows(); i++)
    {
        int label = unique_labels(i);
        // labelsでlabelと一致するものを1としたVector
        Eigen::VectorXi labels_ind = (labels.array() == label).cast<int>();
        Eigen::VectorXi ind = helper::nonzero(labels_ind);
        Eigen::MatrixXd label_xyz = xyz(ind, Eigen::all);

        // transform to value in map
        ChunkOctoNodes label_octonodes;
        if (this->use_node_stats)
        {
            if (is_order)
            {
                label_octonodes = this->_gen_octonodes_with_stats<OctoQueue>(
                    label_xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist, this->quantile);
            }
            else
            {
                label_octonodes = this->_gen_octonodes_with_stats<OctoMap>(
                    label_xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist, this->quantile);
            }
        }
        else
        {
            if (is_order)
            {
                label_octonodes = this->_gen_octonodes_without_stats<OctoQueue>(
                    label_xyz, entity, removed_vox_min_points, removed_vox_max_points, remove_dist);
            }
            else
            {
                label_octonodes = this->_gen_octonodes_without_stats<OctoMap>(label_xyz, entity, removed_vox_min_points,
                                                                              removed_vox_max_points, remove_dist);
            }
        }
        this->entity_octonodes.insert_or_assign(NodeClusterKey{entity, label}, label_octonodes);
    }

    return *this;
}

/** entity, cluster_idsに対応するentity_octonodesを切り出してkey, valueを保持したままで値を返すメソッド,
entityは一つだけだが、cluster_idsは複数取り出すこともできる
+ 入力:
    1. entity: 取り出したいNodeEntity
    2. cluster_ids: 取り出したいクラスタ番号, nullの場合entityに該当するもの全てを取り出す
+ 出力:
    {(cluster_id, entity): value for cluster_id in cluster_ids}みたいな辞書

 */
std::map<NodeClusterKey, OctoMap>
OctoTree::collect_by_clusters_with_key(const NodeEntity& entity,
                                       const std::optional<std::vector<std::optional<int>>>& cluster_ids)
{
    const auto& map = this->entity_octonodes;
    std::map<NodeClusterKey, OctoMap> result;

    auto to_octomap = [](const ChunkOctoNodes& nodes) -> OctoMap
    {
        OctoMap merged;
        std::visit(
            [&merged](auto&& container)
            {
                using T = std::decay_t<decltype(container)>;
                if constexpr (std::is_same_v<T, OctoMap>)
                {
                    merged.insert(container.begin(), container.end());
                }
                else if constexpr (std::is_same_v<T, OctoQueue>)
                {
                    for (const auto& pair : container)
                    {
                        merged.insert_or_assign(pair.first, pair.second);
                    }
                }
                else
                {
                    throw std::logic_error("Unsupported ChunkOctoNodes type");
                }
            },
            nodes);
        return merged;
    };

    if (cluster_ids)
    {
        for (const auto& cid : cluster_ids.value())
        {
            auto range = map.equal_range(NodeClusterKey{entity, cid});
            for (auto it = range.first; it != range.second; ++it)
            {
                result[it->first] = to_octomap(it->second);
            }
        }
    }
    else
    {
        auto lower = map.lower_bound(NodeClusterKey{entity, std::nullopt});
        auto upper = map.upper_bound(NodeClusterKey{entity, std::numeric_limits<int>::max()});
        for (auto it = lower; it != upper; ++it)
        {
            result[it->first] = to_octomap(it->second);
        }
    }
    return result;
}

/**
 * @brief entityの中で、cluster_idsに該当するクラスタのChunkOctoNodesをOctoMapでひとまとめにする
 *
 * @param entity 取り出す対象のentity
 * @param cluster_ids 取り出す対象のクラスタ番号, nulloptの場合、該当するentity全てを取り出す
 * @return OctoMap 全部を重ね合わせたOctoMap
 */
OctoMap OctoTree::collect_nodes_by_clusters(const NodeEntity& entity,
                                            const std::optional<std::vector<std::optional<int>>>& cluster_ids)
{
    const auto& map = this->entity_octonodes;

    OctoMap result;
    auto add_nodes = [&result](const ChunkOctoNodes& nodes)
    {
        std::visit(
            [&result](auto&& container)
            {
                using T = std::decay_t<decltype(container)>;
                if constexpr (std::is_same_v<T, OctoMap>)
                {
                    result.insert(container.begin(), container.end());
                }
                else if constexpr (std::is_same_v<T, OctoQueue>)
                {
                    for (const auto& pair : container)
                    {
                        result.insert(pair);
                    }
                }
                else
                {
                    throw std::logic_error("Unsupported ChunkOctoNodes type");
                }
            },
            nodes);
    };
    if (cluster_ids)
    {
        for (const auto& cid : cluster_ids.value())
        {
            auto range = map.equal_range(NodeClusterKey{entity, cid});
            for (auto it = range.first; it != range.second; ++it)
            {
                add_nodes(it->second);
            }
        }
    }
    else
    {
        auto lower = map.lower_bound(NodeClusterKey{entity, std::nullopt});
        auto upper = map.upper_bound(NodeClusterKey{entity, std::numeric_limits<int>::max()});
        for (auto it = lower; it != upper; ++it)
        {
            add_nodes(it->second);
        }
    }
    return result;
}

/**
 * @brief entitiesとclusters_idsに該当するクラスタをOctoMapでひとまとめにする
 *
 * @param entities 取り出す対象のentityのリスト
 * @param cluster_ids 取り出す対象のクラスタ番号, nulloptの場合、該当するentity全てを取り出す
 * @return OctoMap
 */
OctoMap
OctoTree::collect_nodes_by_entities_and_clusters(const std::vector<NodeEntity>& entities,
                                                 const std::optional<std::vector<std::optional<int>>>& cluster_ids)
{
    OctoMap result;

    for (const auto& entity : entities)
    {
        OctoMap partial = this->collect_nodes_by_clusters(entity, cluster_ids);
        for (auto& [coord, node] : partial)
        {
            result.insert_or_assign(coord, std::move(node));
        }
    }

    return result;
}

/**
 * @brief
 * entitiesに該当する八分木ノードでdepthだけ上に登った中でvox_rangesの範囲に入っているものを、登った階層で保持してまとめてOctoMapに変換してを返す
 *
 * @param entities 取り出す対象のentityのリスト
 * @param vox_ranges depthの高さでの離散座標の範囲, nullの場合何もしない
 * @param depth 一番下の階層から何階層登ったところで八分木ノードを取り出すかを表す整数, nullの場合一番下の階層
 * @return OctoMap 取り出した八分木ノードをまとめたOctoMap
 */
OctoMap
OctoTree::collect_nodes_by_entities_with_depth(const std::vector<NodeEntity>& entities,
                                               const std::vector<std::optional<std::tuple<int, int>>>& vox_ranges,
                                               std::optional<int> depth)
{
    const auto& map = this->entity_octonodes;
    auto diff_tree_depth = this->_get_tree_diff(depth);
    std::unordered_set<NodeEntity, NodeEntityHash> entity_set(entities.begin(), entities.end());

    OctoMap target_octonodes;

    auto in_range = [&](const VoxelCoord& coord) -> bool
    {
        if (vox_ranges.empty())
            return true;
        auto [x, y, z] = coord;
        int vals[3] = {x, y, z};
        // std::cout << "x = " << x << "y = " << y << std::endl;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (vox_ranges[axis])
            {
                auto [min_v, max_v] = *vox_ranges[axis];
                if (vals[axis] < min_v || vals[axis] > max_v)
                {
                    return false;
                }
            }
        }
        return true;
    };

    for (auto it = map.begin(); it != map.end(); ++it)
    {
        if (entity_set.count(it->first.entity))
        {
            std::visit(
                [&](auto&& container)
                {
                    using T = std::decay_t<decltype(container)>;
                    if constexpr (std::is_same_v<T, OctoMap>)
                    {
                        for (const auto& [vox_coord, octonode] : container)
                        {
                            VoxelCoord append_vox_coord =
                                diff_tree_depth > 0
                                    ? OctoNode::morton_decode_3d(octonode.morton_code >> (3 * diff_tree_depth))
                                    : vox_coord;
                            if (in_range(append_vox_coord))
                            {
                                target_octonodes.insert_or_assign(append_vox_coord, octonode);
                            }
                        }
                    }
                    else if constexpr (std::is_same_v<T, OctoQueue>)
                    {
                        for (const auto& pair : container)
                        {
                            VoxelCoord append_vox_coord =
                                diff_tree_depth > 0
                                    ? OctoNode::morton_decode_3d(pair.second.morton_code >> (3 * diff_tree_depth))
                                    : pair.first;
                            if (in_range(append_vox_coord))
                            {
                                target_octonodes.insert_or_assign(append_vox_coord, pair.second);
                            }
                        }
                    }
                    else
                    {
                        throw std::domain_error("type of chunk_octonodes should be OctoMap or OctoQueue");
                    }
                },
                it->second);
        }
    }

    return target_octonodes;
}

/**
 * @brief target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層でまとめて実座標に変換して返す
 *
 * @param target_entities 取り出す対象のentityのリスト
 * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
 * @return Eigen::MatrixXd 実座標の点群 n*3
 */
Eigen::MatrixXd OctoTree::get_np_from_entity_octonodes_by_chunk(const std::vector<NodeEntity>& target_entities,
                                                                std::optional<int> tree_depth)
{
    if (this->entity_octonodes.empty())
    {
        return Eigen::MatrixXd();
    }

    OctoMap target_octonodes_in_depth = this->collect_nodes_by_entities_with_depth(target_entities, {}, tree_depth);

    Eigen::MatrixXi np_vox_coords({target_octonodes_in_depth.size(), size_t(3)});
    int i = 0;
    for (const auto& [vox_coord, octonode] : target_octonodes_in_depth)
    {
        np_vox_coords(i, 0) = std::get<0>(vox_coord);
        np_vox_coords(i, 1) = std::get<1>(vox_coord);
        np_vox_coords(i, 2) = std::get<2>(vox_coord);
        i++;
    }

    return this->vox2w_coords(np_vox_coords, tree_depth);
}

/**
 * @brief target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層でまとめて実座標に変換して返す
 *
 * @param target_entities 取り出す対象のentityのリスト
 * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
 * @return Eigen::MatrixXd 実座標の点群 n*3
 */
Eigen::MatrixXi OctoTree::get_vox_from_entity_octonodes_by_chunk(const std::vector<NodeEntity>& target_entities,
                                                                 std::optional<int> tree_depth)
{
    if (this->entity_octonodes.empty())
    {
        return Eigen::MatrixXi();
    }

    auto diff_tree_depth = this->_get_tree_diff(tree_depth);
    std::unordered_set<VoxelCoord, TupleHash> vox_coords;
    OctoMap target_octonodes;

    // target_entitiesに該当するChunkOctoNodesを取り出して、OctoMapでまとめる
    for (const auto& target_entity : target_entities)
    {
        for (const auto& [node_cluster_key, chunk_octonodes] : this->entity_octonodes)
        {
            if (node_cluster_key.entity == target_entity)
            {
                std::visit(
                    [&](auto&& container)
                    {
                        using T = std::decay_t<decltype(container)>;
                        if constexpr (std::is_same_v<T, OctoMap>)
                        {
                            for (const auto& [vox_coord, octonode] : container)
                            {
                                target_octonodes.insert_or_assign(vox_coord, octonode);
                                // target_octonodes.emplace_back(vox_coord, octonode);
                            }
                        }
                        else if constexpr (std::is_same_v<T, OctoQueue>)
                        {
                            for (const auto& pair : container)
                            {
                                // target_octonodes.push_back(pair)
                                target_octonodes.insert_or_assign(pair.first, pair.second);
                            }
                        }
                        else
                        {
                            throw std::domain_error("type of chunk_octonodes should be OctoMap or OctoQueue");
                        }
                    },
                    chunk_octonodes);
            }
        }
    }

    if (target_octonodes.size() > 0)
    {
        for (const auto& [vox_coord, octonode] : target_octonodes)
        {
            VoxelCoord append_vox_coord =
                diff_tree_depth > 0 ? OctoNode::morton_decode_3d(octonode.morton_code >> (3 * diff_tree_depth))
                                    : vox_coord;
            vox_coords.insert(append_vox_coord);
        }
    }

    Eigen::MatrixXi np_vox_coords({vox_coords.size(), size_t(3)});
    int i = 0;
    for (const auto& value : vox_coords)
    {
        np_vox_coords(i, 0) = std::get<0>(value);
        np_vox_coords(i, 1) = std::get<1>(value);
        np_vox_coords(i, 2) = std::get<2>(value);
        i++;
    }

    return np_vox_coords;
}

/**
 * @brief target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層で実座標に変換して返す,
 * key毎にリストで持つ場合に使うメソッド
 *
 * @param target_entities 取り出す対象のentityのリスト
 * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
 * @return std::vector<Eigen::MatrixXd> keyの数の要素を持つ配列で、各要素に実座標の点群が入っている
 */
std::vector<Eigen::MatrixXd>
OctoTree::get_np_from_entity_octonodes_by_list(const std::vector<NodeEntity>& target_entities,
                                               std::optional<int> tree_depth)
{
    if (this->entity_octonodes.empty())
    {
        return {};
    }

    auto diff_tree_depth = this->_get_tree_diff(tree_depth);

    auto& map = this->entity_octonodes;
    std::unordered_set<NodeEntity, NodeEntityHash> entity_set(target_entities.begin(), target_entities.end());
    std::vector<Eigen::MatrixXd> group_mat;
    group_mat.reserve(entity_set.size());
    for (auto it = map.begin(); it != map.end(); ++it)
    {
        // 対象のentityかチェック
        if (entity_set.count(it->first.entity))
        {
            std::vector<VoxelCoord> one_group_voxel;
            std::unordered_set<VoxelCoord, VoxelCoordHash>
                vox_seen; // one_group_voxel内での重複を除去するためのコレクション
            std::visit(
                [&](auto&& container)
                {
                    // ChunkOctoNodes内の要素を見ていく
                    for (const auto& pair : container)
                    {
                        const OctoNode& node = pair.second;
                        const VoxelCoord& base_coord = pair.first;

                        // 最下層より上の階層で点群を取得する場合はシフトしてdecode
                        VoxelCoord append_vox_coord =
                            diff_tree_depth > 0 ? OctoNode::morton_decode_3d(node.morton_code >> (3 * diff_tree_depth))
                                                : base_coord;

                        // 重複要素のチェック, insertのsecondで判定できる
                        if (vox_seen.insert(append_vox_coord).second)
                        {
                            one_group_voxel.push_back(append_vox_coord);
                        }
                    }
                    // using T = std::decay_t<decltype(container)>;
                    // if constexpr (std::is_same_v<T, OctoMap>) {
                    //   for (const auto &[vox_coord, octonode] : container) {
                    //     VoxelCoord append_vox_coord =
                    //         diff_tree_depth > 0
                    //             ? OctoNode::morton_decode_3d(octonode.morton_code >>
                    //                                          (3 * diff_tree_depth))
                    //             : vox_coord;
                    //     if (vox_seen.insert(append_vox_coord).second) {
                    //       one_group_voxel.insert(append_vox_coord);
                    //     }
                    //   }

                    //} else if constexpr (std::is_same_v<T, OctoQueue>) {
                    //  for (const auto &pair : container) {
                    //    VoxelCoord append_vox_coord =
                    //        diff_tree_depth > 0
                    //            ? OctoNode::morton_decode_3d(pair.second.morton_code >>
                    //                                         (3 * diff_tree_depth))
                    //            : pair.first;
                    //    one_group_voxel.insert(append_vox_coord);
                    //  }

                    //} else {
                    //  throw std::domain_error(
                    //      "type of chunk_octonodes should be OctoMap or OctoQueue");
                    //}
                },
                it->second);

            // 離散座標を実座標に変換する
            Eigen::MatrixXi np_vox_coords(one_group_voxel.size(), size_t(3));
            int i = 0;
            for (const auto& value : one_group_voxel)
            {
                np_vox_coords(i, 0) = std::get<0>(value);
                np_vox_coords(i, 1) = std::get<1>(value);
                np_vox_coords(i, 2) = std::get<2>(value);
                i++;
            }
            group_mat.push_back(this->vox2w_coords(np_vox_coords, tree_depth));
        }
    }

    return group_mat;
}

/**
 * @brief cluster_entityのクラスタ未割当のChunkOctoNodesを取得し、cluster_depthの階層でLiDAR点群にして返す
 * Remark:
 * 普通のget_np...と異なる形で定義しているのは、クラスタリング結果を必要な階層で八分木ノードに入れる必要があり、その辺りの情報を作る部分があるため
 *
 * @param cluster_entity クラスタリング対象となるNodeEntity
 * @param cluster_depth 一番下の階層から何階層上でクラスタリングを行うかを表す整数
 * @return Eigen::MatrixXd クラスタリングに用いる実座標の点群 n*3
 */
Eigen::MatrixXd OctoTree::get_clustering_data_by_entity(NodeEntity cluster_entity, std::optional<int> cluster_depth)
{
    // クラスタリング対象の属性のデータをOctoMapで取り出す
    OctoMap unlabeled_octonodes = OctoTree::collect_nodes_by_clusters(cluster_entity, {std::nullopt});

    auto _cluster_depth = cluster_depth.has_value() ? cluster_depth : this->max_tree_depth;
    auto diff_tree_depth = this->_get_tree_diff(_cluster_depth);
    this->_clustering_tree_depth = _cluster_depth;

    // クラスタリングする階層と、最下層の対応付けを行う
    this->cluster_vox2deepest_vox.clear();
    if (diff_tree_depth > 0)
    {
        for (const auto& [key, octonode] : unlabeled_octonodes)
        {
            auto ancestor_vox_coord = OctoNode::morton_decode_3d(octonode.morton_code >> 3 * diff_tree_depth);
            this->cluster_vox2deepest_vox.try_emplace(ancestor_vox_coord, std::vector<std::tuple<int, int, int>>());
            this->cluster_vox2deepest_vox.at(ancestor_vox_coord).push_back(key);
        }
    }

    // map<VoxelCoord,
    // V>をMatrixXi<VoxelCoord>に変換して、それを実座標に移して結果を返す
    auto mapkey2mat = [](auto& map)
    {
        Eigen::MatrixXi mat(map.size(), size_t(3));
        size_t i = 0;
        for (const auto& kv : map)
        {
            const auto& key = kv.first;
            mat(i, 0) = std::get<0>(key);
            mat(i, 1) = std::get<1>(key);
            mat(i, 2) = std::get<2>(key);
            ++i;
        }
        return mat;
    };
    Eigen::MatrixXi vox_coords =
        (diff_tree_depth > 0) ? mapkey2mat(this->cluster_vox2deepest_vox) : mapkey2mat(unlabeled_octonodes);
    return this->vox2w_coords(vox_coords, _cluster_depth);
}

/**
 * @brief cluster_entityでクラスタが未割当のものを削除して、削除したものを返す
 * @param cluster_entity: 削除されるNodeEntity
 * @return OctoMap 削除されたOctoMap
 */
OctoMap OctoTree::pop_unlabeled_nodes(NodeEntity cluster_entity)
{
    auto& map = this->entity_octonodes;
    auto it = map.find(NodeClusterKey{cluster_entity, std::nullopt}); // NodeClusterKey, ChunkOctoNodes
    if (it == map.end())
    {
        throw std::logic_error("cluster_entityにクラスタ未割当のデータがないです。");
    }

    // クラスタ未割当データは、この処理で取り除くので、eraseする
    ChunkOctoNodes unlabeled_octonodes = std::move(it->second);
    map.erase(it);

    OctoMap octomap = std::visit(
        [](const auto&& nodes) -> const OctoMap
        {
            using T = std::decay_t<decltype(nodes)>;
            if constexpr (std::is_same_v<T, OctoMap>)
            {
                return nodes;
            }
            else
            {
                throw std::logic_error("Octomap以外のChunkOctoNodesは未対応です。");
            }
        },
        std::move(unlabeled_octonodes));

    return octomap;
}

/**
 * @brief entitiesに該当するkeyを削除する
 * @param entities 削除されるNodeEntityのリスト
 */
void OctoTree::erase_nodes_for_entities_noret(const std::vector<NodeEntity>& entities)
{
    auto& map = this->entity_octonodes;
    std::unordered_set<NodeEntity, NodeEntityHash> entity_set(entities.begin(), entities.end());

    for (auto it = map.begin(); it != map.end();)
    {
        if (entity_set.count(it->first.entity))
        {
            it = map.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

/**
 * @brief entitiesに該当するentity_octonodesのChunkOctoNodesを削除する
 *
 * @param entities 削除されるNodeEntityのリスト
 * @return std::vector<ChunkOctoNodes> 削除されたもののリスト
 */
std::vector<ChunkOctoNodes> OctoTree::erase_nodes_for_entities(const std::vector<NodeEntity>& entities)
{
    auto& map = this->entity_octonodes;
    std::vector<ChunkOctoNodes> removed_nodes;

    std::unordered_set<NodeEntity, NodeEntityHash> entity_set(entities.begin(), entities.end());

    for (auto it = map.begin(); it != map.end();)
    {
        if (entity_set.count(it->first.entity))
        {
            removed_nodes.push_back(std::move(it->second));
            it = map.erase(it);
        }
        else
        {
            ++it;
        }
    }

    return removed_nodes;
}

/**
 * @brief クラスタリング結果を基に、clustered_dataをcluster_entityに格納する,
 * 一番下の階層で各野する場合に呼ばれるメソッド
 * @param clustered_data クラスタリングに使った点群 n*3
 * @param labels クラスタリング結果 n次元ベクトル
 * @param cluster_entity クラスタリング結果を入れるNodeEntity
 * @return OctoTree& 八分木インスタンスそのもののメモリ番地
 */
OctoTree&
OctoTree::_insert_labels_and_move_in_octonodes_deepest_layer(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                             const Eigen::Ref<const Eigen::MatrixXi>& labels,
                                                             NodeEntity cluster_entity)
{
    OctoMap unlabeled_octonodes = this->pop_unlabeled_nodes(cluster_entity);

    // クラスタリングで使った実座標データを八分木座標に変換する
    Eigen::MatrixXd oct_coords(this->w2oct_coords(clustered_data));

    // 八分木座標を離散座標に変換する
    Eigen::MatrixXi vox_coords(this->oct2vox_coords(oct_coords, std::nullopt, false));

    Eigen::VectorXi unique_labels = helper::calc_unique_vectorxi(labels);

    for (int i = 0; i < unique_labels.rows(); i++)
    {
        int label = unique_labels(i);
        // labelsでlabelと一致するものを1としたVector
        Eigen::VectorXi labels_ind = (labels.array() == label).cast<int>();
        Eigen::VectorXi ind = helper::nonzero(labels_ind);
        Eigen::MatrixXi coords = vox_coords(ind, Eigen::all);

        OctoMap m;
        for (int j = 0; j < coords.rows(); j++)
        {
            const auto& elem = std::make_tuple(coords(j, 0), coords(j, 1), coords(j, 2));
            const auto& value = unlabeled_octonodes.at(elem);
            m.insert_or_assign(elem, value);
        }
        // this->entity_octonodes.value().insert_or_assign(NodeClusterKey{cluster_entity,
        // label}, m);
        this->entity_octonodes.insert_or_assign(NodeClusterKey{cluster_entity, label}, m);
    }

    this->_clustering_tree_depth = std::nullopt;

    return *this;
}

/**
 * @brief cluster_entityのラベル未割当のChunkOctoNodesに対して、クラスタリングデータとラベルを基にラベルを付与する
 *
 * @param clustered_data クラスタリングに使った点群 n*3
 * @param labels クラスタリング結果 n次元ベクトル
 * @param cluster_entity クラスタリング結果を入れるNodeEntity
 * @param cluster_depth クラスタリングで用いた階層, nullの場合一番下の階層
 * @return OctoTree& 八分木インスタンスそのもののメモリ番地
 */
OctoTree& OctoTree::insert_labeles_and_move_in_octonodes(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                         const Eigen::Ref<const Eigen::MatrixXi>& labels,
                                                         NodeEntity cluster_entity)
{
    // クラスタリング結果のチェック
    this->_check_data_label_consistency(clustered_data, labels);

    // クラスタリングデータの深さを基に処理を分ける
    auto diff_tree_depth = this->_get_tree_diff(this->_clustering_tree_depth);

    if (diff_tree_depth == 0)
    {
        // 八分木の最下層のデータでクラスタリングした場合は、それ専用のメソッドで処理を行う
        return this->_insert_labels_and_move_in_octonodes_deepest_layer(clustered_data, labels, cluster_entity);
    }

    OctoMap unlabeled_octonodes = this->pop_unlabeled_nodes(cluster_entity);

    //  クラスタリングで用いたデータを離散座標に変換する
    Eigen::MatrixXi clustered_vox_coords = this->w2vox_coords(clustered_data, this->_clustering_tree_depth, false);

    Eigen::VectorXi unique_labels = helper::calc_unique_matrixxi(labels);

    for (int i = 0; i < unique_labels.size(); i++)
    {
        int label = unique_labels(i);
        Eigen::VectorXi labels_idx = (labels.array() == label).cast<int>();
        labels_idx = helper::nonzero(labels_idx);
        Eigen::MatrixXi labeled_clustered_vox_coords = clustered_vox_coords(labels_idx, Eigen::all);

        OctoMap contains_octo_nodes;
        for (int j = 0; j < labeled_clustered_vox_coords.rows(); j++)
        {
            const auto cluster_vox =
                std::make_tuple(labeled_clustered_vox_coords(j, 0), labeled_clustered_vox_coords(j, 1),
                                labeled_clustered_vox_coords(j, 2));
            this->cluster_vox2deepest_vox.try_emplace(cluster_vox);
            for (const auto& deepest_vox : this->cluster_vox2deepest_vox.at(cluster_vox))
            {
                const auto& value = unlabeled_octonodes.at(deepest_vox);
                contains_octo_nodes.insert_or_assign(deepest_vox, value);
            }
        }
        // this->entity_octonodes.value().insert_or_assign(
        this->entity_octonodes.insert_or_assign(NodeClusterKey{cluster_entity, label}, contains_octo_nodes);
    }

    this->_clustering_tree_depth = std::nullopt;
    return *this;
}

/**
 * @brief entity_octonodesのfrom_keysのNodeEntity, cluster_idをto_keysのNodeEntity, cluster_idに置き換える,
 * 既にkeyが存在する場合は上書きする
 *
 * @param from_keys 置き換え前のNodeClusterKeyのリスト
 * @param to_keys 置き換え後のNodeClusterKeyのリスト
 * @return OctoTree&
 */
OctoTree&
OctoTree::replace_entities_in_octonodes(const NodeEntity from_entity,
                                        const std::map<std::optional<int>, NodeEntity>& cluster_transfered_entity)
{
    auto& map = this->entity_octonodes;

    for (auto& [cluster_id, target_entity] : cluster_transfered_entity)
    {
        auto extracted = map.extract(NodeClusterKey{from_entity, cluster_id});
        if (!extracted.empty())
        {
            NodeClusterKey new_key{target_entity, cluster_id};
            // 既にnew_keyに該当するkeyがentity_octonodesに存在する場合は消してから挿入
            if (map.find(new_key) != map.end())
            {
                map.erase(new_key);
            }

            extracted.key() = std::move(new_key);
            map.insert(std::move(extracted));
        }
    }
    return *this;
}

/**
 * @brief depthにおける最小、最大の離散座標を返す
 * @param depth
 * @return std::pair<VoxelCoord, VoxelCoord> 最小、最大の離散座標のpair
 */
std::pair<VoxelCoord, VoxelCoord> OctoTree::get_min_max_coord(std::optional<int> depth)
{
    int _depth = depth.has_value() ? depth.value() : this->max_tree_depth;
    int max_vox_val = static_cast<int>(std::pow(2, _depth));

    return {std::make_tuple(0, 0, 0), std::make_tuple(max_vox_val, max_vox_val, max_vox_val)};
}

/**
 * @brief lidar点群で、min_data, max_dataの外側の点群を取り除く
 * @param pcd_data 対象となるlidar点群
 * @param min/max_data 含める、最小と最大の範囲
 * @return Eigen::VectorXi 含める点群のindex
 */
Eigen::VectorXi OctoTree::limit_pcd_range(const Eigen::Ref<const Eigen::MatrixXd>& pcd_data,
                                          const Eigen::Ref<const Eigen::Vector3d>& min_data,
                                          const Eigen::Ref<const Eigen::Vector3d>& max_data)
{
    const auto& pcd_x = pcd_data(Eigen::all, 0);
    const auto& pcd_y = pcd_data(Eigen::all, 1);
    const auto& pcd_z = pcd_data(Eigen::all, 2);

    Eigen::VectorXi arry =
        ((pcd_x.array() >= min_data(0)) && (pcd_x.array() <= max_data(0)) && (pcd_y.array() >= min_data(1)) &&
         (pcd_y.array() <= max_data(1)) && (pcd_z.array() >= min_data(2)) && (pcd_z.array() <= max_data(2)))
            .cast<int>()
            .matrix();

    arry = helper::nonzero(arry);

    return arry;
};

/**
 * @brief 各離散座標における統計量を計算して、VoxStatsにして結果を返す
 * @param vox_xyz 離散座標
 * @param w_xyz 実座標
 * @return 離散座標に対するVoxStatsの辞書
 */
VoxelCoordMap<std::shared_ptr<VoxStats>>
OctoTree::calc_vox_statistics(const Eigen::Ref<const Eigen::MatrixXi>& vox_xyz,
                              const Eigen::Ref<const Eigen::MatrixXd>& w_xyz) const
{
    int n = vox_xyz.rows();
    int m = w_xyz.rows();
    assert(n == m);

    VoxelCoordMap<std::shared_ptr<VoxStats>> vox2stats;
    for (int i = 0; i < n; i++)
    {
        const auto key = std::make_tuple(vox_xyz(i, 0), vox_xyz(i, 1), vox_xyz(i, 2));
        Point w_coord = {w_xyz(i, 0), w_xyz(i, 1), w_xyz(i, 2), false};
        auto& stats_ptr = vox2stats[key];
        if (!stats_ptr)
        {
            // keyが存在しないので、vox_statsを生成
            stats_ptr = std::make_shared<VoxStats>();
        }
        stats_ptr->insert_points(w_coord);
    }
    return vox2stats;
}

/**
 * @brief 各離散座標のquantileを計算する
 * @param vox_xyz 離散座標
 * @param w_xyz 実座標
 * @param quantile 何パーセントの点を取るか
 * @param 各離散座標のquantileに該当する点の辞書
 */
VoxelCoordMap<std::tuple<float, float, float>>
OctoTree::calc_vox_quantile(const Eigen::Ref<const Eigen::MatrixXi>& vox_xyz,
                            const Eigen::Ref<const Eigen::MatrixXd>& w_xyz, float quantile) const
{
    int n = vox_xyz.rows();
    int m = w_xyz.rows();
    if (n != m)
    {
        throw std::invalid_argument("vox_xyzとw_xyzの行数は同じである必要があります. vox_xyz: " + std::to_string(n) +
                                    ", w_xyz: " + std::to_string(m) + ".");
    }

    if (quantile < 0 || 1 < quantile)
    {
        throw std::invalid_argument(
            "quantileは[0,1]の範囲で指定する必要があります., quantile = " + std::to_string(quantile) + ".");
    }

    // vox_coordをkey
    // (dist, w_coord)のペアのベクトルをvalueにしたmapを作る
    VoxelCoordMap<std::vector<std::pair<float, std::tuple<float, float, float>>>> vox2dist;
    // 各vox_coordに対して、距離とw_coordの組をvectorで紐づける
    for (int i = 0; i < n; i++)
    {
        const auto vox_coord = std::make_tuple(vox_xyz(i, 0), vox_xyz(i, 1), vox_xyz(i, 2));

        double x = w_xyz(i, 0);
        double y = w_xyz(i, 1);
        double z = w_xyz(i, 2);
        auto w_coord = std::make_tuple(x, y, z);
        double dist = x * x + y * y + z * z;

        if (vox2dist.find(vox_coord) == vox2dist.end())
        {
            // keyが存在しないので、vox_statsを生成
            vox2dist[vox_coord] = std::vector<std::pair<float, std::tuple<float, float, float>>>();
        }
        vox2dist[vox_coord].push_back(std::make_pair(dist, w_coord));
    }

    // 各vox_coordのvectorをdistでソートして、ソートした中で該当するindexのw_coordを取ってくる
    VoxelCoordMap<std::tuple<float, float, float>> vox2quantile;
    for (const auto& [key, vox_vec] : vox2dist)
    {
        // distでソート
        auto sorted_vox_vec = vox_vec;
        std::sort(sorted_vox_vec.begin(), sorted_vox_vec.end(),
                  [](const std::pair<double, std::tuple<double, double, double>>& a,
                     const std::pair<double, std::tuple<double, double, double>>& b) { return a.first < b.first; });

        // quantile番目の要素をvox2quantileに格納する
        size_t quantile_index = static_cast<size_t>(sorted_vox_vec.size() * quantile);
        if (quantile_index >= sorted_vox_vec.size())
        {
            quantile_index = sorted_vox_vec.size() - 1;
        }
        vox2quantile[key] = sorted_vox_vec[quantile_index].second;
    }
    return vox2quantile;
}
