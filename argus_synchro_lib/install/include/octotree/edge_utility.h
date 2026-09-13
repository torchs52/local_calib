#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/core.hpp>

#include <numbers>
#include <cmath>
#include <vector>
#include <optional>
#include <tuple>
#include "alias.h"
#include "OctoTree.h"
#include "NodeEntity.h"

namespace edge_det
{
/** 崖検出に用いるものを入れておく名前空間
 */

const double PI = std::acos(-1.0); // std::numbers::piが呼べなかったので

enum class BevCoord
{
    /** 鳥観図の作るかの列挙
     */
    CARTESIAN, // デカルト座標で鳥瞰図を作る場合に用いる
    POLAR      // 極座標で鳥瞰図を作る場合に用いる
};

// name of aggregation
enum class AggName
{
    /** 鳥観図の各画素値をどのように設定するかの列挙
     */
    MEAN, // 格子内のz値の平均値
    MAX,  // 格子内のz値の最大値
    MIN,  // 格子内のz値の最小値
    LAST  // 格子内のz値の最後に格納されていた値
};

/** LiDAR座標の範囲に対応する画素の座標を返す関数
 */
std::vector<VoxelRangeOpt> convert_ranges_to_vox(const std::vector<WorldRangeOpt>& w_ranges,
                                                 const OctoTree& octotree_obj,
                                                 std::optional<int> tree_depth = std::nullopt);

/** matをfrom_min, from_maxからto_min, to_maxの値の範囲にスケールを変える関数, to_minより小さい場合は、to_min,
 * to_maxより大きい場は、to_maxになる
 */
Eigen::MatrixXd scale_value(const Eigen::MatrixXd& mat, double from_min, double from_max, double to_min = 0.0,
                            double to_max = 255.0);

/**
 * @brief LiDAR座標から極座標における格子座標を計算する関数
 * @details 極座標に変換された座標p(polar_origin)=(radius, angle)はこの関数で
 * floor((p - discrete_origin) / grid_size)に変換される
 *
 * @param w_coords LiDAR座標
 * @param grid_size 極座標の格子の幅
 * @param discrete_origin 極座標の画素値0をどこにするかを表すオフセット
 * @param polar_origin 極座標の原点
 * @return MatrixInt
 */
MatrixInt create_polar_grid(const MatrixDouble& w_coords, PointXY grid_size, PointXY discrete_origin = {0.0, -1.0 * PI},
                            PointXY polar_origin = {0.0, 0.0});

/**
 * @brief target_entitiesに入っている点群をvox_rangesの範囲で取り出して、離散座標と実座標を返す関数
 *
 * @param octotree_obj 八分木インスタンス
 * @param vox_ranges 取り出すデータの範囲
 * @param target_entities 取り出すNodeEntityのリスト
 * @param tree_depth 取り出す階層
 * @param coord_origin 実座標の原点をどこにするか
 * @return std::tuple<MatrixDouble, MatrixInt> 実座標と離散座標の組
 */
std::tuple<MatrixDouble, MatrixInt> from_octotree_to_coords(OctoTree& octotree_obj,
                                                            std::vector<VoxelRangeOpt> vox_ranges,
                                                            std::vector<NodeEntity> target_entities,
                                                            std::optional<int> tree_depth, PointXYZ coord_origin);

/**
 * @brief 八分木のtarget_entities, fwd_range, side_rangeの範囲で鳥瞰図を作成する関数
 *
 * @param octotree_obj
 * @param fwd_range LiDAR座標のx軸の範囲 (min, max)のtuple
 * @param side_range LiDAR座標のy軸の範囲 (min, max)のtuple
 * @param grid_size 鳥観図の実座標の格子の幅(x軸, y軸)のtuple
 * @param bev_shape 鳥観図の格子数(x軸, y軸)のtuple
 * @param target_entities 鳥観図を作るNodeEntityのリスト
 * @param bev_depth 鳥観図を作る八分木の階層, nullの場合八分木の一番下の階層
 * @param bev_coord 鳥観図で用いる軸, デカルト座標(CARESIAN)か、極座標(POLAR)を選べる
 * @param agg_name 同じ格子内の点の集約方法, MAXにすれば同じ格子内の最大のz値をその格子の画素値とする
 * @param coord_origin LiDAR座標のどの点を鳥瞰図の原点とするか
 * @param discrete_origin 格子座標にした際にどの点を原点とするか
 * @param scaled 画素値の最大・最小を定めるか
 * @param min_scale_z 最小のz値, この値より小さいz値はmin_scale_zになる
 * @param max_scale_z 最大のz値, この値より大きいz値はmax_scale_zになる
 * @param min_bev_val 鳥観図の最小画素値
 * @param max_bev_val 鳥観図の最大画素値
 * @param nan_fill_value 何も値がない部分をどの値で埋めるか
 * @return MatrixU8RM 型はalias.hに載っているもの, opencvの入力を想定した型にしている
 */
// MatrixU8RM
MatrixU8RM octotree2bev(OctoTree& octotree_obj, std::tuple<float, float> fwd_range, std::tuple<float, float> side_range,
                        PointXY grid_size, PointPxPy bev_shape, std::vector<NodeEntity> target_entities,
                        std::optional<int> bev_depth = std::nullopt, BevCoord bev_coord = BevCoord::POLAR,
                        AggName agg_name = AggName::MAX, PointXYZ coord_origin = {0.0, 0.0, 0.0},
                        PointXY discrete_origin = {0.0, -1.0 * PI}, bool scaled = true, double min_scale_z = -1.88,
                        double max_scale_z = -0.88, double min_bev_val = 0.0, double max_bev_val = 255.0,
                        unsigned char nan_fill_value = 0);

void mask_img_by_value(const cv::Mat& src, cv::Mat& dst, float low_value = 50.0f, float high_value = 200.0f);

MatrixI8RM mask_img_by_value_py(const MatrixU8RM& src, float low_value = 50.0f, float high_value = 200.0f);
} // namespace edge_det
