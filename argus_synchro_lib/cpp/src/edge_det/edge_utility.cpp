#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <opencv2/core.hpp>
#include <algorithm>
#include <numeric>
#include <iostream>

#include "cpp_helper_lib/eigen_operator.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoNode.h"
#include "octotree/OctoTree.h"
#include "octotree/transform.h"
#include "octotree/edge_utility.h"

namespace edge_det
{
/** LiDAR座標の範囲に対応する画素の座標を返す関数
 */
std::vector<VoxelRangeOpt> convert_ranges_to_vox(const std::vector<WorldRangeOpt>& w_ranges, OctoTree& octotree_obj,
                                                 std::optional<int> tree_depth)
{
    std::vector<VoxelRangeOpt> vox_ranges;
    vox_ranges.resize(w_ranges.size());

    for (size_t axis = 0; axis < w_ranges.size(); ++axis)
    {
        if (!w_ranges[axis])
        {
            vox_ranges[axis] = std::nullopt;
            continue;
        }

        auto [min_w, max_w] = *w_ranges[axis];
        Eigen::MatrixXd w_coords(2, 3);
        w_coords.setZero();
        w_coords(0, axis) = min_w;
        w_coords(1, axis) = max_w;

        Eigen::MatrixXi vox_coords = octotree_obj.w2vox_coords(w_coords, tree_depth, false);

        int min_v = vox_coords(0, axis);
        int max_v = vox_coords(1, axis);
        if (min_v > max_v)
            std::swap(min_v, max_v);
        // std::cout << "axis=" << axis << ", min_v=" << min_v << ", max_v=" << max_v << std::endl;

        vox_ranges[axis] = std::make_tuple(min_v, max_v);
    }

    return vox_ranges;
}

/** matをfrom_min, from_maxからto_min, to_maxの値の範囲にスケールを変える関数, to_minより小さい場合は、to_min,
 * to_maxより大きい場は、to_maxになる
 */
Eigen::MatrixXd scale_value(const Eigen::MatrixXd& mat, double from_min, double from_max, double to_min, double to_max)
{
    Eigen::MatrixXd res = mat; // コピーして作業

    double scale = (to_max - to_min) / (from_max - from_min);
    double intercept = to_min - scale * from_min;

    // 要素ごとに線形変換してから clip
    res = (res.array() * scale + intercept).matrix();
    res = res.array().min(to_max).max(to_min).matrix();

    return res;
}

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
MatrixInt create_polar_grid(const MatrixDouble& w_coords, PointXY grid_size, PointXY discrete_origin,
                            PointXY polar_origin)
{
    // デカルト座標を極座標に変換
    // w_coordsの時点で既に並進をしているので、
    MatrixDouble w_polar = cartesian_to_polar(w_coords, polar_origin);

    // 極座標を格子座標に変換
    return real_to_grid(w_polar, grid_size, discrete_origin);
}

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
                                                            std::optional<int> tree_depth, PointXYZ coord_origin)
{
    // vox_rangesの範囲で、target_entitiesに該当する点群を{離散座標: 八分木ノード}の辞書にする
    OctoMap target_octonodes =
        octotree_obj.collect_nodes_by_entities_with_depth(target_entities, vox_ranges, tree_depth);

    // 離散座標の行列に変換する
    Eigen::MatrixXi vox_coords(target_octonodes.size(), 3);
    int i = 0;
    for (const auto& [coord, node] : target_octonodes)
    {
        vox_coords(i, 0) = std::get<0>(coord);
        vox_coords(i, 1) = std::get<1>(coord);
        vox_coords(i, 2) = std::get<2>(coord);
        i++;
    }

    // 離散座標を実座標に変換する
    Eigen::MatrixXd w_coords = octotree_obj.vox2w_coords(vox_coords, tree_depth);

    // 実座標をcoord_originだけ並進する
    Eigen::Vector3d origin_vec(std::get<0>(coord_origin), std::get<1>(coord_origin), std::get<2>(coord_origin));
    MatrixDouble w_trans_coords = w_coords.array().rowwise() - origin_vec.array().transpose();
    return {w_trans_coords, vox_coords};
}

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
MatrixU8RM octotree2bev(OctoTree& octotree_obj, std::tuple<float, float> fwd_range, std::tuple<float, float> side_range,
                        PointXY grid_size, PointPxPy bev_shape, std::vector<NodeEntity> target_entities,
                        std::optional<int> bev_depth, BevCoord bev_coord, AggName agg_name, PointXYZ coord_origin,
                        PointXY discrete_origin, bool scaled, double min_scale_z, double max_scale_z,
                        double min_bev_val, double max_bev_val, unsigned char nan_fill_value)
{
    // fwd_range, side_rangeに応じて, 離散座標の最大, 最小値を計算する
    std::vector<VoxelRangeOpt> vox_ranges =
        convert_ranges_to_vox({fwd_range, side_range, std::nullopt}, octotree_obj, bev_depth);
    if (!vox_ranges[0].has_value() || !vox_ranges[1].has_value())
    {
        throw std::runtime_error("x or y range not available in vox_ranges");
    }

    // vox_rangesに応じたLiDAR座標, 離散座標の組を取得する
    auto [w_coords, vox_coords] =
        from_octotree_to_coords(octotree_obj, vox_ranges, target_entities, bev_depth, coord_origin);

    auto [x_min, x_max] = *vox_ranges[0];
    auto [y_min, y_max] = *vox_ranges[1];

    // 鳥観図の作り方に応じた離散点を生成する
    int x_size, y_size;
    int offset_for_mat_x, offset_for_mat_y;
    MatrixInt grid_xy;
    switch (bev_coord)
    {
    case BevCoord::CARTESIAN:
        grid_xy = vox_coords;
        // set bev ranges
        x_size = x_max - x_min + 1;
        y_size = y_max - y_min + 1;
        offset_for_mat_x = x_min;
        offset_for_mat_y = y_min;
        break;
    case BevCoord::POLAR:
        grid_xy = create_polar_grid(w_coords, grid_size, discrete_origin, {0.0, 0.0});
        x_size = std::get<0>(bev_shape);
        y_size = std::get<1>(bev_shape);
        offset_for_mat_x = 0;
        offset_for_mat_y = 0;
        break;
    default:
        throw std::runtime_error("bev_coord should be CARTESIAN or POLAR");
    }

    // 離散座標が同じz値を蓄積
    std::map<std::pair<int, int>, std::vector<double>> groups;
    for (int i = 0; i < grid_xy.rows(); i++)
    {
        int x = grid_xy(i, 0);
        int y = grid_xy(i, 1);

        double wz = w_coords(i, 2);
        groups[{x, y}].push_back(wz);
    }

    // z値の集約を行って鳥瞰図を作る
    Eigen::MatrixXd xy_stats = Eigen::MatrixXd::Constant(x_size, y_size, std::numeric_limits<double>::quiet_NaN());
    for (auto& [xy, zs] : groups)
    {
        int x = xy.first;
        int y = xy.second;

        double value = 0;
        switch (agg_name)
        {
        case AggName::MEAN:
            // std::cout << "sum=" << std::accumulate(zs.begin(), zs.end(), 0.0) << ", size=" << zs.size() << std::endl;
            value = std::accumulate(zs.begin(), zs.end(), 0.0) / zs.size();
            break;
        case AggName::MAX:
            // std::cout << "sum=" << std::accumulate(zs.begin(), zs.end(), 0.0) << ", size=" << zs.size() << std::endl;
            value = *std::max_element(zs.begin(), zs.end());
            break;
        case AggName::MIN:
            // std::cout << "sum=" << std::accumulate(zs.begin(), zs.end(), 0.0) << ", size=" << zs.size() << std::endl;
            value = *std::min_element(zs.begin(), zs.end());
            break;
        case AggName::LAST:
            // std::cout << "sum=" << std::accumulate(zs.begin(), zs.end(), 0.0) << ", size=" << zs.size() << std::endl;
            value = zs.back();
            break;
        }
        xy_stats(x - offset_for_mat_x, y - offset_for_mat_y) = value;
    }

    // スケーリングする
    Eigen::MatrixXd scaled_mat = xy_stats;
    if (scaled)
    {
        scaled_mat = scale_value(scaled_mat, min_scale_z, max_scale_z, min_bev_val, max_bev_val);
    }

    // nanを部分を埋めつつ、戻り値の形式で行列を作る
    MatrixU8RM img(x_size, y_size);
    for (int i = 0; i < x_size; ++i)
    {
        for (int j = 0; j < y_size; ++j)
        {
            double v = scaled_mat(i, j);
            if (std::isnan(v))
            {
                img(i, j) = nan_fill_value;
            }
            else
            {
                if (scaled)
                {
                    v = std::round(std::clamp(v, min_bev_val, max_bev_val));
                }
                img(i, j) = static_cast<unsigned char>(static_cast<int>(v));
            }
        }
    }

    // return helper::eigenToMatView(img);
    return img;
}

void mask_img_by_value(const cv::Mat& src, cv::Mat& dst, float low_value, float high_value)
{
    CV_Assert(src.type() == CV_8U || src.type() == CV_32F);

    // 出力は {-1,0,1} → signed int8
    dst.create(src.size(), CV_8SC1);

    const int rows = src.rows;
    const int cols = src.cols;

    if (src.type() == CV_8U)
    {
        for (int y = 0; y < rows; ++y)
        {
            const uint8_t* s = src.ptr<uint8_t>(y);
            int8_t* d = dst.ptr<int8_t>(y);

            for (int x = 0; x < cols; ++x)
            {
                const uint8_t v = s[x];
                d[x] = (v < low_value) ? -1 : (v > high_value) ? 1 : 0;
            }
        }
    }
    else
    {
        for (int y = 0; y < rows; ++y)
        {
            const float* s = src.ptr<float>(y);
            int8_t* d = dst.ptr<int8_t>(y);

            for (int x = 0; x < cols; ++x)
            {
                const float v = s[x];
                d[x] = (v < low_value) ? -1 : (v > high_value) ? 1 : 0;
            }
        }
    }
}

MatrixI8RM mask_img_by_value_py(const MatrixU8RM& src, float low_value, float high_value)
{
    cv::Mat cv_src = helper::eigenToMatView(src);
    cv::Mat cv_dst;
    mask_img_by_value(cv_src, cv_dst, low_value, high_value);
    return helper::matToEigenCopyI8(cv_dst);
}

} // namespace edge_det
