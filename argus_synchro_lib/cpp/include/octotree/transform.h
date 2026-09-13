#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>
#include <optional>

#include "alias.h"

namespace edge_det
{
/**
 * @brief 崖検知関係の変換処理に関わる関数
 *
 */

enum class DediscretizeMethod
{
    /**
     * @brief 格子座標をどのように実空間に写すかの方法を列挙した型
     *
     */
    MED, // 格子の中心に対応する座標
    MIN, // 格子の最小のxyに対応する座標
    MAX  // 格子の最大のxyに対応する座標
};

/**
 * @brief 格子座標を2次元実座標に変換する
 * @details (x + grid_offset) * grid_size + real_offsetを返す
 * @param grid_coords_2d 格子座標(n, 2)行列
 * @param grid_size 格子サイズ
 * @param grid_offset 格子のオフセット
 * @param real_offset 実数のオフセット
 * @param repr_method 格子のどの点を実数点として使うか
 * @return MatrixDouble 実数上の座標(n, 2)行列
 */
MatrixDouble grid_to_real(const MatrixInt& grid_coords_2d, PointXY grid_size, PointPxPy grid_offset = {0, 0},
                          PointXY real_offset = {0.0, 0.0}, DediscretizeMethod repr_method = DediscretizeMethod::MED);

/**
 * @brief 2次元実座標を格子座標に変換する
 * @details floor(x-real_offset) / grid_size - grid_offsetを返す
 * @param real_coords_2d 2次元実座標(n,2)行列
 * @param grid_size 実空間上の格子の幅
 * @param real_offset 実空間のオフセット
 * @param grid_offset 格子のオフセット
 * @return MatrixInt 格子座標(n, 2)行列
 *
 */
MatrixInt real_to_grid(const MatrixDouble& real_coords_2d, PointXY grid_size, PointXY real_offset = {0.0, 0.0},
                       PointPxPy grid_offset = {0, 0});

/**
 * @brief 極座標をデカルト座標に変換する
 * @details (radius * cos(theta), radius * sin(theta)) + from_polar_originを返す
 * @param real_polar_coords 極座標 (n, 2)行列, (動径方向, 角度方向)の順で入っている
 * @param from_polar_origin 極座標の中心座標
 * @return MatrixDouble
 */
MatrixDouble polar_to_cartesian(const MatrixDouble& real_polar_coords, PointXY from_polar_origin);

/**
 * @brief デカルト座標上の点を極座標に変換する関数
 * @details polar_origin分並進した(x,y)に対して(sqrt(x^2+y^2), atan2(y,x))を返す
 * @param points デカルト座標の点
 * @param polar_origin 極座標の原点
 * @return std::tuple<Eigen::MatrixXd Eigen::MatrixXd> 極座標の各座標での点, (動径方向, 角度方向)のtuple
 */
MatrixDouble cartesian_to_polar(const MatrixDouble& points, PointXY polar_origin);
} // namespace edge_det
