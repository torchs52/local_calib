
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "octotree/transform.h"

namespace edge_det
{

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
MatrixDouble grid_to_real(const MatrixInt& grid_coords_2d, PointXY grid_size, PointPxPy grid_offset,
                          PointXY real_offset, DediscretizeMethod repr_method)
{
    auto [grid_offset_x, grid_offset_y] = grid_offset;
    auto [grid_size_x, grid_size_y] = grid_size;
    auto [real_offset_x, real_offset_y] = real_offset;
    // 離散化の条件に応じてオフセットを与える
    switch (repr_method)
    {
    case DediscretizeMethod::MED:
        grid_offset_x += 0.5;
        grid_offset_y += 0.5;
    case DediscretizeMethod::MIN:
        grid_offset_x += 1.0;
        grid_offset_y += 1.0;
    }
    MatrixDouble real_coords_2d(grid_coords_2d.rows(), 2);
    for (int i = 0; i < grid_coords_2d.rows(); i++)
    {
        real_coords_2d(i, 0) = (grid_coords_2d(i, 0) + grid_offset_x) * grid_size_x + real_offset_x;
        real_coords_2d(i, 1) = (grid_coords_2d(i, 1) + grid_offset_y) * grid_size_y + real_offset_y;
    }
    return real_coords_2d;
}

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
MatrixInt real_to_grid(const MatrixDouble& real_coords_2d, PointXY grid_size, PointXY real_offset,
                       PointPxPy grid_offset)
{
    auto [grid_size_x, grid_size_y] = grid_size;
    auto [real_offset_x, real_offset_y] = real_offset;
    auto [grid_offset_x, grid_offset_y] = grid_offset;
    MatrixInt grid_coords_2d(real_coords_2d.rows(), 2);
    for (int i = 0; i < real_coords_2d.rows(); i++)
    {
        grid_coords_2d(i, 0) = std::floor((real_coords_2d(i, 0) - real_offset_x) / grid_size_x) - grid_offset_x;
        grid_coords_2d(i, 1) = std::floor((real_coords_2d(i, 1) - real_offset_y) / grid_size_y) - grid_offset_y;
    }

    return grid_coords_2d;
}

/**
 * @brief 極座標をデカルト座標に変換する
 * @details (radius * cos(theta), radius * sin(theta)) + from_polar_originを返す
 * @param real_polar_coords 極座標 (n, 2)行列, (動径方向, 角度方向)の順で入っている
 * @param from_polar_origin 極座標の中心座標
 * @return MatrixDouble
 */
MatrixDouble polar_to_cartesian(const MatrixDouble& real_polar_coords, PointXY from_polar_origin)
{
    MatrixDouble real_cartesian_coords(real_polar_coords.rows(), 2);
    for (int i = 0; i < real_polar_coords.rows(); i++)
    {
        double radius = real_polar_coords(i, 0);
        double theta = real_polar_coords(i, 1);
        real_cartesian_coords(i, 0) = radius * std::cos(theta) + std::get<0>(from_polar_origin);
        real_cartesian_coords(i, 1) = radius * std::sin(theta) + std::get<1>(from_polar_origin);
    }

    return real_cartesian_coords;
}

/**
 * @brief デカルト座標上の点を極座標に変換する関数
 * @details polar_origin分並進した(x,y)に対して(sqrt(x^2+y^2), atan2(y,x))を返す
 * @param points デカルト座標の点
 * @param polar_origin 極座標の原点
 * @return std::tuple<Eigen::MatrixXd Eigen::MatrixXd> 極座標の各座標での点, (動径方向, 角度方向)のtuple
 */
MatrixDouble cartesian_to_polar(const MatrixDouble& points, PointXY polar_origin)
{
    MatrixDouble polar(points.rows(), 2);
    for (int i = 0; i < points.rows(); i++)
    {
        // polar_origin中心で、距離と角度を計算
        double x = points(i, 0) - std::get<0>(polar_origin);
        double y = points(i, 1) - std::get<1>(polar_origin);
        polar(i, 0) = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        if (y == 0.0 && x == 0.0)
        {
            // 原点は角度0とする
            polar(i, 1) = 0;
        }
        else
        {
            polar(i, 1) = std::atan2(y, x);
        }
    }
    return polar;
}
} // namespace edge_det
