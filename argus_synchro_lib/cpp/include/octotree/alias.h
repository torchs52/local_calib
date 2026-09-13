#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <tuple>
#include <optional>

// namespaceはいらない?
using VoxelCoord = std::tuple<int, int, int>; // type alias of tuple<int, int, int>

namespace edge_det
{
/**
 * @brief 崖検知で用いるaliasが入った部分
 *
 */

using WorldRangeOpt = std::optional<std::tuple<float, float>>; // LiDAR座標のxy座標を表すalias
using VoxelRangeOpt = std::optional<std::tuple<int, int>>;     // 画像格子座標のxy座標を表すalias

using PointXYZ = std::tuple<float, float, float>; // 実空間上のxyz座標を表現するalias
using PointXY = std::tuple<float, float>;         // 実空間上のxy座標を表現するalias
using PointPxPy = std::tuple<int, int>;           // 画像上のxy座標を表現するalias

using MatrixInt = Eigen::MatrixXi;    //整数行列に用いるalias
using MatrixDouble = Eigen::MatrixXd; // 実数行列に用いるalias

// opencvに渡すように用いる行列のalias
using MatrixU8RM = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using MatrixI8RM = Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
} // namespace edge_det
