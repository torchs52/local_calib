#include "detect3d/utils.h"

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <open3d/Open3D.h>
#include <vector>

// static関数宣言
template <typename T> static inline void VectorExtend(std::vector<T>& base, const std::vector<T>& extention);

template <typename T> inline void VectorExtend(std::vector<T>& base, const std::vector<T>& extention)
{
    base.insert(base.end(), extention.begin(), extention.end());
}

Eigen::VectorXi dbscan(const Eigen::Ref<const Eigen::MatrixXd>& matrix_pc, double eps, int min_samples)
{
    // Eigen::MatrixXd->PointCloud
    std::vector<Eigen::Vector3d> points;
    points.reserve(matrix_pc.rows());
    for (auto i = 0; i < matrix_pc.rows(); ++i)
    {
        points.emplace_back(matrix_pc.row(i).transpose());
    }
    open3d::geometry::PointCloud pcd(points);

    // DBSCAN実施
    std::vector<int> db = pcd.ClusterDBSCAN(eps, min_samples, false);
    Eigen::VectorXi labels = Eigen::Map<Eigen::VectorXi>(db.data(), db.size());

    return labels;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
bounding_box(const Eigen::MatrixXd& mtx_pc, const Eigen::VectorXi& unique_labels, const Eigen::VectorXi& labels)
{
    std::vector<std::vector<double>> multi_minmax;
    std::vector<std::vector<double>> multi_points;
    std::vector<std::vector<double>> multi_lines;

    for (auto idx : unique_labels)
    {
        if (idx == -1)
            continue;
        if (idx > labels.size())
            break;

        // # クラスタごとにインデックスを取得
        auto indices = where<int>(labels, [idx](int label) { return label == idx; });

        // # 取得したインデックスで点群を抜き出し
        Eigen::MatrixXd extracted_pc(indices.size(), mtx_pc.cols());
        for (Eigen::Index i = 0; i < indices.size(); ++i)
        {
            extracted_pc.row(i) = mtx_pc.row(indices(i));
        }

        // # 抜き出した点群から最小最大値の点を見つける
        Eigen::MatrixXd extracted_pc_x = extracted_pc.block(0, 0, extracted_pc.rows(), 1);
        auto x_max = extracted_pc_x.maxCoeff();
        auto x_min = extracted_pc_x.minCoeff();
        Eigen::MatrixXd extracted_pc_y = extracted_pc.block(0, 1, extracted_pc.rows(), 1);
        auto y_max = extracted_pc_y.maxCoeff();
        auto y_min = extracted_pc_y.minCoeff();
        Eigen::MatrixXd extracted_pc_z = extracted_pc.block(0, 2, extracted_pc.rows(), 1);
        auto z_max = extracted_pc_z.maxCoeff();
        auto z_min = extracted_pc_z.minCoeff();

        std::vector<double> minmax = {x_min, x_max, y_min, y_max, z_min, z_max};

        // # ８個のポイントを設定
        std::vector<std::vector<double>> points = {
            {x_min, y_min, z_min}, {x_max, y_min, z_min}, {x_min, y_max, z_min}, {x_max, y_max, z_min},
            {x_min, y_min, z_max}, {x_max, y_min, z_max}, {x_min, y_max, z_max}, {x_max, y_max, z_max},
        };

        std::vector<std::vector<double>> lines = {
            {0.0 + 8 * idx, 1.0 + 8 * idx}, {0.0 + 8 * idx, 2.0 + 8 * idx}, {1.0 + 8 * idx, 3.0 + 8 * idx},
            {2.0 + 8 * idx, 3.0 + 8 * idx}, {4.0 + 8 * idx, 5.0 + 8 * idx}, {4.0 + 8 * idx, 6.0 + 8 * idx},
            {5.0 + 8 * idx, 7.0 + 8 * idx}, {6.0 + 8 * idx, 7.0 + 8 * idx}, {0.0 + 8 * idx, 4.0 + 8 * idx},
            {1.0 + 8 * idx, 5.0 + 8 * idx}, {2.0 + 8 * idx, 6.0 + 8 * idx}, {3.0 + 8 * idx, 7.0 + 8 * idx},
        };

        multi_minmax.emplace_back(minmax);
        VectorExtend(multi_lines, lines);
        VectorExtend(multi_points, points);
    }

    Eigen::MatrixXd multi_points_mtx; // # 立体物 8隅座標
    Eigen::MatrixXd multi_lines_mtx;  // # 立体物 ラインデータ
    Eigen::MatrixXd multi_minmax_mtx; // # 立体物 minmax for each xyz

    if ((unique_labels.size() == 1) && (unique_labels(0) == -1))
    {
        multi_points_mtx = Eigen::MatrixXd::Zero(8 * 1, 3);
        multi_lines_mtx = Eigen::MatrixXd::Zero(12 * 1, 2);
        multi_minmax_mtx = Eigen::MatrixXd::Zero(1 * 1, 6);
    }
    else
    {
        multi_points_mtx = StdVectorToEigenMatrix(multi_points);
        multi_lines_mtx = StdVectorToEigenMatrix(multi_lines);
        multi_minmax_mtx = StdVectorToEigenMatrix(multi_minmax);
    }

    return {multi_points_mtx, multi_lines_mtx, multi_minmax_mtx};
}
