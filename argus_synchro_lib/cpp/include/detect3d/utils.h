#pragma once

#include <tuple>
#include <vector>
#include <Eigen/Core>

#include <iostream>
#include <functional>

// *** 定数定義 ***
constexpr Eigen::Index NO_SLICE = -1L;

// *** 関数プロトタイプ宣言 ***
template <typename Derived>
static inline Derived ResizeMatrix(const Eigen::MatrixBase<Derived>& matrix, Eigen::Index row, Eigen::Index col);

template <typename T> static inline Eigen::Vector<T, Eigen::Dynamic> StdVectorToEigenVector(std::vector<T>& vec);

// std::vectorをEigen::Matrixに変換
template <typename T>
static inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
StdVectorToEigenMatrix(const std::vector<std::vector<T>>& vec);

template <typename T>
static inline Eigen::VectorXi where(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& labels,
                                    const std::function<bool(T)>&& condition);

Eigen::VectorXi dbscan(const Eigen::Ref<const Eigen::MatrixXd>& matrix_pc, double eps, int min_samples);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
bounding_box(const Eigen::MatrixXd& mtx_pc, const Eigen::VectorXi& unique_labels, const Eigen::VectorXi& labels);

// inline関数定義

template <typename Derived>
inline Derived ResizeMatrix(const Eigen::MatrixBase<Derived>& matrix, Eigen::Index row, Eigen::Index col)
{
    auto mat = matrix.derived();
    const Eigen::Index row_base = mat.rows(); // スライス前の行数
    const Eigen::Index col_base = mat.cols(); // スライス前の列数

    // row,colに負数を指定された場合と、元の行数より大きい値を指定された行/列はスライスしない。
    const Eigen::Index row_slice = (row < 0) || (row > row_base) ? row_base : row; // スライス後の行数
    const Eigen::Index col_slice = (col < 0) || (col > col_base) ? col_base : col; // スライス後の列数

    if ((row_slice != row_base) || (col_slice != col_base))
    {
        return mat.block(0, 0, row_slice, col_slice);
    }
    else
    {
        return mat;
    }
}

template <typename T> static inline Eigen::Vector<T, Eigen::Dynamic> StdVectorToEigenVector(std::vector<T>& vec)
{
    if (vec.empty())
    {
        return Eigen::Vector<T, Eigen::Dynamic>().setZero();
    }

    Eigen::Vector<T, Eigen::Dynamic> e_vec = Eigen::Map<Eigen::Vector<T, Eigen::Dynamic>>(&vec[0], vec.size());

    return e_vec;
}

template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> StdVectorToEigenMatrix(const std::vector<std::vector<T>>& vec)
{
    using matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    if (vec.empty() || vec[0].empty())
    {
        return matrix().setZero(); // 空行列を返す
    }

    // Vectorのサイズ確認
    auto rows = vec.size();
    auto cols = vec[0].size();

    // 2次元のvectorを1次元のvector展開しなおす
    std::vector<T> t;
    t.reserve(rows * cols);
    for (auto i : vec)
    {
        t.insert(t.end(), i.begin(), i.end());
    }

    matrix mat = Eigen::Map<matrix>(&t[0], cols, rows).transpose();

    return mat;
}

template <typename T>
inline Eigen::VectorXi where(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& labels,
                             const std::function<bool(T)>&& condition)
{
    std::vector<int> indices_v;

    for (int i = 0; i < labels.size(); ++i)
    {
        if (condition(labels(i)))
        {
            indices_v.emplace_back(i);
        }
    }

    return StdVectorToEigenVector(indices_v);
}
