#pragma once

#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <vector>

// 垂直方向（row方向）に連結、NumPy の vstack に相当
// NOTE vectorを渡すときは、.transposeしてから渡す。
// <3,1>の様になっているため、<1,3>に変更する。
template <typename DerivedA, typename DerivedB>
auto vstack(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B)
{
    // 列数が同じであることを確認
    assert(A.cols() == B.cols() && "vstack: Number of columns must be equal");
    // 結果行列のサイズ：行数は A.rows()+B.rows()、列数は A.cols()
    Eigen::Matrix<typename DerivedA::Scalar, Eigen::Dynamic, Eigen::Dynamic> result(A.rows() + B.rows(), A.cols());
    // 上半分に A を代入
    result.block(0, 0, A.rows(), A.cols()) = A;
    // 下半分に B を代入
    result.block(A.rows(), 0, B.rows(), B.cols()) = B;
    return result;
}

// 水平方向（column方向）に連結、NumPy の hstack に相当
template <typename DerivedA, typename DerivedB>
auto hstack(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B)
{
    // 行数が同じであることを確認
    assert(A.rows() == B.rows() && "hstack: Number of rows must be equal");
    // 結果行列のサイズ：行数は A.rows()、列数は A.cols() + B.cols()
    Eigen::Matrix<typename DerivedA::Scalar, Eigen::Dynamic, Eigen::Dynamic> result(A.rows(), A.cols() + B.cols());
    // 左側に A を代入
    result.block(0, 0, A.rows(), A.cols()) = A;
    // 右側に B を代入
    result.block(0, A.cols(), B.rows(), B.cols()) = B;
    return result;
}

// テンプレート関数：Eigen の行列（またはベクトル）を1次元
// Eigen::VectorXへ「平坦化」する。 関数内部では Eigen::Map
// を使用して、既存のメモリを参照する1次元ベクトルを生成します。
// 注意：元の行列が連続領域に確保されている前提です。
template <typename Derived> auto ravel(const Eigen::MatrixBase<Derived>& mat)
{
    using Scalar = typename Derived::Scalar;
    // Eigen::Map によって、mat.data() が指す連続領域を size()
    // 要素の1次元ベクトルとして参照する。
    return Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(mat.data(), mat.size());
}
