#include "cpp_helper_lib/eigen_operator.h"
#include <nanoflann.hpp>

#include <vector>
#include <limits>
#include <cmath>

#include <stdexcept>
#include <tuple>

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> helper::filter_rows_by_range(
    const Eigen::MatrixBase<Derived>& mat,
    const std::vector<std::optional<std::tuple<typename Derived::Scalar, typename Derived::Scalar>>>& ranges)
{
    using Scalar = typename Derived::Scalar;
    using MatrixT = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    if (ranges.size() != static_cast<size_t>(mat.cols()))
    {
        throw std::invalid_argument("ranges size must match number of columns of mat");
    }

    std::vector<Eigen::RowVector<Scalar, Eigen::Dynamic>> valid_rows;
    for (int i = 0; i < mat.rows(); ++i)
    {
        bool ok = true;
        for (int j = 0; j < mat.cols(); ++j)
        {
            Scalar val = mat(i, j);
            if (!ranges[j])
                continue;
            auto [min_val, max_val] = *ranges[j];
            if (val < min_val || val > max_val)
            {
                ok = false;
                break;
            }
        }
        if (ok)
        {
            valid_rows.push_back(mat.row(i));
        }
    }

    MatrixT filtered(valid_rows.size(), mat.cols());
    for (size_t i = 0; i < valid_rows.size(); ++i)
    {
        filtered.row(i) = valid_rows[i];
    }

    return filtered;
}

Eigen::VectorXi helper::calc_unique_vectorxi(const Eigen::Ref<const Eigen::VectorXi>& vec)
{
    std::set<int> vecset;
    for (int i = 0; i < vec.size(); i++)
    {
        vecset.insert(vec(i));
    }
    Eigen::VectorXi result(vecset.size());

    auto ite = vecset.begin();
    for (int i = 0; i < result.size(); i++)
    {
        result(i) = *ite;
        ite++;
    }
    return result;
}
Eigen::MatrixXi helper::calc_unique_matrixxi(const Eigen::Ref<const Eigen::MatrixXi>& vec)
{
    std::set<std::tuple<int, int, int>> vecset;
    for (int i = 0; i < vec.rows(); i++)
    {
        vecset.insert({vec(i, 0), vec(i, 1), vec(i, 2)});
    }
    Eigen::MatrixXi result(vecset.size(), 3);

    auto ite = vecset.begin();
    for (int i = 0; i < result.rows(); i++)
    {
        result(i, 0) = std::get<0>(*ite);
        result(i, 1) = std::get<1>(*ite);
        result(i, 2) = std::get<2>(*ite);
        ite++;
    }
    return result;
}

/**Vectorで1となっている箇所のインデックスを返す。
 */
Eigen::VectorXi helper::nonzero(const Eigen::Ref<const Eigen::VectorXi>& vec)
{
    Eigen::VectorXi ret_vec(vec.count());
    int x = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        if (vec(i) == 1)
        {
            ret_vec(x) = i;
            x++;
        }
    }

    return ret_vec;
}
/**arrayでTrueとなっている箇所のインデックスを返す。
 */
Eigen::VectorXi helper::nonzero(const Eigen::Ref<const arrayXb>& arr)
{
    Eigen::VectorXi ret_vec(arr.count());
    int x = 0;
    for (int i = 0; i < arr.size(); i++)
    {
        if (arr(i) == true)
        {
            ret_vec(x) = i;
            x++;
        }
    }

    return ret_vec;
}

/*** cdist
 * WARNING 点数が増えるととても遅くなる可能性がある(1frame 数100ms)
 *
 * */
Eigen::MatrixXd helper::cdist(const Eigen::Ref<const Eigen::MatrixXd>& XA, const Eigen::Ref<const Eigen::MatrixXd>& XB)
{
    Eigen::MatrixXd Y(XA.rows(), XB.rows());
    for (int row = 0; row < XA.rows(); row++)
    {
        for (int col = 0; col < XB.rows(); col++)
        {
            Y(row, col) = ((XA.row(row) - XB.row(col))).norm();
        }
    }
    return Y;
}

/*** calc_cdist_min
 * cdistの最小距離とインデックスを返す。
 * cdistと比較すると、行列を作成しないため、こちらの方速い場合がある。
 *
 * */
std::tuple<double, int, int> helper::calc_cdist_min(const Eigen::Ref<const Eigen::MatrixXd>& XA,
                                                    const Eigen::Ref<const Eigen::MatrixXd>& XB)
{
    // 固定値（迷ったら10でOK）
    constexpr int leaf_max_size = 10;

    using RowMatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    RowMatXd xb_rm(XB.rows(), XB.cols());
    xb_rm.noalias() = XB;

    using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<RowMatXd, 3>;
    KDTree kdtree(3, xb_rm, leaf_max_size);
    kdtree.index_->buildIndex();

    double min_dist = std::numeric_limits<double>::infinity();
    int min_iA = -1;
    int min_iB = -1;

    for (Eigen::Index iA = 0; iA < XA.rows(); ++iA)
    {
        double q[3] = {XA(iA, 0), XA(iA, 1), XA(iA, 2)};
        KDTree::IndexType iB = 0;
        double nn_dist_sq = 0.0; // nanoflannは距離²を返す

        kdtree.query(q, /*num_closest=*/1, &iB, &nn_dist_sq);

        if (nn_dist_sq < min_dist)
        {
            min_dist = nn_dist_sq;
            min_iA = static_cast<int>(iA);
            min_iB = static_cast<int>(iB);
        }
    }

    return {std::sqrt(min_dist), min_iA, min_iB};
}

cv::Mat helper::eigenToMatView(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& m)
{
    return cv::Mat(m.rows(), m.cols(), CV_8UC1, const_cast<uint8_t*>(m.data())
                   // m.cols() * sizeof(uint8_t)
    );
}

Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
helper::matToEigenView(const cv::Mat& mat)
{

    CV_Assert(mat.type() == CV_8UC1);
    CV_Assert(mat.isContinuous());

    return Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        mat.data, mat.rows, mat.cols);
}

Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
helper::matToEigenViewI8(const cv::Mat& mat)
{

    CV_Assert(mat.type() == CV_8SC1);
    CV_Assert(mat.isContinuous());

    return Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        reinterpret_cast<const int8_t*>(mat.data), mat.rows, mat.cols);
}

Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> helper::matToEigenCopy(const cv::Mat& mat)
{
    CV_Assert(mat.type() == CV_8UC1);
    CV_Assert(mat.isContinuous());

    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> out(mat.rows, mat.cols);

    std::memcpy(out.data(), mat.data, mat.total() * mat.elemSize());

    return out;
}

Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> helper::matToEigenCopyI8(const cv::Mat& mat)
{
    CV_Assert(mat.type() == CV_8SC1);
    CV_Assert(mat.isContinuous());

    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> out(mat.rows, mat.cols);

    std::memcpy(out.data(), mat.data, mat.total() * mat.elemSize());

    return out;
}
