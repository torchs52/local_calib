#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

using arrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;

class helper
{
  public:
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> filter_rows_by_range(
        const Eigen::MatrixBase<Derived>& mat,
        const std::vector<std::optional<std::tuple<typename Derived::Scalar, typename Derived::Scalar>>>& ranges);
    static Eigen::VectorXi calc_unique_vectorxi(const Eigen::Ref<const Eigen::VectorXi>& vec);

    static Eigen::MatrixXi calc_unique_matrixxi(const Eigen::Ref<const Eigen::MatrixXi>& vec);

    static Eigen::VectorXi nonzero(const Eigen::Ref<const Eigen::VectorXi>& vec);
    static Eigen::VectorXi nonzero(const Eigen::Ref<const arrayXb>& arr);
    static Eigen::MatrixXd cdist(const Eigen::Ref<const Eigen::MatrixXd>& XA,
                                 const Eigen::Ref<const Eigen::MatrixXd>& XB);
    static std::tuple<double, int, int> calc_cdist_min(const Eigen::Ref<const Eigen::MatrixXd>& XA,
                                                       const Eigen::Ref<const Eigen::MatrixXd>& XB);

    /**
     * @brief Eigen::Matrixとcv::Matの変換関数
     *
     * @param m
     * @return cv::Mat
     */
    static cv::Mat
    eigenToMatView(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& m);

    /**
     * @brief cv::MatとEigen::Mapの変換関数, mat側の要素がunsigned charの場合に使う
     * @details
     * viewを返しているのでコピーは発生しないらしいが、ライフタイムの管理が怪しくて、Python側に渡す場合は、メモリコピーする関数を準備したほうが良いらしい
     * @todo 今のところ, unsigned charとint8_tの行列しか想定していないが、増えてきたら修正する
     *
     * @param m
     * @return Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
     */
    static Eigen::Map<const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    matToEigenView(const cv::Mat& m);

    /**
     * @brief cv::MatとEigen::Mapの変換関数, mat側の要素がint8_tの場合に使う
     *
     * @param m
     * @return Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
     */
    static Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
    matToEigenViewI8(const cv::Mat& m);

    /**
     * @brief matをメモリコピーしてEigen::Matrixを作る関数, コピーしたい行列の要素がunsigned charの場合に使う
     * @details cv::Matで処理をしていて、最終的にPython側に渡す場合は、これを通せばPython側でnumpy.NDArrayとして扱える
     *
     * @param mat
     * @return Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
     */
    static Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    matToEigenCopy(const cv::Mat& mat);

    /**
     * @brief matをメモリコピーしてEigen::Matrixを作る関数, コピーしたい行列の要素がint8_tの場合に使う
     * @details cv::Matで処理をしていて、最終的にPython側に渡す場合は、これを通せばPython側でnumpy.NDArrayとして扱える
     *
     * @param mat
     * @return Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
     */
    static Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    matToEigenCopyI8(const cv::Mat& mat); // Eigen::Matrix(Vector)を文字列化
    template <typename Derived>
    static inline std::string EigenMatrixToString(const Eigen::MatrixBase<Derived>& matrix, bool is_compact);

    // Eigen::Vectorを文字列化
    template <typename Derived> static inline std::string EigenVectorToString(const Eigen::MatrixBase<Derived>& matrix);
};

// Eigen::Matrix(Vector)を文字列化
template <typename Derived>
inline std::string helper::EigenMatrixToString(const Eigen::MatrixBase<Derived>& matrix, bool is_compact)
{
    auto mtx = matrix.derived();
    if (mtx.rows() < 1 || mtx.cols() < 1)
        return "Empty Matrix";

    auto rows = mtx.rows();
    bool is_skip = (rows > 10) && is_compact;

    std::string msg = "[\n";
    for (Eigen::Index i = 0; i < rows; ++i)
    {
        if (is_skip && i >= 3)
        {
            is_skip = false;
            i = rows - 3;
            msg.append("\t... ,\n");
        }
        msg.append("\t[");
        for (Eigen::Index j = 0; j < mtx.cols(); ++j)
        {
            msg.append(std::to_string(mtx(i, j))).append(", ");
        }
        msg.append("],\n");
    }

    msg.erase(msg.size() - 2, 2);
    return msg + "]";
}

// Eigen::Vectorを文字列化
template <typename Derived> inline std::string helper::EigenVectorToString(const Eigen::MatrixBase<Derived>& matrix)
{
    auto mtx = matrix.derived();
    if (mtx.rows() < 1 || mtx.cols() < 1)
    {
        return "Empty Vector";
    }

    if (mtx.cols() != 1)
    {
        return "Matrix";
    }

    std::string msg = "[";
    for (Eigen::Index i = 0; i < mtx.rows(); ++i)
    {
        msg.append(std::to_string(mtx(i, 0))).append(", ");
    }
    msg.erase(msg.size() - 2, 2);
    return msg + "]";
}
