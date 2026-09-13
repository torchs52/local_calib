#pragma once

#include "MachineConf.h"

#include <Eigen/Core>
#include <optional>

/**
 * @brief 外心計算を行うクラス
 * fitで複数の点から外心を計算して
 * 結果はcenterというインスタンス変数に入る
 * 外心計算に必要なのは3点だけだったので、複数の点の中から3点を選んで、全体に対して最も当てはまりの良いものを使う処理を行っている
 */
class CircumcenterProcessor
{
  public:
    std::optional<Eigen::VectorXd> center; // 外心の中心座標(2次元ベクトル)
    std::optional<double> radius;
    Eigen::Matrix<double, 6, 1> points; // 選ばれた3点(3*2次元)

    CircumcenterProcessor();

    /**
     * @brief pointsに対して当てはまりの良い3点を選ぶ
     *
     * @param points
     * @return CircumcenterProcessor&
     */
    CircumcenterProcessor& fit(const Eigen::Ref<const Eigen::MatrixXd>& points);

    /**
     * @brief pointsがcenter, radiusの外心にどれだけ乗っているか評価する
     *
     * @param points
     * @param center
     * @param radius
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d score(const Eigen::Ref<const Eigen::MatrixXd>& points,
                          const Eigen::Ref<const Eigen::VectorXd>& center, double radius);

    /**
     * @brief p0, p1, p2を用いて外心計算
     *
     * @param p0
     * @param p1
     * @param p2
     * @return std::tuple<Eigen::MatrixXd, double>
     */
    std::tuple<Eigen::MatrixXd, double> _calc_circumcenter(const Eigen::Ref<const Eigen::VectorXd>& p0,
                                                           const Eigen::Ref<const Eigen::VectorXd>& p1,
                                                           const Eigen::Ref<const Eigen::VectorXd>& p2);
};
