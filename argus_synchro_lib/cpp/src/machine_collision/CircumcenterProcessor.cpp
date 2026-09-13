#include "octotree/CircumcenterProcessor.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <map>
#include <set>

CircumcenterProcessor::CircumcenterProcessor() : center(std::nullopt), radius(std::nullopt)
{
}

/**
 * @brief 与えられたpointsに対する外心計算
 *
 * @param points
 * @return CircumcenterProcessor&
 */
CircumcenterProcessor& CircumcenterProcessor::fit(const Eigen::Ref<const Eigen::MatrixXd>& points)
{
    std::map<std::tuple<std::tuple<double, double>, std::tuple<double, double>, std::tuple<double, double>>,
             std::tuple<double, double, Eigen::MatrixXd, double>>
        res;

    // pointsから3点選んで外心計算を行い、他の点の当てはまりが良いかどうかを評価する
    for (int p0 = 0; p0 < points.rows(); p0++)
    {
        for (int p1 = p0; p1 < points.rows(); p1++)
        {
            if (p1 == p0)
                continue;
            for (int p2 = p1; p2 < points.rows(); p2++)
            {
                if (p2 == p1 || p2 == p1)
                    continue;

                Eigen::MatrixXd est_center;
                double est_radius;
                std::tie(est_center, est_radius) =
                    this->_calc_circumcenter(points(p0, Eigen::all), points(p1, Eigen::all), points(p2, Eigen::all));
                Eigen::VectorXd res_quantities = this->score(points, est_center, est_radius);

                auto res_point = std::make_tuple(std::make_tuple(points(p0, 0), points(p0, 1)),
                                                 std::make_tuple(points(p1, 0), points(p1, 1)),
                                                 std::make_tuple(points(p2, 0), points(p2, 1)));

                auto res_quantities_mean = res_quantities.mean();
                auto res_quantities_std =
                    sqrt((res_quantities.array() - res_quantities_mean).square().sum() / (res_quantities.size() - 1));
                res.insert_or_assign(res_point,
                                     std::make_tuple(res_quantities_mean, res_quantities_std, est_center, est_radius));
            }
        }
    }

    double chosen_value = std::get<0>(res.begin()->second);
    std::tuple<std::tuple<double, double>, std::tuple<double, double>, std::tuple<double, double>> chosen_key =
        res.begin()->first;
    for (const auto& [key, value] : res)
    {
        if (std::get<0>(value) < chosen_value)
        {
            chosen_key = key;
            chosen_value = std::get<0>(value);
        }
    }

    // 良さげな点の結果をインスタンス変数に入れる
    this->center = std::get<2>(res.at(chosen_key));
    this->radius = std::get<3>(res.at(chosen_key));
    this->points << std::get<0>(std::get<0>(chosen_key)), std::get<1>(std::get<0>(chosen_key)),
        std::get<0>(std::get<1>(chosen_key)), std::get<1>(std::get<1>(chosen_key)),
        std::get<0>(std::get<2>(chosen_key)), std::get<1>(std::get<2>(chosen_key));
    return *this;
}

/**
 * @brief スコア計算を行う
 * @details |radius - sqrt(sum((points - center)**2))|の計算を行っている
 *
 * @param points
 * @param center
 * @param radius
 * @return Eigen::Vector3d
 */
Eigen::Vector3d CircumcenterProcessor::score(const Eigen::Ref<const Eigen::MatrixXd>& points,
                                             const Eigen::Ref<const Eigen::VectorXd>& center, double radius)
{
    return (radius - (points.array().rowwise() - center.transpose().array())
                         .square()
                         .matrix()
                         .array()
                         .rowwise()
                         .sum()
                         .matrix()
                         .array()
                         .sqrt())
        .abs();
}

/**
 * @brief p0, p1, p2を用いて外心計算を行う
 *
 * @param p0
 * @param p1
 * @param p2
 * @return std::tuple<Eigen::MatrixXd, double>
 */
std::tuple<Eigen::MatrixXd, double>
CircumcenterProcessor::_calc_circumcenter(const Eigen::Ref<const Eigen::VectorXd>& p0,
                                          const Eigen::Ref<const Eigen::VectorXd>& p1,
                                          const Eigen::Ref<const Eigen::VectorXd>& p2)
{
    Eigen::VectorXd p01 = p1 - p0;
    Eigen::VectorXd p02 = p2 - p0;

    double bc = p01.dot(p02);
    double bb = p01.dot(p01);
    double cc = p02.dot(p02);

    Eigen::MatrixXd weight_mat(2, 2);
    weight_mat << bb, bc, bc, cc;

    Eigen::Vector2d est_weight = weight_mat.llt().solve(Eigen::Vector2d(bb, cc) / 2.0);

    Eigen::VectorXd est_center = est_weight(0) * p01 + est_weight(1) * p02 + p0;

    double est_radius = sqrt((est_center - p0).array().square().sum());

    return std::make_tuple(est_center, est_radius);
}
