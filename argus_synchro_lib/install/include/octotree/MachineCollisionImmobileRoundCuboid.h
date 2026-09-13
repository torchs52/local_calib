#pragma once

#include "MachineCollisionBase.h"

#include <optional>
#include <vector>
class MachineCollisionImmobileRoundCuboid : public MachineCollisionBase
{
    /**
     * @brief 直方体+円弧柱で構成される非可動部位に対する機体点群除去を行うクラス
     *
     */
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 直方体部分の形状パラメータ
    Eigen::MatrixXd cuboid_form_points;

    // 円弧柱部分の形状パラメータ
    Eigen::MatrixXd round_form_points;

    // 円弧柱の高さ範囲
    std::tuple<double, double> round_pillar_zrange;

    // 円弧柱の最小x座標
    double round_pillar_min_x;

    // 円弧柱の各中心座標
    std::vector<std::tuple<double, double>> round_pillar_centers;

    // 円弧柱の各半径
    std::vector<double> round_pillar_radiuses;

    /**
     * @brief Construct a new Machine Collision Immobile Round Cuboid:: Machine Collision Immobile Round Cuboid object
     * @details
     * コンストラクタの引数から円弧柱の中心を求めたり、機体点群除去のファイルに入っている情報を分けたりして、check_pcd_on_selfが実行できる状態にする
     * @remark remark text
     * machine_form_pointsを入れているcsvに、色々な情報を持つ座標が入っていて、可読性が悪いので、HexaPrismと同じようにjsonファイルで情報を管理したほうが追い
     *
     * @param machine_info
     * @param filename
     * @param initial_offsets
     * @param reverse
     */
    MachineCollisionImmobileRoundCuboid(
        MachineConf machine_info, const std::string& filename,
        const std::optional<std::tuple<double, double, double>>& initial_offsets = std::nullopt,
        const std::optional<std::tuple<bool, bool, bool>>& reverse = std::nullopt);

    ~MachineCollisionImmobileRoundCuboid();

    /**
     * @brief 直方体+円弧柱の非可動な機体部位に対して、近い点群を見つける
     *
     * @param xyz 点群 (n,3)行列
     * @param remove_dist 除去範囲(x,y,z)
     * @param roll_angle roll角
     * @param pitch_angle pitch角
     * @param yaw_angle yaw角
     * @param transform_mat 変換行列
     * @return arrayXb
     */
    arrayXb check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                              const std::tuple<double, double, double>& remove_dist = std::make_tuple(0.11, 0.11, 0.11),
                              double roll_angle = 0, double pitch_angle = 0, double yaw_angle = 0,
                              const std::optional<Eigen::MatrixXd>& transform_mat = std::nullopt) const override;
};
