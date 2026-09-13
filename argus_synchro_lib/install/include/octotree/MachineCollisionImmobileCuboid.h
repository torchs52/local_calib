#pragma once

#include "MachineCollisionBase.h"
#include <optional>

class MachineCollisionImmobileCuboid : public MachineCollisionBase
{
    /**
     * @brief 機体部位で機体点群除去を直方体近似で行う部位で、旋回で回転しない部分を表現するクラス
     *
     */
  public:
    MachineCollisionImmobileCuboid(
        MachineConf machine_info, const std::string& filename,
        const std::optional<std::tuple<double, double, double>>& initial_offsets = std::nullopt,
        const std::optional<std::tuple<bool, bool, bool>>& reverse = std::nullopt);

    ~MachineCollisionImmobileCuboid();

    /**
     * @brief 非可動部かつ直方体形状の周りの点群除去
     * @details machine_form_pointsの1行目が最小値のxyz, 2行目が最大値のxyzとして、情報を取り出した後で必要な判定を行う
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
