#pragma once

#include "MachineCollisionBase.h"
#include <optional>

class MachineCollisionMobileCuboid : public MachineCollisionBase
{
    /**
     * @brief 機体で旋回と共に動く部分を直方体で近似するクラス
     *
     */
  public:
    MachineCollisionMobileCuboid(
        MachineConf machine_info, const std::string& filename,
        const std::optional<std::tuple<double, double, double>>& initial_offsets = std::nullopt,
        const std::optional<std::tuple<bool, bool, bool>>& reverse = std::nullopt);

    ~MachineCollisionMobileCuboid();

    /**
     * @brief roll, pitch, yawだけ回転させたときの行列を求めるヘルパー関数
     * @details xyzの順で回転
     *
     * @param roll_rad_angle roll角
     * @param pitch_rad_angle pitch角
     * @param yaw_rad_angle yaw角
     * @return Matrix3d
     */
    Matrix3d _rotate(double roll_rad_angle, double pitch_rad_angle, double yaw_rad_angle) const;

    /**
     * @brief 旋回を考慮して、機体点群除去を行う
     * @details
     * 点群を旋回角だけ回転させた後で、回転した点群が機体の立方体をremove_distだけ膨らませた範囲に入っているかどうか判定して、入っているものを除去対象にする
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
