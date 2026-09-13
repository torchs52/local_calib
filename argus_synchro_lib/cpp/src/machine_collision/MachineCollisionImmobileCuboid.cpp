#include "octotree/MachineCollisionImmobileCuboid.h"

MachineCollisionImmobileCuboid::MachineCollisionImmobileCuboid(
    MachineConf machine_info, const std::string& filename,
    const std::optional<std::tuple<double, double, double>>& initial_offsets,
    const std::optional<std::tuple<bool, bool, bool>>& reverse)
    : MachineCollisionBase(machine_info, filename, initial_offsets, reverse)
{
}

MachineCollisionImmobileCuboid::~MachineCollisionImmobileCuboid()
{
}

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
arrayXb MachineCollisionImmobileCuboid::check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                                                          const std::tuple<double, double, double>& remove_dist,
                                                          double roll_angle, double pitch_angle, double yaw_angle,
                                                          const std::optional<Eigen::MatrixXd>& transform_mat) const
{

    // machine_form_pointsから必要な情報を取り出す
    Eigen::Vector3d min_range = this->machine_form_points(0, Eigen::all);
    Eigen::Vector3d max_range = this->machine_form_points(1, Eigen::all);

    Eigen::MatrixXd target_xyz = xyz;

    // 直方体部分の機体除去判定,
    auto target_x = target_xyz(Eigen::all, 0);
    auto target_y = target_xyz(Eigen::all, 1);
    auto target_z = target_xyz(Eigen::all, 2);
    arrayXb remove_ind = ((target_x.array() >= (min_range(0) - std::get<0>(remove_dist)) &&
                           target_x.array() <= (max_range(0) + std::get<0>(remove_dist))) &&
                          (target_y.array() >= (min_range(1) - std::get<1>(remove_dist)) &&
                           target_y.array() <= (max_range(1) + std::get<1>(remove_dist))) &&
                          (target_z.array() >= (min_range(2) - std::get<2>(remove_dist)) &&
                           target_z.array() <= (max_range(2) + std::get<2>(remove_dist))));

    return remove_ind;
}
