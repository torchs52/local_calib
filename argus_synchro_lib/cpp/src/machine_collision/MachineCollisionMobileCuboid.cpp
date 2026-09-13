#include "octotree/MachineCollisionMobileCuboid.h"
#include "octotree/controller.h"
#include "octotree/machine_collision.h"

MachineCollisionMobileCuboid::MachineCollisionMobileCuboid(
    MachineConf machine_info, const std::string& filename,
    const std::optional<std::tuple<double, double, double>>& initial_offsets,
    const std::optional<std::tuple<bool, bool, bool>>& reverse)
    : MachineCollisionBase(machine_info, filename, initial_offsets, reverse)
{
}

MachineCollisionMobileCuboid::~MachineCollisionMobileCuboid()
{
}

/**
 * @brief roll, pitch, yawだけ回転させたときの行列を求めるヘルパー関数
 * @details xyzの順で回転
 *
 * @param roll_rad_angle roll角
 * @param pitch_rad_angle pitch角
 * @param yaw_rad_angle yaw角
 * @return Matrix3d
 */
Matrix3d MachineCollisionMobileCuboid::_rotate(double roll_rad_angle, double pitch_rad_angle,
                                               double yaw_rad_angle) const
{
    return rotate_x(roll_rad_angle) * rotate_y(pitch_rad_angle) * rotate_z(yaw_rad_angle);
}

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
arrayXb MachineCollisionMobileCuboid::check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                                                        const std::tuple<double, double, double>& remove_dist,
                                                        double roll_angle, double pitch_angle, double yaw_angle,
                                                        const std::optional<Eigen::MatrixXd>& transform_mat) const
{
    // 形状点から直方体の最小と最大を取得
    Eigen::Vector3d min_range = this->machine_form_points(0, Eigen::all);
    Eigen::Vector3d max_range = this->machine_form_points(1, Eigen::all);
    Eigen::Vector3d offsets(0, 0, 0);

    // 回転と並進を計算
    Matrix3d _rotate_mat;
    Eigen::Vector3d _trans_vec;
    if (transform_mat == std::nullopt)
    {
        // Todo: 並進は今後必要になるかもしれないが、情報がないので、一旦0にしている
        _rotate_mat = this->_rotate(roll_angle, pitch_angle, yaw_angle);
        _trans_vec = offsets;
    }
    else
    {
        _rotate_mat = transform_mat.value()(Eigen::seq(0, 3, 1), Eigen::seq(0, 3, 1));
        _trans_vec = transform_mat.value()(Eigen::seq(0, 3, 1), Eigen::last);
    }
    // trans_vecを中心として回転, 回転後元の座標に戻すため再度trans_vecのオフセットを掛けている
    Eigen::MatrixXd rot_xyz =
        ((xyz.array().rowwise() - _trans_vec.transpose().array()).matrix() * _rotate_mat.transpose())
            .array()
            .rowwise() +
        _trans_vec.transpose().array();
    Eigen::VectorXd rot_x = rot_xyz(Eigen::all, 0);
    Eigen::VectorXd rot_y = rot_xyz(Eigen::all, 1);
    Eigen::VectorXd rot_z = rot_xyz(Eigen::all, 2);

    arrayXb remove_ind = ((rot_x.array() >= (min_range(0) - std::get<0>(remove_dist)) &&
                           rot_x.array() <= (max_range(0) + std::get<0>(remove_dist))) &&
                          (rot_y.array() >= (min_range(1) - std::get<1>(remove_dist)) &&
                           rot_y.array() <= (max_range(1) + std::get<1>(remove_dist))) &&
                          (rot_z.array() >= (min_range(2) - std::get<2>(remove_dist)) &&
                           rot_z.array() <= (max_range(2) + std::get<2>(remove_dist))));

    return remove_ind;
}
