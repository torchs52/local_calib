#pragma once

#include "MachineCollisionBase.h"
#include <memory>

namespace machine_rm
{
/**
 * @brief 機体点群除去に用いるクラスを入れる名前空間
 * @todo 他の機体点群除去関連の処理もこの名前空間に入れｒる
 *
 */

using t_min_max = std::tuple<double, double>;
using t_xyz = std::tuple<double, double, double>;
using t_dyn_real_mat = Eigen::MatrixXd;

class BaseRange
{
    /** MachineCollisionBaseの中でさらに構造を持つ場合に用いる抽象クラス
     */
  public:
    virtual ~BaseRange() = default;

    /**
     * @brief BaseRangeに対する除外判定を行う, 継承先で振る舞いは規定
     *
     * @param xyz 点群
     * @param remove_dist 除外する長さ
     * @return arrayXb xyzと同じ長さの外(n,)ベクトル, 除外される点群であればtrue
     */
    virtual arrayXb check_pcd(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                              std::tuple<double, double, double> remove_dist = std::make_tuple(0.12, 0.12, 0.04)) = 0;
};

class CuboidRange : public BaseRange
{
    /** 六角柱の直方体部分の情報を保持するクラス
     */
  public:
    t_min_max x_minmax; // (x座標の最小, 最大)が入っているtuple
    t_min_max y_minmax; // (y座標の最小, 最大)が入っているtuple
    t_min_max z_minmax; // (z座標の最小, 最大)が入っているtuple

    t_min_max x_range_ratio; // (最小側を何倍延ばすか, 最大側を何倍延ばすか)が入っているtuple
    t_min_max y_range_ratio; // (最小側を何倍延ばすか, 最大側を何倍延ばすか)が入っているtuple
    t_min_max z_range_ratio; // (最小側を何倍延ばすか, 最大側を何倍延ばすか)が入っているtuple

    CuboidRange(const t_min_max& x_minmax, const t_min_max& y_minmax, const t_min_max& z_minmax,
                const t_min_max& x_range_ratio = std::make_tuple(-1.0, 1.0),
                const t_min_max& y_range_ratio = std::make_tuple(-1.0, 1.0),
                const t_min_max& z_range_ratio = std::make_tuple(-1.0, 1.0));

    ~CuboidRange();

    /**
     * @brief 直方体周辺の除去対象となる点群を見つける
     * @details 直方体に位置によってはある軸方向は判定しなかったりするので、そういった調整をxyz_range_ratioで行う
     * @param xyz
     * @param remove_dist
     * @return arrayXb
     */
    arrayXb check_pcd(const Eigen::Ref<const t_dyn_real_mat>& xyz,
                      t_xyz remove_dist = std::make_tuple(0.12, 0.12, 0.04)) override;
};

class TriPillar : public BaseRange
{
    /** 六角柱の三角柱部分の情報を保持するクラス
     */
  public:
    t_min_max z_minmax;
    std::vector<std::tuple<double, double>> tri_points;
    t_min_max x_remove_offset_ratio;
    t_min_max y_remove_offset_ratio;
    t_min_max z_remove_offset_ratio;
    bool vec_is_reverse;

    TriPillar(const t_min_max& z_minmax, const std::vector<std::tuple<double, double>>& tri_points,
              const t_min_max& x_remove_offset_ratio, const t_min_max& y_remove_offset_ratio,
              const t_min_max& z_remove_offset_ratio, bool vec_is_reverse = false);

    ~TriPillar();

    /**
     * @brief 三角柱の形状に応じて除去対象の点群を見つける
     *
     * @param xyz
     * @param remove_dist
     * @return arrayXb
     */
    arrayXb check_pcd(const Eigen::Ref<const t_dyn_real_mat>& xyz,
                      t_xyz remove_dist = std::make_tuple(0.12, 0.12, 0.04)) override;

  private:
    /**
     * @brief 三角形の内側を判定する
     * @details 境界との符号付き距離を法線ベクトルで求めて、閾値で判定している
     *
     * @param xyz
     * @param normal_vec_2d
     * @param x_range
     * @param y_range
     * @param z_range
     * @param point_on_line
     * @return arrayXb
     */
    arrayXb check_pcd_side(const Eigen::Ref<const t_dyn_real_mat>& xyz,
                           const Eigen::Ref<const Eigen::Vector2d>& normal_vec_2d, const t_min_max& x_range,
                           const t_min_max& y_range, const t_min_max& z_range,
                           const Eigen::Ref<const Eigen::Vector2d>& point_on_line);
};

class MachineCollisionImmobileHexaPrism : public MachineCollisionBase
{
    /**
     * @brief 上部旋回体上の六角柱部分の機体除去処理を行うクラス
     * @details
     * 六角柱はそれをさらに区分けにしたhex_base_ranges単位で判定を行う構造になっているので、hex_base_rangesを作って、作ったhex_base_rangesを基に機体周辺の点か判定を行う
     *
     */
  public:
    std::vector<std::shared_ptr<BaseRange>> hex_base_ranges;

    MachineCollisionImmobileHexaPrism(
        MachineConf machine_info, const std::string& filename,
        const std::optional<std::tuple<double, double, double>>& initial_offsets = std::nullopt,
        const std::optional<std::tuple<bool, bool, bool>>& reverse = std::nullopt,
        const std::string& machine_form_extension = ".jsonc");

    ~MachineCollisionImmobileHexaPrism();

    /**
     * @brief hex_base_rangesの各部位で内側判定を行う
     *
     * @param xyz
     * @param remove_dist
     * @param roll_angle
     * @param pitch_angle
     * @param yaw_angle
     * @param transform_mat
     * @return arrayXb
     */
    arrayXb check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                              const std::tuple<double, double, double>& remove_dist = std::make_tuple(0.11, 0.11, 0.11),
                              double roll_angle = 0, double pitch_angle = 0, double yaw_angle = 0,
                              const std::optional<Eigen::MatrixXd>& transform_mat = std::nullopt) const override;
};

/**
 * @brief xyzの各行についてx_range, y_range, z_rangeの範囲に入っているか判定する
 * @details x,y,z_rangeがnulloptの場合は飛ばして、andを取った結果を返す
 *
 * @param xyz 点群(n,3)行列
 * @param x_range xの最小最大, nullの場合は飛ばす
 * @param y_range yの最小最大, nullの場合は飛ばす
 * @param z_range zの最小最大, nullの場合は飛ばす
 * @return arrayXb
 */
arrayXb is_in_interval(const Eigen::Ref<const t_dyn_real_mat>& xyz,
                       const std::optional<t_min_max>& x_range = std::nullopt,
                       const std::optional<t_min_max>& y_range = std::nullopt,
                       const std::optional<t_min_max>& z_range = std::nullopt);
} // namespace machine_rm
