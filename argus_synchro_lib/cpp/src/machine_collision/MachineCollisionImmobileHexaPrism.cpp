#include "octotree/MachineCollisionImmobileHexaPrism.h"
#include "octotree/machine_collision.h"

namespace machine_rm
{

CuboidRange::CuboidRange(const t_min_max& x_minmax, const t_min_max& y_minmax, const t_min_max& z_minmax,
                         const t_min_max& x_range_ratio, const t_min_max& y_range_ratio, const t_min_max& z_range_ratio)
    : x_minmax(x_minmax), y_minmax(y_minmax), z_minmax(z_minmax), x_range_ratio(x_range_ratio),
      y_range_ratio(y_range_ratio), z_range_ratio(z_range_ratio)
{
}

CuboidRange::~CuboidRange()
{
}

/**
 * @brief 直方体周辺の除去対象となる点群を見つける
 * @details 直方体に位置によってはある軸方向は判定しなかったりするので、そういった調整をxyz_range_ratioで行う
 * @param xyz
 * @param remove_dist
 * @return arrayXb
 */
arrayXb CuboidRange::check_pcd(const Eigen::Ref<const t_dyn_real_mat>& xyz, t_xyz remove_dist)
{
    auto [x_min, x_max] = this->x_minmax;
    auto [y_min, y_max] = this->y_minmax;
    auto [z_min, z_max] = this->z_minmax;

    auto [x_range_from_ratio, x_range_to_ratio] = this->x_range_ratio;
    auto [y_range_from_ratio, y_range_to_ratio] = this->y_range_ratio;
    auto [z_range_from_ratio, z_range_to_ratio] = this->z_range_ratio;

    auto [remove_dist_x, remove_dist_y, remove_dist_z] = remove_dist;

    auto target_x = xyz.col(0).array();
    arrayXb cond_x = ((x_min + x_range_from_ratio * remove_dist_x) <= target_x) *
                     (target_x <= (x_max + x_range_to_ratio * remove_dist_x));

    auto target_y = xyz.col(1).array();
    arrayXb cond_y = ((y_min + y_range_from_ratio * remove_dist_y) <= target_y) *
                     (target_y <= (y_max + y_range_to_ratio * remove_dist_y));

    auto target_z = xyz.col(2).array();
    arrayXb cond_z = ((z_min + z_range_from_ratio * remove_dist_z) <= target_z) *
                     (target_z <= (z_max + z_range_to_ratio * remove_dist_z));

    arrayXb remove_ind = cond_x * cond_y * cond_z;
    return remove_ind;
}

TriPillar::TriPillar(const t_min_max& z_minmax, const std::vector<std::tuple<double, double>>& tri_points,
                     const t_min_max& x_remove_offset_ratio, const t_min_max& y_remove_offset_ratio,
                     const t_min_max& z_remove_offset_ratio, bool vec_is_reverse)
    : z_minmax(z_minmax), tri_points(tri_points), x_remove_offset_ratio(x_remove_offset_ratio),
      y_remove_offset_ratio(y_remove_offset_ratio), z_remove_offset_ratio(z_remove_offset_ratio),
      vec_is_reverse(vec_is_reverse)
{
}

TriPillar::~TriPillar()
{
}

/**
 * @brief 三角柱の形状に応じて除去対象の点群を見つける
 *
 * @param xyz
 * @param remove_dist
 * @return arrayXb
 */
arrayXb TriPillar::check_pcd(const Eigen::Ref<const t_dyn_real_mat>& xyz, t_xyz remove_dist)
{
    if (this->tri_points.size() < 2)
    {
        throw std::logic_error("tri_points should be larger than 2");
    }
    // 三角形の端点にあたる(x,y)座標を取り出す
    auto [p1x, p1y] = this->tri_points[0];
    auto [p2x, p2y] = this->tri_points[1];

    auto [x_remove_offset_from_ratio, x_remove_offset_to_ratio] = this->x_remove_offset_ratio;
    auto [y_remove_offset_from_ratio, y_remove_offset_to_ratio] = this->y_remove_offset_ratio;
    auto [z_remove_offset_from_ratio, z_remove_offset_to_ratio] = this->z_remove_offset_ratio;
    auto [z_min, z_max] = this->z_minmax;
    auto [remove_dist_x, remove_dist_y, remove_dist_z] = remove_dist;

    // 端点をremove_dist分だけ伸ばす
    Eigen::Vector2d p1(p1x, p1y);
    Eigen::Vector2d p1_offset(x_remove_offset_from_ratio * remove_dist_x, y_remove_offset_from_ratio * remove_dist_y);
    Eigen::Vector2d p2(p2x, p2y);
    Eigen::Vector2d p2_offset(x_remove_offset_to_ratio * remove_dist_x, y_remove_offset_to_ratio * remove_dist_y);

    Eigen::Vector2d trans_p1 = p1 + p1_offset;
    Eigen::Vector2d trans_p2 = p2 + p2_offset;

    t_min_max target_x =
        (trans_p1[0] < trans_p2[0]) ? t_min_max{trans_p1[0], trans_p2[0]} : t_min_max{trans_p2[0], trans_p1[0]};
    t_min_max target_y =
        (trans_p1[1] < trans_p2[1]) ? t_min_max{trans_p1[1], trans_p2[1]} : t_min_max{trans_p2[1], trans_p1[1]};
    t_min_max target_z{z_min + z_remove_offset_from_ratio * remove_dist_z,
                       z_max + z_remove_offset_to_ratio * remove_dist_z};

    // 三角形の端点から法線ベクトルを作る
    Eigen::Vector2d p12 = p1 - p2;
    Eigen::Vector2d remove_line_vec(-p12[1], p12[0]);
    Eigen::Vector2d remove_line_normal = remove_line_vec.normalized();
    if (this->vec_is_reverse)
    {
        remove_line_normal = -1 * remove_line_normal;
    }

    // 境界の内側にあたる点をcheck_pcd_sideで判定する
    arrayXb remove_inds = this->check_pcd_side(xyz, remove_line_normal, target_x, target_y, target_z, trans_p1);

    return remove_inds;
}

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
arrayXb TriPillar::check_pcd_side(const Eigen::Ref<const t_dyn_real_mat>& xyz,
                                  const Eigen::Ref<const Eigen::Vector2d>& normal_vec_2d, const t_min_max& x_range,
                                  const t_min_max& y_range, const t_min_max& z_range,
                                  const Eigen::Ref<const Eigen::Vector2d>& point_on_line)
{
    Eigen::MatrixXd xy = xyz.leftCols(2);
    Eigen::VectorXd signed_dist = (xy.rowwise() - point_on_line.transpose()) * normal_vec_2d;
    arrayXb inside_line_inds = (signed_dist.array() < 0.0);

    arrayXb inside_cuboid_inds = is_in_interval(xyz, x_range, y_range, z_range);

    arrayXb remove_inds = inside_line_inds && inside_cuboid_inds;

    return remove_inds;
}

MachineCollisionImmobileHexaPrism::MachineCollisionImmobileHexaPrism(
    MachineConf machine_info, const std::string& filename,
    const std::optional<std::tuple<double, double, double>>& initial_offsets,
    const std::optional<std::tuple<bool, bool, bool>>& reverse, const std::string& machine_form_extension)
    : MachineCollisionBase(machine_info, filename, initial_offsets, reverse, machine_form_extension)
{
    this->hex_base_ranges =
        load_base_ranges_from_json(machine_info.get_form_points_filename(filename, machine_form_extension));
}

MachineCollisionImmobileHexaPrism::~MachineCollisionImmobileHexaPrism()
{
}

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
arrayXb MachineCollisionImmobileHexaPrism::check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                                                             const std::tuple<double, double, double>& remove_dist,
                                                             double roll_angle, double pitch_angle, double yaw_angle,
                                                             const std::optional<Eigen::MatrixXd>& transform_mat) const
{
    arrayXb cond = arrayXb::Constant(xyz.rows(), false);
    if (this->hex_base_ranges.size() == 0)
    {
        return cond;
    }

    for (auto hex_base_range : this->hex_base_ranges)
    {
        // 各パーツに対して条件判定を行ってどこかでtrueの場合trueにする
        cond = cond || hex_base_range->check_pcd(xyz, remove_dist);
    }

    return cond;
}

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
arrayXb is_in_interval(const Eigen::Ref<const t_dyn_real_mat>& xyz, const std::optional<t_min_max>& x_range,
                       const std::optional<t_min_max>& y_range, const std::optional<t_min_max>& z_range)
{
    if (xyz.size() == 0)
    {
        return arrayXb();
    }

    arrayXb cond = arrayXb::Constant(xyz.rows(), true);

    if (x_range.has_value())
    {
        auto [min_val, max_val] = *x_range;
        auto target_col = xyz.col(0).array();
        cond = cond && ((min_val < target_col) && (target_col < max_val));
    }

    if (y_range.has_value())
    {
        auto [min_val, max_val] = *y_range;
        auto target_col = xyz.col(1).array();
        cond = cond && ((min_val < target_col) && (target_col < max_val));
    }

    if (z_range.has_value())
    {
        auto [min_val, max_val] = *z_range;
        auto target_col = xyz.col(2).array();
        cond = cond && ((min_val < target_col) && (target_col < max_val));
    }
    return cond;
}
} // namespace machine_rm
