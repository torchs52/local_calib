#include "octotree/MachineCollisionImmobileRoundCuboid.h"
#include "octotree/CircumcenterProcessor.h"

#include <algorithm>

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
MachineCollisionImmobileRoundCuboid::MachineCollisionImmobileRoundCuboid(
    MachineConf machine_info, const std::string& filename,
    const std::optional<std::tuple<double, double, double>>& initial_offsets,
    const std::optional<std::tuple<bool, bool, bool>>& reverse)
    : MachineCollisionBase(machine_info, filename, initial_offsets, reverse)
{
    // 直方体部分と円柱部分で使う情報を分ける
    this->cuboid_form_points = this->machine_form_points(Eigen::seq(0, 2, 1), Eigen::all);

    this->round_form_points = this->machine_form_points(Eigen::seq(2, Eigen::last, 1), Eigen::all);

    // 円弧柱の高さを取り出す
    this->round_pillar_zrange = std::make_tuple(this->round_form_points(Eigen::all, 2).minCoeff(),
                                                this->round_form_points(Eigen::all, 2).maxCoeff());

    // 円弧柱のxの最小値を決める
    this->round_pillar_min_x = this->round_form_points(Eigen::all, 0).minCoeff();

    // 各円弧柱の中心と半径を取り出す
    std::vector<double> offsets;
    for (int i = 0; i < this->round_form_points.rows(); i += 6)
    {
        offsets.push_back(i);
    }
    std::vector<std::tuple<double, double>> round_pillar_centers;
    std::vector<double> round_pillar_radiuses;
    for (const auto& offset : offsets)
    {
        Eigen::MatrixXd selected_round_form_points =
            this->round_form_points(Eigen::seq(offset, offset + 2, 1), Eigen::seq(0, 1, 1));
        auto clf = CircumcenterProcessor().fit(selected_round_form_points);

        round_pillar_centers.push_back({clf.center.value()(0), clf.center.value()(1)});

        round_pillar_radiuses.push_back(clf.radius.value());
    }
    this->round_pillar_centers = round_pillar_centers;
    this->round_pillar_radiuses = round_pillar_radiuses;
}

MachineCollisionImmobileRoundCuboid::~MachineCollisionImmobileRoundCuboid()
{
}

/**
 * @brief 直方体+円弧柱の非可動な機体部位に対して、近い点群を見つける
 *
 * @param xyz
 * @param remove_dist
 * @param roll_angle
 * @param pitch_angle
 * @param yaw_angle
 * @param transform_mat
 * @return arrayXb
 */
arrayXb MachineCollisionImmobileRoundCuboid::check_pcd_on_self(
    const Eigen::Ref<const Eigen::MatrixXd>& xyz, const std::tuple<double, double, double>& remove_dist,
    double roll_angle, double pitch_angle, double yaw_angle, const std::optional<Eigen::MatrixXd>& transform_mat) const
{
    Eigen::Vector3d min_range = this->machine_form_points(0, Eigen::all);
    Eigen::Vector3d max_range = this->machine_form_points(1, Eigen::all);
    Eigen::Vector3d offsets(-3.5, 0.05, 0);

    Eigen::MatrixXd target_xyz = xyz; // - offsets

    // 直方体部分の機体除去判定,
    // xの正の方向に、円弧柱があるので、正側は膨らませない
    auto target_x = target_xyz(Eigen::all, 0);
    auto target_y = target_xyz(Eigen::all, 1);
    auto target_z = target_xyz(Eigen::all, 2);
    arrayXb cuboid_remove_ind = ((target_x.array() >= (min_range(0) - std::get<0>(remove_dist)) &&
                                  target_x.array() <= (max_range(0) + std::get<0>(remove_dist))) &&
                                 (target_y.array() >= (min_range(1) - std::get<1>(remove_dist)) &&
                                  target_y.array() <= (max_range(1) + std::get<1>(remove_dist))) &&
                                 (target_z.array() >= (min_range(2) - std::get<2>(remove_dist)) &&
                                  target_z.array() <= (max_range(2) + std::get<2>(remove_dist))));
    // 円弧部分の機体除去判定
    // 円弧のz,xの条件判定を行う
    arrayXb round_pillar_remove_ind =
        (xyz(Eigen::all, 0).array() >= this->round_pillar_min_x) &&
        (target_z.array() >= (std::get<0>(this->round_pillar_zrange) - std::get<2>(remove_dist)) &&
         (target_z.array() <= (std::get<1>(this->round_pillar_zrange) + std::get<2>(remove_dist))));

    // 円弧内側の条件判定を行う
    arrayXb round_remove_ind(xyz.rows());
    for (int i = 0; i < xyz.rows(); i++)
    {
        round_remove_ind(i) = false;
    }

    auto len = std::min(this->round_pillar_centers.size(), this->round_pillar_radiuses.size());
    for (size_t i = 0; i < len; i++)
    {
        const auto& center = this->round_pillar_centers.at(i);
        const auto& radius = this->round_pillar_radiuses.at(i);
        round_remove_ind =
            round_remove_ind ||
            ((target_xyz(Eigen::all, Eigen::seq(0, 2, 1)).array().rowwise() -
              Eigen::Vector2d(std::get<0>(center), std::get<1>(center))

                  .transpose()
                  .array())
                 .square()
                 .matrix()
                 .rowwise()
                 .sum()
                 .array()
                 .sqrt() <= (radius + std::sqrt(pow(std::get<0>(remove_dist), 2) + pow(std::get<1>(remove_dist), 2))));
    }

    round_pillar_remove_ind = round_pillar_remove_ind && round_remove_ind;

    return (cuboid_remove_ind || round_pillar_remove_ind);
}
