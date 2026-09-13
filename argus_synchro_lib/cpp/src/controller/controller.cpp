#include "octotree/controller.h"
#include "cpp_helper_lib/eigen_operator.h"
#include "scene/common.h"
#include <algorithm>
#include <stdexcept>

/** 衝突判定の引数を生成するためのビルダークラス
 */
OctotreeCollisionConfigBuilder::OctotreeCollisionConfigBuilder(NodeEntity dest_entity) : dest_entity(dest_entity)
{
}

OctotreeCollisionConfigBuilder& OctotreeCollisionConfigBuilder::setOctotree(OctoTree& octotree_obj)
{
    this->octotree_obj = &octotree_obj;
    return *this;
}

OctotreeCollisionConfigBuilder&
OctotreeCollisionConfigBuilder::setSrcMeasureEntities(const std::vector<NodeEntity>& ents)
{
    this->src_measure_entities = ents;
    return *this;
}
OctotreeCollisionConfigBuilder&
OctotreeCollisionConfigBuilder::setSrcDetectEntities(const std::vector<NodeEntity>& ents)
{
    this->src_detect_entities = ents;
    return *this;
}

OctotreeCollisionConfigBuilder&
OctotreeCollisionConfigBuilder::setSrcLabels(const std::optional<std::vector<std::optional<int>>> src_labels)
{
    this->src_labels = src_labels;
    return *this;
}
OctotreeCollisionConfigBuilder&
OctotreeCollisionConfigBuilder::setDestLabels(const std::optional<std::vector<std::optional<int>>> dest_labels)
{
    this->dest_labels = dest_labels;
    return *this;
}

OctotreeCollisionConfigBuilder& OctotreeCollisionConfigBuilder::setDialatePointSize(int size)
{
    this->dialate_point_size = size;
    return *this;
}
OctotreeCollisionConfigBuilder& OctotreeCollisionConfigBuilder::setDistanceThreshold(std::optional<double> th)
{
    this->distance_threshold = th;
    return *this;
}
OctotreeCollisionConfigBuilder& OctotreeCollisionConfigBuilder::setAngles(double roll_rad, double pitch_rad,
                                                                          double yaw_rad)
{
    this->roll_angle = roll_rad;
    this->pitch_angle = pitch_rad;
    this->yaw_angle = yaw_rad;
    return *this;
}
OctotreeCollisionConfigBuilder&
OctotreeCollisionConfigBuilder::setDetectWindow(std::optional<Eigen::Vector3d> detect_window)
{
    this->detect_window = detect_window;
    return *this;
}

/**
 * @brief setした内容に基づいて衝突判定の引数を生成する
 * @details 必須な引数の設定がなければ例外を投げて, そうでないものはデフォルト値を入れて引数を作っている
 * @return OctotreeCollisionConfig
 */
OctotreeCollisionConfig OctotreeCollisionConfigBuilder::build()
{
    if (this->octotree_obj == nullptr || this->src_measure_entities.empty() || this->src_detect_entities.empty())
    {
        throw std::runtime_error("octotree_obj, src_measure_entities, and src_detect_entities must be set.");
    }
    return OctotreeCollisionConfig{this->octotree_obj,
                                   this->src_measure_entities,
                                   this->src_detect_entities,
                                   this->dest_entity,
                                   this->src_labels,
                                   this->dest_labels,
                                   this->dialate_point_size,
                                   this->detect_window,
                                   this->roll_angle,
                                   this->pitch_angle,
                                   this->yaw_angle,
                                   this->distance_threshold,
                                   this->metric};
}

Matrix3d rotate_x(double theta)
{
    Matrix3d result;
    result(0, 0) = 1.0;
    result(0, 1) = 0.0;
    result(0, 2) = 0.0;

    result(1, 0) = 0.0;
    result(1, 1) = cos(theta);
    result(1, 2) = -sin(theta);

    result(2, 0) = 0.0;
    result(2, 1) = sin(theta);
    result(2, 2) = cos(theta);

    return result;
}

Matrix3d rotate_y(double theta)
{
    Matrix3d result;
    result(0, 0) = cos(theta);
    result(0, 1) = 0.0;
    result(0, 2) = sin(theta);

    result(1, 0) = 0.0;
    result(1, 1) = 1.0;
    result(1, 2) = 0.0;

    result(2, 0) = -sin(theta);
    result(2, 1) = 0.0;
    result(2, 2) = cos(theta);

    return result;
}

Matrix3d rotate_z(double theta)
{
    Matrix3d result;
    result(0, 0) = cos(theta);
    result(0, 1) = -sin(theta);
    result(0, 2) = 0.0;

    result(1, 0) = sin(theta);
    result(1, 1) = cos(theta);
    result(1, 2) = 0.0;

    result(2, 0) = 0.0;
    result(2, 1) = 0.0;
    result(2, 2) = 1.0;

    return result;
}

Matrix3d rotate_xyz(double roll_rad_angle, double pitch_rad_angle, double yaw_rad_angle)
{
    return rotate_x(roll_rad_angle) * rotate_y(pitch_rad_angle) * rotate_z(yaw_rad_angle);
}

/**
 * @brief 八分木にLiDAR点群を入れて、入れたデータをクラスタリングに使うデータに変換してそれを返す
 * @remark 入れたデータは(null, target_entity)をkeyとしたentity_octonodesに格納される
 *
 * @param pcd_points LiDAR点群 n*3
 * @param octotree_pcd 八分木インスタンス
 * @param target_entity どのNodeEntityに入れるか
 * @param point_depth どの階層でクラスタリングデータを作るか, nullの場合一番下の階層でクラスタリングデータを作る
 * @return std::tuple<Eigen::MatrixXd, OctoTree&> クラスタリングデータと八分木インスタンスのメモリ番地のtuple
 */
std::tuple<Eigen::MatrixXd, OctoTree&> octotree_accum_points(const Eigen::Ref<const Eigen::MatrixXd>& pcd_points,
                                                             OctoTree& octotree_pcd, NodeEntity target_entity,
                                                             std::optional<int> point_depth)
{
    // 機体除去されたlidar点群を八分木に入れる
    octotree_pcd.insert_or_entity_octonodes(pcd_points, target_entity, true);

    // 入れた点群をクラスタリングデータとして取り出す
    Eigen::MatrixXd octotree_points = octotree_pcd.get_clustering_data_by_entity(target_entity, point_depth);

    return {octotree_points, octotree_pcd};
}

/**
 * @brief 機体周囲のLiDAR点群を除去する関数
 *
 * @param pcd_points LiDAR点群
 * @param l_machine_col 各機体のどこを除去するかが入っているリスト
 * @param remove_dist どのくらい機体を除去するかを表すtuple, nullの場合は点群除去を行わない
 * @param roll_angle 機体の回転角度
 * @param pitch_angle 機体の回転角度
 * @param yaw_angle 機体の回転角度
 * @return Eigen::MatrixXd 機体除去後のLiDAR点群
 */
Eigen::MatrixXd remove_machine_points(const Eigen::Ref<const Eigen::MatrixXd>& pcd_points,
                                      const std::vector<MachineCollisionBase*>& l_machine_col,
                                      const std::optional<std::tuple<double, double, double>>& remove_dist,
                                      double roll_angle, double pitch_angle, double yaw_angle)
{
    // 機体除去処理
    auto machine_size = l_machine_col.size();
    size_t point_size = pcd_points.rows();
    MatrixX3b remove_ind_all(machine_size, point_size);

    // 機体部位のそれぞれで近い点群のチェックを行う
    for (size_t i = 0; i < machine_size; i++)
    {
        if (remove_dist.has_value())
        {
            const arrayXb row = l_machine_col.at(i)->check_pcd_on_self(pcd_points, remove_dist.value(), roll_angle,
                                                                       pitch_angle, yaw_angle);
            remove_ind_all.row(static_cast<Eigen::Index>(i)) = row;
        }
        else
        {
            for (size_t j = 0; j < point_size; j++)
            {
                remove_ind_all(i, j) = false;
            }
        }
    }
    arrayXb not_remove_ind = !(remove_ind_all.array().colwise().any()).cast<bool>();
    // 除去対象ではないインデックスの番号を値として含んでいる配列(ex: 1,3,4,...)
    Eigen::VectorXi not_remove = helper::nonzero(not_remove_ind);
    Eigen::MatrixXd removed_pcd_points = pcd_points(not_remove, Eigen::all);

    return removed_pcd_points;
}

/**
 * @brief 回転などを行う八分木インスタンスに回転並進させた点群を入れて、八分木インスタンスを更新
 * @deprecated entity_octonodesを使うのに変えてからこの関数は今は使っていない
 */
OctoTree transfer_movable_octotree(OctoTree& octotree_obj, const Eigen::Ref<const Eigen::MatrixXd>& octotree_points,
                                   double roll_angle, double pitch_angle, double yaw_angle)
{
    Eigen::Vector3d offsets(0.0, 0.0, 0.0);
    Eigen::MatrixXd update_points = ((octotree_points.array().rowwise() - offsets.array().transpose()).matrix() *
                                     rotate_xyz(roll_angle, pitch_angle, yaw_angle).transpose())
                                        .array()
                                        .rowwise() +
                                    offsets.array().transpose();
    return octotree_obj.create_octonodes(update_points);
}

/**
 * @brief 衝突判定を実際に行う関数
 * @deprecated entity_octonodesなどを使うようになってから使っていない
 */
std::tuple<std::map<std::variant<int, std::string>, CollisionDetResult>, OctoTree, OctoTree, OctoTree>
octotree_collision_detection(OctoTree& octotree_pcd, OctoTree& octotree_machine_mobile_detect,
                             OctoTree& octotree_machine_immobile_detect,
                             const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_detect,
                             OctoTree& octotree_machine_mobile_measure, OctoTree& octotree_machine_immobile_measure,
                             const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_measure,
                             LayerBasedCollisionDetector& collision_detector,
                             const std::optional<std::vector<int>>& dest_labels, int dialate_point_size,
                             const std::optional<Eigen::Vector3d>& detect_window, double roll_angle, double pitch_angle,
                             double yaw_angle, std::optional<double> distance_threshold,
                             const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{
    return _octotree_collision_detection<int>(
        octotree_pcd, octotree_machine_mobile_detect, octotree_machine_immobile_detect, machine_mobile_points_detect,
        octotree_machine_mobile_measure, octotree_machine_immobile_measure, machine_mobile_points_measure,
        collision_detector, dest_labels, dialate_point_size, detect_window, roll_angle, pitch_angle, yaw_angle,
        distance_threshold, metric);
}

/**
 * @brief 衝突判定を実際に行う関数
 * @deprecated entity_octonodesなどを使うようになってから使っていない
 */
std::tuple<std::map<std::variant<int, std::string>, CollisionDetResult>, OctoTree, OctoTree, OctoTree>
octotree_collision_detection(OctoTree& octotree_pcd, OctoTree& octotree_machine_mobile_detect,
                             OctoTree& octotree_machine_immobile_detect,
                             const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_detect,
                             OctoTree& octotree_machine_mobile_measure, OctoTree& octotree_machine_immobile_measure,
                             const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_measure,
                             NeighborBasedCollisionDetector& collision_detector,
                             const std::optional<std::vector<int>>& dest_labels, int dialate_point_size,
                             const std::optional<Eigen::Vector3d>& detect_window, double roll_angle, double pitch_angle,
                             double yaw_angle, std::optional<double> distance_threshold,
                             const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{
    return _octotree_collision_detection<std::tuple<int, int, int>>(
        octotree_pcd, octotree_machine_mobile_detect, octotree_machine_immobile_detect, machine_mobile_points_detect,
        octotree_machine_mobile_measure, octotree_machine_immobile_measure, machine_mobile_points_measure,
        collision_detector, dest_labels, dialate_point_size, detect_window, roll_angle, pitch_angle, yaw_angle,
        distance_threshold, metric);
}

/**
 * @brief 対象の点群を回転並進させて必要なNodeEntityに格納する
 *
 * @param octotree_obj 八分木インスタンス
 * @param octotree_points 対象の点群
 * @param transfered_entity 対象のNodeEntity
 * @param entity_replace 対象のNodeEntityのentity_octonodesを上書きするかどうか, trueの場合,
 * transfered_entityをkeyに持つentity_octonodesを全て削除してから変のお後の点群を入れる
 * @param roll_angle 機体の回転角度
 * @param pitch_angle 機体の回転角度
 * @param yaw_angle 機体の回転角度
 * @return OctoTree& 八分木インスタンスのメモリ番地
 */
OctoTree& update_movable_entity(OctoTree& octotree_obj, Eigen::MatrixXd octotree_points, NodeEntity transfered_entity,
                                bool entity_replace, double roll_angle, double pitch_angle, double yaw_angle)
{
    // 並進ベクトル
    // Todo: 引数にして任意の並進もできるようにした方が良い
    Eigen::Vector3d offsets(0.0, 0.0, 0.0);

    // 対象の点群を回転させる
    Eigen::MatrixXd update_points = ((octotree_points.array().rowwise() - offsets.array().transpose()).matrix() *
                                     rotate_xyz(roll_angle, pitch_angle, yaw_angle).transpose())
                                        .array()
                                        .rowwise() +
                                    offsets.array().transpose();

    // 回転した点群をtransfered_entityに書き込む
    return octotree_obj.insert_or_entity_octonodes(update_points, transfered_entity, entity_replace);
}

ClusterColMap octotree_collision_detection_entities(LayerBasedCollisionDetector& collision_detector,
                                                    const OctotreeCollisionConfig& cfg)
{
    return _octotree_collision_detection_entities<int>(collision_detector, cfg);
}

ClusterColMap octotree_collision_detection_entities(NeighborBasedCollisionDetector& collision_detector,
                                                    const OctotreeCollisionConfig& cfg)
{
    return _octotree_collision_detection_entities<std::tuple<int, int, int>>(collision_detector, cfg);
}

t_py_col_res cluster_col_map_to_py(const ClusterColMap& clusters)
{
    t_py_col_res result;
    result.reserve(clusters.size());
    for (const auto& [label, val] : clusters)
    {
        result.emplace(label, std::make_tuple(val.src_node, val.dest_node, val.src_coord, val.dest_coord,
                                              val.src_dest_dist, std::nullopt));
    }
    return result;
}
