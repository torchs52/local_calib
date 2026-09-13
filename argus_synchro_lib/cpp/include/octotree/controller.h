#pragma once

#include "AbstractCollisionDetector.h"
#include "LayerBasedCollisionDetector.h"
#include "MachineCollisionBase.h"
#include "NeighborBasedCollisionDetector.h"
#include "NodeEntity.h"
#include "OctoTree.h"
#include "scene/common.h"
#include <optional>

using Matrix3d = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
// Use Eigen::MatrixXd instead of Matrix3d
using Matrix3i = Eigen::Matrix<int, 3, 3, Eigen::ColMajor>;
// Use Eigen::MatrixXd instead of Matrix3d

/**
 * @brief 衝突判定に用いる引数をまとめた構造体
 *
 */
struct OctotreeCollisionConfig
{
    // 衝突判定に用いるデータが入った八分木インスタンス (所有権は呼び出し元)
    OctoTree* octotree_obj = nullptr;

    // 最短部位計算のsrc側のNodeEntityのリスト
    std::vector<NodeEntity> src_measure_entities;

    // 接触可能性探索のsrc側のNodeEntityのリスト
    std::vector<NodeEntity> src_detect_entities;

    /**
     * @brief 最短部位計算, 接触可能性探索のdest側のNodeEntity
     * @remark dest側がLiDAR点群を想定していて、最短部位計算,
     * 接触可能性探索計算で同じ単一のNodeEntityなのでsrc側と同じ感じになっていない
     * @todo src側と同じ感じに作り直しても良い気もするので、時間があれば直す
     *
     */
    NodeEntity dest_entity;

    // 衝突判定計算に用いるsrc側のラベル, nullの場合、全てのNodeEntityを用いる
    std::optional<std::vector<std::optional<int>>> src_labels = std::nullopt;

    // 衝突判定計算に用いるdest側のラベル, nullの場合、全てのNodeEntityを用いる
    std::optional<std::vector<std::optional<int>>> dest_labels = std::nullopt;

    // 衝突判定をどれだけ膨らませるかを表す整数
    // Todo dilateのタイポ
    int dialate_point_size = 2;

    // dest側で衝突判定から除外される範囲, src側の最大最小 +-
    // detect_windowに入っていないdest側の点群は衝突判定から除外される, nullの場合はこの処理は無効
    std::optional<Eigen::Vector3d> detect_window = Eigen::Vector3d(3.0, 3.0, 3.0);

    // 機体のroll, pitch, yaw角
    double roll_angle = 0.0;
    double pitch_angle = 0.0;
    double yaw_angle = 0.0;

    // 最短部位がこの閾値より大きい場合は衝突判定から除外される
    std::optional<double> distance_threshold = 10.0;

    // 距離関数, nullの場合ユークリッド距離が使われる想定
    // Todo: nullでない場合の処理が作れていないので作った方が良い
    std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt;
};

/**
 * @brief 衝突判定に用いる引数を生成するための構造体
 * @remark
 * ビルダーパターンを意識して作ったが引数がそんなに状況によって変わらないので要らない気もするが議論の場がなく、まず作った結果残っている状況
 *
 */
struct OctotreeCollisionConfigBuilder
{
    OctoTree* octotree_obj = nullptr;
    std::vector<NodeEntity> src_measure_entities;
    std::vector<NodeEntity> src_detect_entities;
    NodeEntity dest_entity;

    std::optional<std::vector<std::optional<int>>> src_labels = std::nullopt;
    std::optional<std::vector<std::optional<int>>> dest_labels = std::nullopt;
    int dialate_point_size = 2;
    std::optional<Eigen::Vector3d> detect_window = Eigen::Vector3d(3.0, 3.0, 3.0);
    double roll_angle = 0.0;
    double pitch_angle = 0.0;
    double yaw_angle = 0.0;
    std::optional<double> distance_threshold = 10.0;
    std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt;

    OctotreeCollisionConfigBuilder(NodeEntity dest_entity);

    OctotreeCollisionConfigBuilder& setOctotree(OctoTree& octotree_obj);

    OctotreeCollisionConfigBuilder& setSrcMeasureEntities(const std::vector<NodeEntity>& ents);
    OctotreeCollisionConfigBuilder& setSrcDetectEntities(const std::vector<NodeEntity>& ents);

    OctotreeCollisionConfigBuilder& setSrcLabels(const std::optional<std::vector<std::optional<int>>> src_labels);
    OctotreeCollisionConfigBuilder& setDestLabels(const std::optional<std::vector<std::optional<int>>> dest_labels);

    OctotreeCollisionConfigBuilder& setDialatePointSize(int size);
    OctotreeCollisionConfigBuilder& setDistanceThreshold(std::optional<double> th);
    OctotreeCollisionConfigBuilder& setAngles(double roll, double pitch, double yaw);
    OctotreeCollisionConfigBuilder& setDetectWindow(std::optional<Eigen::Vector3d> detect_window);

    /**
     * @brief setした内容に基づいて衝突判定の引数を生成する
     *
     * @return OctotreeCollisionConfig 衝突判定で用いる引数の一覧
     */
    OctotreeCollisionConfig build();
};

/**
 * x軸周りにthetaだけ回転する行列を作成する
 */
Matrix3d rotate_x(double theta);

/**
 * y軸周りにthetaだけ回転する行列を作成する
 */
Matrix3d rotate_y(double theta);

/**
 * z軸周りにthetaだけ回転する行列を作成する
 */
Matrix3d rotate_z(double theta);

/**
 * roll, pitch, yawのradianが与えられたときに、回転行列を返す関数
 */
Matrix3d rotate_xyz(double roll_rad_angle, double pitch_rad_angle, double yaw_rad_angle);

/**
 * @brief 八分木にLiDAR点群を入れて、クラスタリングデータを作る
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
                                                             std::optional<int> point_depth);

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
                                      double roll_angle, double pitch_angle, double yaw_angle);

/**
 * @brief 回転などを行う八分木インスタンスに回転並進させた点群を入れて、八分木インスタンスを更新
 * @deprecated entity_octonodesを使うのに変えてからこの関数は今は使っていない
 */
OctoTree transfer_movable_octotree(OctoTree& octotree_obj, const Eigen::Ref<const Eigen::MatrixXd>& octotree_points,
                                   double roll_angle = 0, double pitch_angle = 0, double yaw_angle = 0);

/**
 * @brief 衝突判定を実際に行う関数
 * @deprecated entity_octonodesなどを使うようになってから使っていない
 */
template <typename T>
std::tuple<std::map<std::variant<int, std::string>, CollisionDetResult>, OctoTree, OctoTree, OctoTree>
_octotree_collision_detection(OctoTree& octotree_pcd, OctoTree& octotree_machine_mobile_detect,
                              OctoTree& octotree_machine_immobile_detect,
                              const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_detect,
                              OctoTree& octotree_machine_mobile_measure, OctoTree& octotree_machine_immobile_measure,
                              const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_measure,
                              AbstractCollisionDetector<T>& collision_detector,
                              const std::optional<std::vector<int>>& dest_labels, int dialate_point_size,
                              const std::optional<Eigen::Vector3d>& detect_window, double roll_angle,
                              double pitch_angle, double yaw_angle, std::optional<double> distance_threshold,
                              const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{

    //距離計算用の機体点群の動く部分を動かす
    OctoTree updated_octotree_machine_mobile_measure = transfer_movable_octotree(
        octotree_machine_mobile_measure, machine_mobile_points_measure, roll_angle, pitch_angle, yaw_angle);
    // LiDAR点群のラベル付き八分木ノードに距離計算用機体点群を追加する
    auto copy_map1 = updated_octotree_machine_mobile_measure.unlabeled_octo_nodes.value();
    auto copy_map2 = octotree_machine_immobile_measure.unlabeled_octo_nodes.value();
    copy_map1.merge(copy_map2);
    octotree_pcd.labeled_octo_nodes.value().insert_or_assign(OctoTree::MACHINE_MEASURE_LABEL, copy_map1);

    //接触可能性探索用の機体点群の旋回に対応する部分を更新する
    OctoTree updated_octotree_machine_mobile_detect =
        octotree_machine_mobile_detect.create_octonodes(machine_mobile_points_detect);
    // LiDAR点群のラベル付き八分木ノードに接触可能性探索用機体点群を追加する
    auto copy_map3 = updated_octotree_machine_mobile_detect.unlabeled_octo_nodes.value();
    auto copy_map4 = octotree_machine_immobile_detect.unlabeled_octo_nodes.value();
    copy_map3.merge(copy_map4);
    octotree_pcd.labeled_octo_nodes.value().insert_or_assign(OctoTree::MACHINE_DETECT_LABEL, copy_map3);

    auto nodes = octotree_pcd.labeled_octo_nodes.value().at(OctoTree::MACHINE_DETECT_LABEL);

    // 機体点群を含む八分木を用いて衝突判定を行う
    auto collision_clusters =
        collision_detector.assign_octotree(octotree_pcd)
            .detect_collided_object(OctoTree::MACHINE_DETECT_LABEL, OctoTree::MACHINE_MEASURE_LABEL, dest_labels,
                                    dialate_point_size, detect_window, distance_threshold, metric);
    return {collision_clusters, octotree_pcd, updated_octotree_machine_mobile_measure,
            updated_octotree_machine_mobile_detect};
}

/**
 * @brief 衝突判定を実際に行う関数
 * @deprecated entity_octonodesなどを使うようになってから使っていない
 */
std::tuple<std::map<std::variant<int, std::string>, CollisionDetResult>, OctoTree, OctoTree, OctoTree>
octotree_collision_detection(
    OctoTree& octotree_pcd, OctoTree& octotree_machine_mobile_detect, OctoTree& octotree_machine_immobile_detect,
    const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_detect, OctoTree& octotree_machine_mobile_measure,
    OctoTree& octotree_machine_immobile_measure, const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_measure,
    LayerBasedCollisionDetector& collision_detector, const std::optional<std::vector<int>>& dest_labels = std::nullopt,
    int dialate_point_size = 2, const std::optional<Eigen::Vector3d>& detect_window = Eigen::Vector3d(3.0, 3.0, 3.0),
    double roll_angle = 0, double pitch_angle = 0, double yaw_angle = 0,
    std::optional<double> distance_threshold = 10.0,
    const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt);

/**
 * @brief 衝突判定を実際に行う関数
 * @deprecated entity_octonodesなどを使うようになってから使っていない
 */
std::tuple<std::map<std::variant<int, std::string>, CollisionDetResult>, OctoTree, OctoTree, OctoTree>
octotree_collision_detection(
    OctoTree& octotree_pcd, OctoTree& octotree_machine_mobile_detect, OctoTree& octotree_machine_immobile_detect,
    const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_detect, OctoTree& octotree_machine_mobile_measure,
    OctoTree& octotree_machine_immobile_measure, const Eigen::Ref<const Eigen::MatrixXd>& machine_mobile_points_measure,
    NeighborBasedCollisionDetector& collision_detector,
    const std::optional<std::vector<int>>& dest_labels = std::nullopt, int dialate_point_size = 2,
    const std::optional<Eigen::Vector3d>& detect_window = Eigen::Vector3d(3.0, 3.0, 3.0), double roll_angle = 0,
    double pitch_angle = 0, double yaw_angle = 0, std::optional<double> distance_threshold = 10.0,
    const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt);

// entity_octonodes対応

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
                                bool entity_replace = true, double roll_angle = 0, double pitch_angle = 0,
                                double yaw_angle = 0);

/**
 * @brief configベースで衝突判定を行う
 *
 * @param collision_detector 衝突判定インスタンス
 * @param cfg 衝突判定の引数
 * @return ClusterColMap 各クラスタ毎の衝突判定結果が入ったマップ
 */
template <typename T>
ClusterColMap _octotree_collision_detection_entities(AbstractCollisionDetector<T>& collision_detector,
                                                     const OctotreeCollisionConfig& cfg)
{

    if (cfg.octotree_obj == nullptr)
    {
        throw std::invalid_argument("八分木が設定されていないため、衝突判定ができません");
    }

    // 機体点群を含む八分木を用いて衝突判定を行う
    auto collision_clusters =
        collision_detector.assign_octotree(*cfg.octotree_obj)
            .detect_collided_entities(cfg.src_measure_entities, cfg.src_detect_entities, cfg.dest_entity,
                                      cfg.src_labels, cfg.dest_labels, cfg.dialate_point_size, cfg.detect_window,
                                      cfg.distance_threshold, cfg.metric);
    return collision_clusters;
}

/**
 * @brief configベースで衝突判定を行う, LayerBasedの引数
 *
 * @param collision_detector
 * @param cfg
 * @return ClusterColMap
 */
ClusterColMap octotree_collision_detection_entities(LayerBasedCollisionDetector& collision_detector,
                                                    const OctotreeCollisionConfig& cfg);

/**
 * @brief configベースで衝突判定を行う, NeighborBasedの引数
 *
 * @param collision_detector
 * @param cfg
 * @return ClusterColMap
 */
ClusterColMap octotree_collision_detection_entities(NeighborBasedCollisionDetector& collision_detector,
                                                    const OctotreeCollisionConfig& cfg);

t_py_col_res cluster_col_map_to_py(const ClusterColMap& clusters);
