#pragma once

#include "NodeEntity.h"
#include "OctoNode.h"
#include "OctoTree.h"
#include "alias.h"
#include "cpp_helper_lib/eigen_operator.h"

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
using MatrixX3b = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
// Use MatrixX3b instead of MatrixXd
using Matrix3d = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
// Use Matrix3d instead of Matrix3d
using Matrix3i = Eigen::Matrix<int, 3, 3, Eigen::ColMajor>;
// Use Matrix3i instead of Matrix3d

using LiDARCoord = std::tuple<double, double, double>;
// 実空間の座標のエイリアス

/**
 * @brief 衝突判定の結果の構造体, mapのvalueのtupleが長かったので新しく作った
 * @remark
 * 格納する情報を増やす場合は、属性を追加するだけでなく、Pythonと連携する場合はto_dictに追加したい属性を追加したりする必要がある
 *
 */
struct CollisionDetResult
{
    OctoNode src_node;     // src側の衝突部位に対応する八分木ノード
    OctoNode dest_node;    // dest側の衝突部位に対応する八分木ノード
    LiDARCoord src_coord;  // src側の衝突部位に対応するLiDAR座標
    LiDARCoord dest_coord; // dest側の衝突部位に対応するLiDAR座標
    double src_dest_dist;  // srcとdestのLiDAR座標系における距離

    /**
     * @brief この構造体をPythonにおけるdictに変換するメソッド
     *
     * @return py::dict pythonの辞書インスタンス
     */
    std::map<std::string, std::variant<OctoNode, LiDARCoord, double>> to_dict() const
    {
        std::map<std::string, std::variant<OctoNode, LiDARCoord, double>> d;
        d.insert(std::make_pair("src_node", this->src_node));
        d.insert(std::make_pair("dest_node", this->dest_node));
        d.insert(std::make_pair("src_coord", this->src_coord));
        d.insert(std::make_pair("dest_coord", this->dest_coord));
        d.insert(std::make_pair("src_dest_dist", this->src_dest_dist));
        return d;
    }

    /**
     * @brief この構造体をtupleに変換するメソッド
     *
     * @return std::tuple<OctoNode, OctoNode, LiDARCoord, LiDARCoord, double> srcの最短部位の八分木ノード,
     * destの最短部位の八分木ノード, srcの最短部位のLiDAR座標, destの最短部位のLiDAR座標, 2点の距離
     */
    std::tuple<OctoNode, OctoNode, LiDARCoord, LiDARCoord, double> to_tuple() const
    {
        return std::make_tuple(src_node, dest_node, src_coord, dest_coord, src_dest_dist);
    }
};

using ClusterColMap = std::map<std::optional<int>, CollisionDetResult>;
// 各クラスタ毎の衝突判定結果が入ったマップを表すalias

/**
 * @brief 八分木ノードのLiDAR座標の表現方法を表す列挙型
 *
 */
enum class CoordMethod
{
    VOX_MED, // 八分木ノードの格子中心をLiDAR座標とする場合の設定値
    MEAN,    // 八分木ノードに入っている点の平均値をLiDAR座標とする場合の設定値
    FAR_POINT, // 八分木ノードに入っている点で原点から最も遠い点をLiDAR座標とする場合の設定値
    NEAR_POINT, // 八分木ノードに入っている点で原点から最も近い点をLiDAR座標とする場合の設定値
    QUANTILE // 八分木ノードに入っている点で原点からの距離がquantile位になる点をLiDAR座標とする場合の設定値
};

/**
 * @brief 文字列からCoordMethodを得る
 * @details coord_method_from_string("MEAN")でCoordMethod::MEANが得られる
 * @remarks Python側でgetattr(CoordMethod, xxx)として運用しているが、その方が良い面もると思うので、要確認
 *
 * @param s
 * @return CoordMethod
 */
inline CoordMethod coord_method_from_string(const std::string& s)
{
    if (s == "VOX_MED")
        return CoordMethod::VOX_MED;
    if (s == "MEAN")
        return CoordMethod::MEAN;
    if (s == "FAR_POINT")
        return CoordMethod::FAR_POINT;
    if (s == "NEAR_POINT")
        return CoordMethod::NEAR_POINT;
    if (s == "QUANTILE")
        return CoordMethod::QUANTILE;

    throw std::invalid_argument("Unknown CoordMethod: " + s);
}

template <typename T>
/**
 * @brief 衝突判定を行うクラスを抽象化したクラス
 * @details
処理としては、LiDAR点群の各クラスタの衝突可能性探索を行った後で、衝突可能性があるクラスタに対して最短部位を計算するということを行っている
 抽象化している部分は、衝突可能性探索の部分で
2つの点群のグループの
 1. 八分木のある階層で同じ格子に入っていることとするか
 2. 八分木の格子を膨張させたときに同じ格子に入っていることとするか
といった方法が取れて、どちらでも上位からは同じ呼び出しを行うことで衝突判定の一連の処理ができるようにしている
 *
 */
class AbstractCollisionDetector
{
  public:
    CoordMethod coord_method;      // セルの代表点を何にするかを決めるenum
    OctoTree* octotree_obj = nullptr; // 衝突判定に用いる情報が入っている八分木クラス (所有権は呼び出し元)

    AbstractCollisionDetector(CoordMethod coord_method = CoordMethod::VOX_MED);

    virtual ~AbstractCollisionDetector() = default;

    AbstractCollisionDetector<T>& assign_octotree(OctoTree& octotree_obj);

    /**
     * @brief src_labelとdest_labelsの衝突判定を行う
     * @deprecated 処理が古いのでupdateはされない
     * @remark このメソッド自体は古くて、今はdetect_collided_entitiesを使っている **
     * @details windowを変えることで、src_label側の点群を広げて衝突判定を行う,
     * windowの変え方がどのように衝突判定に影響があるかは、collision_detectionを実装しているクラスに任せているため、そちらを確認する必要がある
     * @param src_label     衝突判定を行いたい側のラベル
     * @param dest_labels   src_labelが衝突しているかどうか判定したいラベル,
     * Noneの場合はsegmented_octonodesでsrc_label以外を対象とする
     * @param windows       src_labelの点群をどれくらい広げるかを表すパラメータ
     * @param detect_window
     * srcに対してどれくらいの範囲で衝突判定を行うかを表すリスト(単位: メートル)
     * @retval
     * 衝突のあったラベル毎にsrcとdestの最小の組とその距離の大きさが入った辞書,
     * 衝突するラベルが存在しなければ、空の辞書を返す
     */
    std::map<std::variant<int, std::string>, CollisionDetResult> detect_collided_object(
        const std::string& src_detect_label, const std::string& src_measure_label,
        const std::optional<std::vector<int>>& dest_labels = std::nullopt, int window = 2,
        const std::optional<Eigen::Vector3d>& detect_window = Eigen::Vector3d(3.0, 3.0, 3.0),
        std::optional<double> distance_threshold = 10.0,
        const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt);

    /**
        @brief 衝突判定全体の処理, NodeEntityベースで衝突判定を行っている
        @details src側は機体点群,
       dest側はLiDAR点群想定で作っていて、機体点群はmeasureとdetectのそれぞれがあるため、それぞれの引数が存在している
        @param src_measure_entities src側で距離計算に用いる属性のリスト
        @param src_detect_entities src側で衝突可能性探索に用いる属性のリスト
        @param dest_entity: dest側で衝突判定に用いる属性
        @param src_labels: src側で衝突判定に用いているラベル番号のリスト, nullの場合は、全てのラベルを見る
        @param dest_labels: dest側で衝突判定に用いているラベル番号のリスト, nullの場合は、全てのラベルを見る
        @param window: 実装クラスに応じた広げる度合を表す数値
        @param detect_window: dest側で衝突判定から除外される範囲, src側の最大最小 +-
       detect_windowに入っていないdest側の点群は衝突判定から除外される, nullの場合はこの処理は無効
        @param distance_threshold: 最短部位がdistance_thresholdより大きい場合は最終的な結果からは除外される,
       nullの場合はこの処理は無効
        @param metric: 最短部位計算方法を表す関数のつもりだったが、今は使っていない
     */
    ClusterColMap detect_collided_entities(
        const std::vector<NodeEntity>& src_measure_entities, const std::vector<NodeEntity>& src_detect_entities,
        NodeEntity dest_entity, const std::optional<std::vector<std::optional<int>>>& src_labels = std::nullopt,
        const std::optional<std::vector<std::optional<int>>>& dest_labels = std::nullopt, int window = 2,
        const std::optional<Eigen::Vector3d>& detect_window = Eigen::Vector3d(3.0, 3.0, 3.0),
        std::optional<double> distance_threshold = 10.0,
        const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt);

    /* keynodesをボクセル離散座標に変換する関数 */
    Eigen::MatrixXi _keynode2array(const std::map<std::tuple<int, int, int>, OctoNode>& keynodes);

    /* keynodesからボクセル内の平均値の座標を取得する関数 */
    Eigen::MatrixXd _mean2array(const std::map<std::tuple<int, int, int>, OctoNode>& keynodes);

    /* keynodesからボクセル内の原点から一番離れている点の座標を取得する関数 */
    Eigen::MatrixXd _far2array(const std::map<std::tuple<int, int, int>, OctoNode>& keynodes);

    /* keynodesからボクセル内の原点から一番近い点の座標を取得する関数 */
    Eigen::MatrixXd _near2array(const std::map<std::tuple<int, int, int>, OctoNode>& keynodes);

    /* keynodesからボクセル内の原点からquantileになっている点の座標を取得する関数
     */
    Eigen::MatrixXd _quantile2array(const std::map<std::tuple<int, int, int>, OctoNode>& keynodes);

    /* keynodesからボクセル内の中心点座標を取得する関数 */
    Eigen::MatrixXd _med2array(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords);

    /* srcとdestのボクセル座標とボクセル内の代表点の座標の組を取得する関数 */
    std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
    _get_vox_w_pairs(const std::map<std::tuple<int, int, int>, OctoNode>& src_nodes,
                     const std::map<std::tuple<int, int, int>, OctoNode>& dest_nodes);

    /**
     * @brief src_nodesとdest_nodesの中で、距離が最小のペアを見つける
     * @param src_nodes, dest_nodes: 最小のペアを見つける2つの点群
     * @param distance_threshold 最短距離がこれより大きい場合は衝突判定の結果に追加しない
     * @return std::optional<CollisionDetResult> 最短部位の情報が入った構造体,
     * nullの場合、有効な最短部位がなかったことを表す
     */
    std::optional<CollisionDetResult> find_minimum_node_pair(
        const std::map<VoxelCoord, OctoNode>& src_nodes, const std::map<VoxelCoord, OctoNode>& dest_nodes,
        std::optional<double> distance_threshold = 10.0,
        const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric = std::nullopt);

    /**
     * @brief Create a dialation coord object
     * @details
     * collision_detectionを呼ぶたびに、src_nodesの膨張していて、非効率だった点、collision_detectionで膨張させる処理を行うのは、多機能持たせている点、現状の実装クラスでは、どの場合も膨張処理をしている点を顧みて実装
     * @param nodes 膨張させる点群が入った辞書
     * @param window 膨張させる幅, 他のパラメータを出てきたら,
     * @return std::set<T> 実装メソッドで膨張させられた点群の集合, それを用いて衝突判定を行う
     */
    virtual std::set<T> create_dialation_coord(const std::map<VoxelCoord, OctoNode>& nodes, int window = 2) = 0;

    /**
     * @brief Create a dest coord object
     * @details
     * LayerBasedの方は、モートン順序の上位ビットがsrcとdestの両方で必要であったので、このメソッドでそのあたりの整合性を保つ
     *
     * @param nodes 膨張させる点群が入った辞書
     * @param window 膨張させる幅, 他のパラメータを出てきたら,
     * @return std::set<T> 実装メソッドで膨張させられた点群の集合, それを用いて衝突判定を行う
     */
    virtual std::set<T> create_dest_coord(const std::map<VoxelCoord, OctoNode>& nodes, int window = 2) = 0;

    /**
     * @brief src_nodesとdest_nodesに衝突する点群があるかどうかを判定する
     *
     * @param src_coords 衝突判定対象の2つの点群
     * @param dest_coords 衝突判定対象の2つの点群
     * @return true 衝突の可能性あり
     * @return false 衝突の可能性なし
     */
    virtual bool collision_detection(const std::set<T>& src_coords, const std::set<T>& dest_coords) = 0;

  private:
    struct MinPairCalc
    {
        Eigen::MatrixXi src_coords;
        Eigen::MatrixXi dest_coords;
        Eigen::MatrixXd XA;
        Eigen::MatrixXd XB;
        double min_dist;
        int minRow;
        int minCol;
    };

    bool _should_use_median_for_nodes(const std::map<std::tuple<int, int, int>, OctoNode>& nodes) const;

    Eigen::MatrixXd _get_vox_w_coords(const std::map<std::tuple<int, int, int>, OctoNode>& nodes,
                                      const Eigen::Ref<const Eigen::MatrixXi>& vox_coords, bool use_median);

    MinPairCalc _calc_min_pair(const std::map<VoxelCoord, OctoNode>& src_nodes,
                               const std::map<VoxelCoord, OctoNode>& dest_nodes);

    std::optional<CollisionDetResult> _build_min_pair_result(const std::map<VoxelCoord, OctoNode>& src_nodes,
                                                             const std::map<VoxelCoord, OctoNode>& dest_nodes,
                                                             const Eigen::MatrixXi& src_coords,
                                                             const Eigen::MatrixXi& dest_coords, double min_dist,
                                                             int minRow, int minCol,
                                                             const std::optional<double>& distance_threshold);
};

/**
 * @brief Construct a new Abstract Collision Detector< T>:: Abstract Collision Detector object
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param coord_method 最短部位計算に用いるノードの代表点
 */
template <typename T>
AbstractCollisionDetector<T>::AbstractCollisionDetector(
    // double distance_threshold,
    CoordMethod coord_method)
    : coord_method(coord_method), octotree_obj(nullptr)
{
}

/**
 * @brief 八分木を衝突判定のインスタンスに設定する
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param octotree_obj
 * @return AbstractCollisionDetector<T>& 衝突判定を行うインスタンス
 */
template <typename T>
AbstractCollisionDetector<T>& AbstractCollisionDetector<T>::assign_octotree(OctoTree& octotree_obj)
{
    this->octotree_obj = &octotree_obj;
    return *this;
}

/**
 * @brief 衝突判定の全体処理
 * @deprecated entity_octonodesを使ってデータを格納することにしてから使っていない
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param src_detect_label
 * @param src_measure_label
 * @param dest_labels
 * @param window
 * @param detect_window
 * @param distance_threshold
 * @param metric
 * @return std::map<std::variant<int, std::string>, CollisionDetResult>
 */
template <typename T>
std::map<std::variant<int, std::string>, CollisionDetResult> AbstractCollisionDetector<T>::detect_collided_object(
    const std::string& src_detect_label, const std::string& src_measure_label,
    const std::optional<std::vector<int>>& dest_labels, int window, const std::optional<Eigen::Vector3d>& detect_window,
    std::optional<double> distance_threshold,
    const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{
    /** 衝突判定処理の実体部分
     */
    if (this->octotree_obj == nullptr)
    {
        throw std::invalid_argument("八分木が設定されていないため、衝突判定ができません");
    }

    auto segmented_octonodes = this->octotree_obj->labeled_octo_nodes.value();

    // 接触可能性探索の対象となるラベルをセグメント済みの八分木ノードから取り出す
    auto src_detect_nodes = segmented_octonodes.at(src_detect_label);

    // 距離計算の対象となるラベルをセグメント済みの八分木ノードから取り出す
    auto src_measure_nodes = segmented_octonodes.at(src_measure_label);

    // src_nodeに対する衝突判定対象としてみる範囲を計算
    int x_min = INT_MAX, y_min = INT_MAX, z_min = INT_MAX;
    int x_max = INT_MIN, y_max = INT_MIN, z_max = INT_MIN;
    for (const auto& [key, value] : src_detect_nodes)
    {
        auto x = std::get<0>(key);
        auto y = std::get<1>(key);
        auto z = std::get<2>(key);

        if (x < x_min)
            x_min = x;
        if (y < y_min)
            y_min = y;
        if (z < z_min)
            z_min = z;

        if (x > x_max)
            x_max = x;
        if (y > y_max)
            y_max = y;
        if (z > z_max)
            z_max = z;
    }

    auto vox_min_range = std::make_tuple(x_min, y_min, z_min);
    auto vox_max_range = std::make_tuple(x_max, y_max, z_max);
    std::optional<Eigen::Vector3i> vox_focus_range =
        detect_window.has_value()
            ? std::optional((detect_window.value().array() / this->octotree_obj->cell_interval.array())
                                .ceil()
                                .cast<int>()
                                .matrix()
                                .eval())
            : std::nullopt;

    std::map<std::variant<int, std::string>, std::map<VoxelCoord, OctoNode>> dest_nodes;
    // セグメント済みの八分木から衝突判定となる点群を取り出す,
    //  src_detect_label, src_measure_labelは除いて取り出す
    if (dest_labels == std::nullopt)
    {
        for (const auto& [dest_label, dest_nodes_tmp] : segmented_octonodes)
        {
            if (std::holds_alternative<int>(dest_label))
            {
                dest_nodes.insert_or_assign(dest_label, dest_nodes_tmp);
            }
            else if (std::holds_alternative<std::string>(dest_label))
            {
                auto str_dest_label = std::get<std::string>(dest_label);
                if (str_dest_label != src_detect_label && str_dest_label != src_measure_label)
                {
                    dest_nodes.insert_or_assign(dest_label, dest_nodes_tmp);
                }
            }
        }
    }
    else
    {
        for (const auto& dest_label : dest_labels.value())
        {
            dest_nodes.insert_or_assign(dest_label, segmented_octonodes.at(dest_label));
        }
    }

    // srcの点群を膨張させる, for文内で毎回実行すると非効率なため、外に出した
    std::set<T> src_detect_coords = this->create_dialation_coord(src_detect_nodes, window);

    const Eigen::MatrixXi src_coords = this->_keynode2array(src_measure_nodes);
    const bool src_use_median = this->_should_use_median_for_nodes(src_measure_nodes);

    std::map<std::variant<int, std::string>, CollisionDetResult> collision_pairs;
    for (const auto& [target_dest_label, target_dest_nodes] : dest_nodes)
    {
        // 衝突判定対象となる範囲が存在する場合は、除外条件を追加
        OctoMap filtered_dest_nodes;
        const OctoMap* dialation_nodes;
        if (vox_focus_range)
        {
            const Eigen::Vector3i focus = vox_focus_range.value();
            for (const auto& [key, value] : target_dest_nodes)
            {
                // xyzのいずれかは、機体を囲う直方体 +-
                // focusのxyzに入っていないものは可能性探索から除外する
                if (!(std::get<0>(vox_min_range) - focus(0) <= std::get<0>(key) &&
                      std::get<0>(key) <= std::get<0>(vox_max_range) + focus(0) &&
                      std::get<1>(vox_min_range) - focus(1) <= std::get<1>(key) &&
                      std::get<1>(key) <= std::get<1>(vox_max_range) + focus(1) &&
                      std::get<1>(vox_min_range) - focus(1) <= std::get<1>(key) &&
                      std::get<1>(key) <= std::get<1>(vox_max_range) + focus(1)))
                {
                    continue;
                }
                filtered_dest_nodes.insert_or_assign(key, value);
            }
            dialation_nodes = &filtered_dest_nodes;
        }
        else
        {
            dialation_nodes = &target_dest_nodes;
        }

        std::set<T> dest_coords = this->create_dest_coord(*dialation_nodes, window);

        // 衝突判定を行う
        if (this->collision_detection(src_detect_coords, dest_coords))
        {
            // srcと見ているdest_labelの中で衝突する可能性があるので、点群のどこが衝突しそうか計算する
            Eigen::MatrixXi dest_coords_arr = this->_keynode2array(target_dest_nodes);
            const bool dest_use_median = this->_should_use_median_for_nodes(target_dest_nodes);
            const bool use_median = src_use_median || dest_use_median;
            Eigen::MatrixXd XA = this->_get_vox_w_coords(src_measure_nodes, src_coords, use_median);
            Eigen::MatrixXd XB = this->_get_vox_w_coords(target_dest_nodes, dest_coords_arr, use_median);
            auto [min_dist, minRow, minCol] = helper::calc_cdist_min(XA, XB);
            const auto& minimum_pair =
                this->_build_min_pair_result(src_measure_nodes, target_dest_nodes, src_coords, dest_coords_arr, min_dist,
                                             minRow, minCol, distance_threshold);

            if (minimum_pair.has_value())
            {
                // 見ているラベルの組の最小距離がdistance_threshold以下の場合、衝突の組に追加
                collision_pairs.insert_or_assign(target_dest_label, minimum_pair.value());
            }
        }
    }
    return collision_pairs;
}

/**
 * @brief entity_octonodesバージョンの衝突判定
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param src_measure_entities 最短部位計算を行うsrc側のNodeEntityのリスト
 * @param src_detect_entities 衝突可能性探索を行うsrc側のNodeEntityのリスト
 * @param dest_entity 最短部位計算, 衝突可能性探索のそれぞれをdest側で行うNodeEntity
 * @param src_labels 計算に用いるsrc側のラベル, nullの場合全部使う
 * @param dest_labels 計算に用いるsrc側のラベル, nullの場合全部使う
 * @param window 膨らませる度合
 * @param detect_window 除外する大きさ
 * @param distance_threshold 最短部位の距離閾値
 * @param metric 距離関数
 * @return ClusterColMap 衝突判定計算結果
 */
template <typename T>
ClusterColMap AbstractCollisionDetector<T>::detect_collided_entities(
    const std::vector<NodeEntity>& src_measure_entities, const std::vector<NodeEntity>& src_detect_entities,
    NodeEntity dest_entity, const std::optional<std::vector<std::optional<int>>>& src_labels,
    const std::optional<std::vector<std::optional<int>>>& dest_labels, int window,
    const std::optional<Eigen::Vector3d>& detect_window, std::optional<double> distance_threshold,
    const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{
    if (this->octotree_obj == nullptr)
    {
        throw std::invalid_argument("八分木が設定されていないため、衝突判定ができません");
    }
    OctoMap src_measure_nodes =
        this->octotree_obj->collect_nodes_by_entities_and_clusters(src_measure_entities, src_labels);
    OctoMap src_detect_nodes =
        this->octotree_obj->collect_nodes_by_entities_and_clusters(src_detect_entities, src_labels);

    // src_nodeに対する衝突判定対象としてみる範囲を計算
    int x_min = INT_MAX, y_min = INT_MAX, z_min = INT_MAX;
    int x_max = INT_MIN, y_max = INT_MIN, z_max = INT_MIN;
    for (const auto& [key, value] : src_detect_nodes)
    {
        auto x = std::get<0>(key);
        auto y = std::get<1>(key);
        auto z = std::get<2>(key);

        if (x < x_min)
            x_min = x;
        if (y < y_min)
            y_min = y;
        if (z < z_min)
            z_min = z;

        if (x > x_max)
            x_max = x;
        if (y > y_max)
            y_max = y;
        if (z > z_max)
            z_max = z;
    }

    auto vox_min_range = std::make_tuple(x_min, y_min, z_min);
    auto vox_max_range = std::make_tuple(x_max, y_max, z_max);
    std::optional<Eigen::Vector3i> vox_focus_range =
        detect_window.has_value()
            ? std::optional((detect_window.value().array() / this->octotree_obj->cell_interval.array())
                                .ceil()
                                .cast<int>()
                                .matrix()
                                .eval())
            : std::nullopt;

    // srcの点群を膨張させる, for文内で毎回実行すると非効率なため、外に出した
    std::set<T> src_detect_coords = this->create_dialation_coord(src_detect_nodes, window);

    std::map<NodeClusterKey, OctoMap> dest_nodes =
        this->octotree_obj->collect_by_clusters_with_key(dest_entity, dest_labels);

    const Eigen::MatrixXi src_coords = this->_keynode2array(src_measure_nodes);
    const bool src_use_median = this->_should_use_median_for_nodes(src_measure_nodes);

    ClusterColMap collision_pairs;
    for (const auto& [entity_cluster_key, target_dest_nodes] : dest_nodes)
    {
        // 衝突判定対象となる範囲が存在する場合は、除外条件を追加
        OctoMap filtered_dest_nodes;
        const OctoMap* dialation_nodes;
        if (vox_focus_range)
        {
            const Eigen::Vector3i focus = vox_focus_range.value();
            for (const auto& [key, value] : target_dest_nodes)
            {
                // xyzのいずれかは、機体を囲う直方体 +-
                // focusのxyzに入っていないものは可能性探索から除外する
                if (!(std::get<0>(vox_min_range) - focus(0) <= std::get<0>(key) &&
                      std::get<0>(key) <= std::get<0>(vox_max_range) + focus(0) &&
                      std::get<1>(vox_min_range) - focus(1) <= std::get<1>(key) &&
                      std::get<1>(key) <= std::get<1>(vox_max_range) + focus(1) &&
                      std::get<1>(vox_min_range) - focus(1) <= std::get<1>(key) &&
                      std::get<1>(key) <= std::get<1>(vox_max_range) + focus(1)))
                {
                    continue;
                }
                filtered_dest_nodes.insert_or_assign(key, value);
            }
            dialation_nodes = &filtered_dest_nodes;
        }
        else
        {
            dialation_nodes = &target_dest_nodes;
        }

        // dest側の一つのクラスタの点群をwindow分だけ膨張させる
        std::set<T> dest_coords = this->create_dest_coord(*dialation_nodes, window);

        // 衝突判定を行う
        if (this->collision_detection(src_detect_coords, dest_coords))
        {
            // srcと見ているdest_labelの中で衝突する可能性があるので、点群のどこが衝突しそうか計算する
            Eigen::MatrixXi dest_coords_arr = this->_keynode2array(target_dest_nodes);
            const bool dest_use_median = this->_should_use_median_for_nodes(target_dest_nodes);
            const bool use_median = src_use_median || dest_use_median;
            Eigen::MatrixXd XA = this->_get_vox_w_coords(src_measure_nodes, src_coords, use_median);
            Eigen::MatrixXd XB = this->_get_vox_w_coords(target_dest_nodes, dest_coords_arr, use_median);
            auto [min_dist, minRow, minCol] = helper::calc_cdist_min(XA, XB);
            const auto& minimum_pair =
                this->_build_min_pair_result(src_measure_nodes, target_dest_nodes, src_coords, dest_coords_arr,
                                             min_dist, minRow, minCol, distance_threshold);

            if (minimum_pair.has_value())
            {
                // 見ているラベルの組の最小距離がdistance_threshold以下の場合、衝突の組に追加
                collision_pairs.insert_or_assign(entity_cluster_key.cluster_id, minimum_pair.value());
            }
        }
    }
    return collision_pairs;
}

//

template <typename T>
Eigen::MatrixXi AbstractCollisionDetector<T>::_keynode2array(const std::map<VoxelCoord, OctoNode>& keynodes)
{
    Eigen::MatrixXi key_coords(int(keynodes.size()), 3);
    int key_nodes_i = 0;
    for (const auto& [key, value] : keynodes)
    {
        key_coords(key_nodes_i, 0) = std::get<0>(key);
        key_coords(key_nodes_i, 1) = std::get<1>(key);
        key_coords(key_nodes_i, 2) = std::get<2>(key);
        key_nodes_i++;
    }
    return key_coords;
}

template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_mean2array(const std::map<VoxelCoord, OctoNode>& keynodes)
{
    /*
    get_mean()を使って処理しようとしたがポインタとかの扱いが上手くできずエラーになったので、
    構造体を直接操作して処理しているが、余裕があれば直す
    quantileに対してほぼ同じ処理をするが、templateとかを上手く使えないので、ほぼ同じ関数を複数作成するが、いずれ直したい
    */
    Eigen::MatrixXd node_coords(int(keynodes.size()), 3);
    const auto* stats = keynodes.begin()->second.node_stats.get();
    if (stats == nullptr || stats->first_moment.is_null)
    {
        // first_momentが値を持っていない場合は空のarrayを返す
        return node_coords;
    }

    int array_i = 0;
    for (const auto& [key, value] : keynodes)
    {
        if (!value.node_stats)
        {
            continue;
        }
        Point mean = value.node_stats->first_moment;
        node_coords(array_i, 0) = mean.x;
        node_coords(array_i, 1) = mean.y;
        node_coords(array_i, 2) = mean.z;

        array_i++;
    }
    return node_coords;
}

template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_far2array(const std::map<VoxelCoord, OctoNode>& keynodes)
{
    Eigen::MatrixXd node_coords(int(keynodes.size()), 3);
    const auto* stats = keynodes.begin()->second.node_stats.get();
    if (stats == nullptr || stats->far_point.is_null)
    {
        // far_pointが値を持っていない場合は空のarrayを返す
        return node_coords;
    }

    int array_i = 0;
    for (const auto& [key, value] : keynodes)
    {
        if (!value.node_stats)
        {
            continue;
        }
        Point far_point = value.node_stats->far_point;
        node_coords(array_i, 0) = far_point.x;
        node_coords(array_i, 1) = far_point.y;
        node_coords(array_i, 2) = far_point.z;

        array_i++;
    }
    return node_coords;
}

template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_near2array(const std::map<VoxelCoord, OctoNode>& keynodes)
{
    Eigen::MatrixXd node_coords(int(keynodes.size()), 3);
    const auto* stats = keynodes.begin()->second.node_stats.get();
    if (stats == nullptr || stats->near_point.is_null)
    {
        // near_pointが値を持っていない場合は空のarrayを返す
        return node_coords;
    }

    int array_i = 0;
    for (const auto& [key, value] : keynodes)
    {
        if (!value.node_stats)
        {
            continue;
        }
        Point near_point = value.node_stats->near_point;
        node_coords(array_i, 0) = near_point.x;
        node_coords(array_i, 1) = near_point.y;
        node_coords(array_i, 2) = near_point.z;

        array_i++;
    }
    return node_coords;
}

template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_quantile2array(const std::map<VoxelCoord, OctoNode>& keynodes)
{
    Eigen::MatrixXd node_coords(int(keynodes.size()), 3);
    const auto* stats = keynodes.begin()->second.node_stats.get();
    if (stats == nullptr || stats->quantile.is_null)
    {
        // quantileが値を持っていない場合は空のarrayを返す
        return node_coords;
    }

    int array_i = 0;
    for (const auto& [key, value] : keynodes)
    {
        if (!value.node_stats)
        {
            continue;
        }
        Point quantile = value.node_stats->quantile;
        node_coords(array_i, 0) = quantile.x;
        node_coords(array_i, 1) = quantile.y;
        node_coords(array_i, 2) = quantile.z;

        array_i++;
    }
    return node_coords;
}

template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_med2array(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords)
{
    if (this->octotree_obj == nullptr)
    {
        throw std::invalid_argument("八分木が設定されていないため、八分木の中心座標を計算できません。先"
                                    "に、assign_octotreeメソッドで八分木を設定してください。");
    }
    return this->octotree_obj->vox2w_coords(vox_coords);
}

/**
 * @brief なんか増えていたコード
 *
 * @tparam T
 * @param nodes
 * @return true
 * @return false
 */
template <typename T>
bool AbstractCollisionDetector<T>::_should_use_median_for_nodes(const std::map<VoxelCoord, OctoNode>& nodes) const
{
    if (!nodes.begin()->second.node_stats)
    {
        return true;
    }
    if (this->coord_method == CoordMethod::VOX_MED)
    {
        return true;
    }
    if (this->coord_method == CoordMethod::QUANTILE)
    {
        return nodes.begin()->second.node_stats->quantile.is_null;
    }
    return false;
}

/**
 * @brief srcとdestの離散座標と実座標の組を取り出す
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param src_nodes src側のと散座標と八分木ノードの組
 * @param dest_nodes dest側のと散座標と八分木ノードの組
 * @return std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> (srcの離散座標, srcの実座標,
 * destの離散座標, destの実座標)の組
 */
template <typename T>
Eigen::MatrixXd AbstractCollisionDetector<T>::_get_vox_w_coords(const std::map<VoxelCoord, OctoNode>& nodes,
                                                                const Eigen::Ref<const Eigen::MatrixXi>& vox_coords,
                                                                bool use_median)
{
    if (use_median)
    {
        return this->_med2array(vox_coords);
    }

    switch (this->coord_method)
    {
    case CoordMethod::MEAN:
        return this->_mean2array(nodes);
    case CoordMethod::FAR_POINT:
        return this->_far2array(nodes);
    case CoordMethod::NEAR_POINT:
        return this->_near2array(nodes);
    case CoordMethod::QUANTILE:
        return this->_quantile2array(nodes);
    case CoordMethod::VOX_MED:
        return this->_med2array(vox_coords);
    default:
        // num型が増えた場合にエラーにならないようにするため念のため置いておく
        return this->_med2array(vox_coords);
    }
}

template <typename T>
std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd>
AbstractCollisionDetector<T>::_get_vox_w_pairs(const std::map<VoxelCoord, OctoNode>& src_nodes,
                                               const std::map<VoxelCoord, OctoNode>& dest_nodes)
{
    /** 2つの点群の集合に対して必要な形式でLiDAR座標上の点群を取得する関数
     */
    Eigen::MatrixXi src_vox_coords = this->_keynode2array(src_nodes);
    Eigen::MatrixXi dest_vox_coords = this->_keynode2array(dest_nodes);

    const bool src_use_median = this->_should_use_median_for_nodes(src_nodes);
    const bool dest_use_median = this->_should_use_median_for_nodes(dest_nodes);
    const bool use_median = src_use_median || dest_use_median;
    Eigen::MatrixXd src_w_coords = this->_get_vox_w_coords(src_nodes, src_vox_coords, use_median);
    Eigen::MatrixXd dest_w_coords = this->_get_vox_w_coords(dest_nodes, dest_vox_coords, use_median);
    return {src_vox_coords, src_w_coords, dest_vox_coords, dest_w_coords};
}

/**
 * @brief なんか増えていたコード
 *
 * @tparam T
 * @param src_nodes
 * @param dest_nodes
 * @return AbstractCollisionDetector<T>::MinPairCalc
 */
template <typename T>
typename AbstractCollisionDetector<T>::MinPairCalc
AbstractCollisionDetector<T>::_calc_min_pair(const std::map<VoxelCoord, OctoNode>& src_nodes,
                                             const std::map<VoxelCoord, OctoNode>& dest_nodes)
{
    MinPairCalc result;
    std::tie(result.src_coords, result.XA, result.dest_coords, result.XB) =
        this->_get_vox_w_pairs(src_nodes, dest_nodes);
    std::tie(result.min_dist, result.minRow, result.minCol) = helper::calc_cdist_min(result.XA, result.XB);
    return result;
}

/**
 * @brief なんか増えていたコード
 *
 * @tparam T
 * @param src_nodes
 * @param dest_nodes
 * @param src_coords
 * @param dest_coords
 * @param min_dist
 * @param minRow
 * @param minCol
 * @param distance_threshold
 * @return std::optional<CollisionDetResult>
 */
template <typename T>
std::optional<CollisionDetResult> AbstractCollisionDetector<T>::_build_min_pair_result(
    const std::map<VoxelCoord, OctoNode>& src_nodes, const std::map<VoxelCoord, OctoNode>& dest_nodes,
    const Eigen::MatrixXi& src_coords, const Eigen::MatrixXi& dest_coords, double min_dist, int minRow, int minCol,
    const std::optional<double>& distance_threshold)
{
    // distance_thresholdが値を持っていて、srcとdestの距離が閾値以上の場合、処理を切り上げる
    if (distance_threshold.has_value() && min_dist > distance_threshold.value())
    {
        return std::nullopt;
    }

    // 距離行列の最小の組のindexを見つける
    Eigen::Vector2i min_src_dest_ind(minRow, minCol);

    // indexからOctoNodeインスタンスを取り出して結果を返す
    auto min_src_octonode = src_nodes.at(
        {src_coords(min_src_dest_ind(0), 0), src_coords(min_src_dest_ind(0), 1), src_coords(min_src_dest_ind(0), 2)});

    auto min_dest_octonode = dest_nodes.at({dest_coords(min_src_dest_ind(1), 0), dest_coords(min_src_dest_ind(1), 1),
                                            dest_coords(min_src_dest_ind(1), 2)});

    Eigen::MatrixXi vox_coords(2, 3);
    auto min_vox_coord = OctoNode::morton_decode_3d(min_src_octonode.morton_code);
    auto max_vox_coord = OctoNode::morton_decode_3d(min_dest_octonode.morton_code);
    vox_coords(0, 0) = std::get<0>(min_vox_coord);
    vox_coords(0, 1) = std::get<1>(min_vox_coord);
    vox_coords(0, 2) = std::get<2>(min_vox_coord);
    vox_coords(1, 0) = std::get<0>(max_vox_coord);
    vox_coords(1, 1) = std::get<1>(max_vox_coord);
    vox_coords(1, 2) = std::get<2>(max_vox_coord);
    auto min_w_coord = this->octotree_obj->vox2w_coords(vox_coords);

    return CollisionDetResult{min_src_octonode, min_dest_octonode,
                              std::tuple(min_w_coord(0, 0), min_w_coord(0, 1), min_w_coord(0, 2)),
                              std::tuple(min_w_coord(1, 0), min_w_coord(1, 1), min_w_coord(1, 2)), min_dist};
}

/**
 * @brief srcとdestのの最短部位のを行う
 *
 * @tparam T 衝突可能性を扱う座標の型
 * @param src_nodes
 * @param dest_nodes
 * @param distance_threshold
 * @param metric
 * @return std::optional<CollisionDetResult>
 */
template <typename T>
std::optional<CollisionDetResult> AbstractCollisionDetector<T>::find_minimum_node_pair(
    const std::map<VoxelCoord, OctoNode>& src_nodes, const std::map<VoxelCoord, OctoNode>& dest_nodes,
    std::optional<double> distance_threshold,
    const std::optional<std::function<double(Eigen::MatrixXd, Eigen::MatrixXd)>> metric)
{
    // distance_thresholdがnullの場合、後続の判定に行く前に条件文を抜けてしまい、minRow,
    // minColがundefinedだったので、前に出した
    // 最小距離に関する情報のみ使用するため、距離行列を作成しない
    const auto min_pair = this->_calc_min_pair(src_nodes, dest_nodes);

    return this->_build_min_pair_result(src_nodes, dest_nodes, min_pair.src_coords, min_pair.dest_coords,
                                        min_pair.min_dist, min_pair.minRow, min_pair.minCol, distance_threshold);
}
