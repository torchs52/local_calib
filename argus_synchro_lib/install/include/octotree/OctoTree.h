#pragma once

#include "NodeEntity.h"
#include "OctoNode.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <variant>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

struct TupleHash
{
    std::size_t operator()(const VoxelCoord& key) const noexcept
    {
        std::size_t hash1 = std::hash<int>{}(std::get<0>(key));
        std::size_t hash2 = std::hash<int>{}(std::get<1>(key));
        std::size_t hash3 = std::hash<int>{}(std::get<2>(key));
        std::size_t seed = hash1;

        seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct NodeEntityHash
{
    std::size_t operator()(const NodeEntity& value) const noexcept
    {
        using UnderT = std::underlying_type_t<NodeEntity>;
        return std::hash<UnderT>{}(static_cast<UnderT>(value));
    }
};

template <typename ValueType> using VoxelCoordMap = std::unordered_map<VoxelCoord, ValueType, TupleHash>;

/**
 * @brief 離散座標を表現するエイリアス
 *
 */
using VoxelCoord = std::tuple<int, int, int>;

/**
 * @brief 離散座標と八分木ノードのペアを表すエイリアス
 *
 */
using VoxelNodePair = std::pair<VoxelCoord, OctoNode>;

/**
 * @brief 順序なしのvoxel座標と対応するOctoNodeのマップ
 *
 */
using OctoMap = std::map<VoxelCoord, OctoNode>;

/**
 * @brief 順序付きのOctoMap
 *
 */
using OctoQueue = std::deque<VoxelNodePair>;

/**
 * @brief voxelとOctoNodeの組が入った構造の型
 *
 */
using ChunkOctoNodes = std::variant<OctoMap, OctoQueue>;

/**
 * @brief NodeEntity + クラスタ番号の組に対して、voxel+OctoNodeの組のマップが対応づいている型
 *
 */
using EntityMap = std::map<NodeClusterKey, // node entity and cluster number
                           ChunkOctoNodes  // corresponding octo nodes
                           >;

/**
 * @brief VoxelCoordをunordered_setの要素に指定するため、hash化する構造体を定義
 *
 */
struct VoxelCoordHash
{
    std::size_t operator()(const VoxelCoord& v) const noexcept
    {
        auto [x, y, z] = v;
        std::size_t h1 = std::hash<int>{}(x);
        std::size_t h2 = std::hash<int>{}(y);
        std::size_t h3 = std::hash<int>{}(z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

class OctoTree
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // octo_nodesで機体を表すkey,
    // NodeEntityに似た定数があるので、何れ修正したい
    static const std::string MACHINE_DETECT_LABEL;
    static const std::string MACHINE_MEASURE_LABEL;

    Eigen::Vector3d max_xyz;       // 八分木の対象となる点群の最大値
    Eigen::Vector3d min_xyz;       // 八分木の対象となる点群の最小値
    Eigen::Vector3d cell_interval; // 八分木セルの間隔
    int max_tree_depth;            // 八分木の最大深さ
    bool use_node_stats;           // 八分木ノードに統計量を持たせるかどうか,
                         // trueの場合八分木ノードに統計量を持たせ、falseの場合Noneを入れる予定だが、まだ準備中
    std::optional<float> quantile; // 八分木統計量としてquantitleを用いる場合のパラメータ

    // クラスタリングされた八分木の情報が入った辞書,
    // Remark: 最初は使っていたが、entity_octonodesを使うことにしてから不要になったので、消した方が良い
    std::optional<std::map<std::variant<int, std::string>, std::map<VoxelCoord, OctoNode>>> labeled_octo_nodes;

    // (クラスタ番号, NodeEntity)をkeyとして八分木ノードがvalue側に入っている辞書
    EntityMap entity_octonodes;

    // クラスタリング実施前の点群の辞書
    // Remark: 最初は使っていたが、entity_octonodesを使うことにしてから不要になったので、消した方が良い
    std::optional<std::map<VoxelCoord, OctoNode>> unlabeled_octo_nodes;

    // 機体点群用に用いる離散点群の最小値, 最大値の座標,
    // LiDARの八分木では使わないので、機体の八分木クラスを別途作って対応したほうが良いが、ちょっと時間がかかるので、八分木一般のクラスに追加
    // Todo: 機体は機体で新しいクラスを作って、固有の属性は持つようにする

    // クラスタリングで用いた離散座標と八分木の最下層の離散座標の対応表,
    // クラスタリングは最下層以上で行われるので、クラスタリングの座標をkeyにして、1体多の辞書を作る
    std::map<VoxelCoord, std::vector<VoxelCoord>> cluster_vox2deepest_vox;

    // クラスタリング用に作ったデータの階層とラベル結果を入れようとしている階層が同じかどうか,
    //  異なっていると正しい結果にならないので、例外を出す
    std::optional<int> _clustering_tree_depth;

    // 八分木における原点
    // Remark: 衝突判定の関係で八分木の原点をずらすのに使っていたが今は使っていない
    std::optional<Eigen::Vector3d> origin_w2oct;

    /**
     * @brief 八分木を表すクラス 八分木の各ノードを持っているが、衝突判定はこのクラスでは行わない
     * @param xyz : 前工程から与えられる点群の座標, インスタンス生成時に入れることもできる
     * @param max_xyz : 計算対象となる点群座標の最大値
     * @param min_xyz : 計算対象となる点群座標の最小値
     * @param max_treed_epth : 木の深さ
     * @param xyz_entity xyzのNodeEntityを表す
     * @param use_node_stats 八分木ノードの中の点群に対して統計量を計算するかどうか
     * @param quantile 統計量としてquantileを計算する場合の何パーセントのquantileを用いるかを表す
     * @param origin_w2oct 八分木原点, nullの場合はLiDAR座標と同じ
     */
    OctoTree(const std::optional<Eigen::MatrixXd>& xyz = std::nullopt,
             const Eigen::Ref<const Eigen::Vector3d>& max_xyz = Eigen::Vector3d(18, 20, 20),
             const Eigen::Ref<const Eigen::Vector3d>& min_xyz = Eigen::Vector3d(-12, -10, -10),
             double max_tree_depth = 7, NodeEntity xyz_entity = NodeEntity::UNK, bool use_node_stats = false,
             std::optional<float> quantile = std::nullopt,
             const std::optional<Eigen::Vector3d>& origin_w2oct = std::nullopt);

    /**
     * @brief tree_depthがmax_tree_depthに対して、どれくらい上の階層か計算して、必要であれば例外を出す
     *
     * @param tree_depth 一番下の階層から何階層登るか
     * @return int 計算で用いる階層
     */
    int _get_tree_diff(std::optional<int> tree_depth = std::nullopt) const;

    /**
     * @brief Create a octonodes object,
     * @remark detectable_points.pyで使っているので、必要だが、そっちを修正したほうが良さそう
     *
     * @param xyz
     * @param entity
     * @param removed_vox_min_points
     * @param removed_vox_max_points
     * @param remove_dist
     * @return OctoTree&
     */
    OctoTree&
    create_octonodes(const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity = NodeEntity::UNK,
                     const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                         removed_vox_min_points = std::nullopt,
                     const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                         removed_vox_max_points = std::nullopt,
                     int remove_dist = 1);

    /**
     * @brief unlabeled_octonodesが存在しなければ与えられたxyz座標を基にunlabeled_octonodesを生成,
     * 存在すれば、与えられたxyz座標から生成されるoctonodesと同じkeyを持つものは置き換えられる
     * unlabeled_octonodesのあるkeyに対するentityは一定で、新しい方を使う
     * @deprecated entity_octonodesに変えてから使っていない
     */
    OctoTree& insert_or_create_octonodes(
        const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity = NodeEntity::UNK,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_min_points = std::nullopt,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_max_points = std::nullopt,
        int remove_dist = 1);

    /*離散な直方体の最小値と最大値が与えられたときに、その外周の離散座標の集合を計算する
     */
    static std::unordered_set<VoxelCoord, TupleHash> get_cuboid_boundary(int min_x, int min_y, int min_z, int max_x,
                                                                         int max_y, int max_z, int step = 2);

    /**
     * @brief 八分木ノードを生成するために、範囲外の点群を除去して除去後の離散座標と実座標のペアを返すメソッド
     *
     * @param xyz 対象となる点群
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param removed_vox_max_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param remove_dist removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param remove_dup 同じ離散座標を持つ点を除外するかどうか
     * @return std::pair<Eigen::MatrixXi, Eigen::MatrixXd 離散座標と実座標
     */
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd>
    _gen_vox_for_octonodes(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                           const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                               removed_vox_min_points = std::nullopt,
                           const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
                               removed_vox_max_points = std::nullopt,
                           int remove_dist = 1, bool remove_dup = true);

    /**
     * @brief 点群を八分木に入れるための形式に変換するメソッド, 統計量の計算を行う場合に用いる
     * @details 八分木ノードを作る過程で、各ノードの実座標における統計量も計算する
     * @remarks
     * 引数で統計量の計算の有無を切り替えられるようにしていたが、統計量の計算をしない場合の処理速度が遅くなったので、呼ぶ関数を切り替えることで処理を行うようにした,
     * @remarks
     * 統計量の計算をしないほうが早いので、統計量を使って後続処理をしない場合は、_gen_octonodes_without_statsを読んだ方が良い
     * @remarks
     * genericsを使っているのは、ノードをmapで保持する場合と、queueで保持する場合があって、入れ物が違う以外は同じ処理になるため
     *
     * @param xyz 八分木に入れる点群
     * @param entity xyzに紐づけるNodeEntity
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param quantile 統計量としてquantileを計算する場合の設定する, nullだと計算いない
     * @return Container OctoMapやOctoQueueを想定したもの, 離散座標と八分木のノードの組のコレクション
     */
    template <typename Container>
    Container _gen_octonodes_with_stats(
        const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity = NodeEntity::UNK,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_min_points = std::nullopt,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_max_points = std::nullopt,
        int remove_dist = 1, std::optional<float> quantile = std::nullopt);

    /**
     * @brief 点群を八分木に入れるための形式に変換するメソッド, 統計量の計算を行う場合に用いる
     * @details 八分木ノードを作る過程で、各ノードの実座標における統計量も計算する
     * @remarks
     * 引数で統計量の計算の有無を切り替えられるようにしていたが、統計量の計算をしない場合の処理速度が遅くなったので、呼ぶ関数を切り替えることで処理を行うようにした,
     * @remarks
     * 統計量の計算をしないほうが早いので、統計量を使って後続処理をしない場合は、_gen_octonodes_without_statsを読んだ方が良い
     * @remarks
     * genericsを使っているのは、ノードをmapで保持する場合と、queueで保持する場合があって、入れ物が違う以外は同じ処理になるため
     *
     * @param xyz 八分木に入れる点群
     * @param entity xyzに紐づけるNodeEntity
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @param removed_vox_min_points
     * removed_vox_min_points-remove_distからremoved_vox_max_points+remove_distの点群は除外する,
     * nullの場合はこの条件で除外しない
     * @return Container OctoMapやOctoQueueを想定したもの, 離散座標と八分木のノードの組のコレクション
     */
    template <typename Container>
    Container _gen_octonodes_without_stats(
        const Eigen::Ref<const Eigen::MatrixXd>& xyz, NodeEntity entity = NodeEntity::UNK,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_min_points = std::nullopt,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_max_points = std::nullopt,
        int remove_dist = 1);

    /**
     * @brief 離散化されたxyz座標群を八分木座標に変換する vox_coordsが空の場合は、空の配列を返す
     * @param voc_coords: 離散化されたxyz座標の集合, tree_depthにおける
     * @param tree_depth: 実座標に変換するときの木の深さ, nullの場合max_tree_depthを使う
     * @return Eigen::MatrixXd 八分木座標 n*3
     */
    Eigen::MatrixXd vox2oct_coords(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords,
                                   std::optional<int> tree_depth = std::nullopt);

    /**
     * @brief 点群を離散化されたxyz座標群に変換、重複座標を除去する
     * @param tree_depth: 点群をどの階層で離散化されたxyzに変換するか, nullの場合、max_tree_depthを基に離散化を行う
     * @param remove_dup: 離散化されたxyzの重複を削除するかどうか,Trueの場合、重複削除を行う
     */
    Eigen::MatrixXi w2vox_coords(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                                 std::optional<int> tree_depth = std::nullopt, bool remove_dep = true);
    /**
     * @brief 点群を離散化された八分木座標群に変換、重複座標を除去する
     * @param tree_depth: 点群をどの階層で離散化されたxyzに変換するか,nullの場合、max_tree_depthを基に離散化を行う
     * @param remove_dup: 離散化されたxyzの重複を削除するかどうか, Trueの場合、重複削除を行う
     * @return Eigen::MatrixXi 離散座標 n*3
     * @remark
     * remove_dup=Trueの場合、得られる結果は適当に整列されるため、oct_coordsの各要素の順序を維持したい場合は、remove_dup=Falseにする必要がある
     */
    Eigen::MatrixXi oct2vox_coords(const Eigen::Ref<const Eigen::MatrixXd>& oct_coords,
                                   std::optional<int> tree_depth = std::nullopt, bool remove_dep = true) const;
    /*離散座標から実座標への変換を行う関数
    元々のvox2w_coordsがvox2oct_coordsとして外部に公開していたので、その整合性を持たせるために新しく作成
    八分木座標というものを新しく考えて、離散座標 <-> 八分木座標 <-> 実座標,
    という変換をするようにしたので、この関数の中で離散座標から八分木座標を経由して実座標に変換している
    */
    Eigen::MatrixXd vox2w_coords(const Eigen::Ref<const Eigen::MatrixXi>& vox_coords,
                                 std::optional<int> tree_depth = std::nullopt);

    /** labeled/unlabeledを使った座標変換系のメソッド
    いずれ消した方が良いが、Pythonコード側に微妙に残っていて消せないので置いておく
    どこかのタイミングで確認してそれぞれ消した方が良い
     */
    // ******* 以下がそのメソッド
    /** unlabeled_octonodesに入っている点群を離散座標で返す関数
     */
    Eigen::MatrixXi get_octonodes_vox_coord_unlabled(std::optional<int> tree_depth = std::nullopt) const;

    /**
     *  八分木ノードから八分木座標を返すメソッド
     *  クラスタリング前の時点のtree_depthにおけるxyz座標を返す
     *  Remark: 最初は使っていたが、entity_octonodesを使うことにしてから不要になったので、消した方が良い
     */
    Eigen::MatrixXd get_octonodes_oct_coord_unlabeled(std::optional<int> tree_depth = std::nullopt);

    /**
     * 八分木ノードから各点群の座標を返すメソッド
     * クラスタリング前の時点のtree_depthにおけるxyz座標を返す
     *  Remark: 最初は使っていたが、entity_octonodesを使うことにしてから不要になったので、消した方が良い
     */
    Eigen::MatrixXd get_octonodes_np_coord_unlabeled(std::optional<int> tree_depth = std::nullopt);

    Eigen::MatrixXd
    get_octonodes_np_coord_labeled_v2(const std::optional<std::vector<int>>& target_labels = std::nullopt,
                                      std::optional<int> tree_depth = std::nullopt);
    /*
     * 八分木ノードから各点群の座標を返すメソッド
     * octo_nodesが作られる前に読んだ場合、空行列を返す
     */
    Eigen::MatrixXd get_octonodes_np_coord_labeled(const std::optional<std::vector<int>>& target_labels = std::nullopt,
                                                   std::optional<int> tree_depth = std::nullopt);

    /*
        クラスタリング結果を反映する
        {ラベル番号, {離散座標, OctoNode}}という形式で保持しようとしているため、
        クラスタリングで使った座標を離散座標に変換して、ラベルごとにunlabledの該当離散座標群の辞書を作成する

        引数:
            - clustered_data: クラスタリングで使ったデータ, 実座標(n*M)の行列
            - labels: クラスタリングの結果, n行配列
                - clusterd_dataとlabelsのlengthは同じである必要がある

        Todo:
       クラスタリングで使ったデータの離散座標がunlabeledの離散座標のkeyと紐づく必要があるが、unlabledの上位階層を使ってクラスタリングする場合はプログラムの修正が必要
        */
    OctoTree& _insert_clustering_result_in_deepest_layer(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                         const Eigen::Ref<const Eigen::MatrixXi>& labels);

    OctoTree& insert_entity_result(const std::map<int, NodeEntity>& cluster2entity);

    // Todo to be deleted
    OctoTree& insert_entity_result_v2(const std::map<int, NodeEntity>& cluster2entity);

    /*max_tree_depthより上位の階層のデータを使ってクラスタリングを行った結果を反映する
     * labelsの各要素に紐づく、最下層のノードの離散座標を持っておく必要があるため、descendant_vox_coordsという引数が追加で必要
     * @retval labels クラスタリングの結果, n行配列
     * @retval descendant_vox_coords
     * クラスタリングデータの各要素に紐づく最下層の離散座標のリスト
     * - 例 : 2番目の要素は2つの離散座標が紐づいている
     * array([list([(0, 229, 79)]),
     * list([(0, 230, 83), (0, 231, 82), (1, 230, 82)]),
     * list([(0, 232, 81)]), ...,
     * list([(230, 155, 82), (231, 155, 82), (231, 155, 83)]),
     * list([(230, 197, 79)]), list([(243, 201, 79)])], dtype = object)
     */
    OctoTree& insert_clustering_result(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                       const Eigen::Ref<const Eigen::VectorXi>& labels);

    /* labeled_octo_nodesからunlabeled_octo_nodesのOctoNodeのクラスタラベルを変更するようにするための関数
    プログラムの修正中
     */
    OctoTree& _insert_clustering_result_in_deepest_layer_v2(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                            const Eigen::Ref<const Eigen::MatrixXi>& labels);
    OctoTree& insert_clustering_result_v2(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                          const Eigen::Ref<const Eigen::VectorXi>& labels);

    Eigen::MatrixXd get_octonodes_np_coord_entity(const std::vector<NodeEntity>& target_entities,
                                                  std::optional<int> tree_depth = std::nullopt);
    // ******* ここまでが対応箇所

    /**
     * @brief
     * クラスタリングで用いたデータとクラスタリングラベルに整合性が取れているかチェックして、問題があれば例外を投げる関数
     * @param clustered_data クラスタリングで用いたデータ
     * @param labels クラスタリングで得られたラベル
     */
    void _check_data_label_consistency(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                       const Eigen::Ref<const Eigen::VectorXi>& labels) const;

    /* LiDAR座標を八分木座標に変換する
    衝突判定において、原点をずらす必要が出てきたため、追加
    */
    Eigen::MatrixXd w2oct_coords(const Eigen::Ref<const Eigen::MatrixXd>& xyz) const;

    /*八分木座標をLiDAR座標に変換する*/
    Eigen::MatrixXd oct2w_coords(const Eigen::Ref<const Eigen::MatrixXd>& oct_coords) const;

    // entity_octonodes関連のメソッドの定義
    /**
     * @brief NodeEntityのみでentity_octonodesに格納する, 格納された点群は(null,
     * entity)をkeyとするentity_octonodesに格納される
     * @remark 立体物点群などを入れるので、その設定をデフォルト引数にしている
     * @param xyz: entity_octonodesに入れられる点群
     * @param entity: 格納するNodeEntity
     * @param entity_replace: 該当するentityのkeyを新しくするかどうか, trueの場合、(*,
     * entity)に該当するkeyは全て削除して、xyzが格納される, falseの場合、(null, entity)のkeyだけxyzに更新される
     * @param is_order: trueの場合、xyzの行順にデータが保持される
     * @param vox_min/max_points: それぞれnullでない場合、min/maxの範囲離散座標が入っていなければ除外される,
     * どちらかがnullの場合は除外する処理は行われない
     * @param remove_dist 除外する大きさ, 離散座標における大きさ
     * @return 八分木インスタンスそのもののメモリ番地を返す
     */
    OctoTree& insert_or_entity_octonodes(
        const Eigen::Ref<const Eigen::MatrixXd>& xyz, const NodeEntity& entity = NodeEntity::OTHER,
        bool entity_replace = true, bool is_order = false,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_min_points = std::nullopt,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_max_points = std::nullopt,
        int remove_dist = 1);

    /**
     * @brief NodeEntity + クラスタ番号毎にOctoNodeをentity_octonodesに格納する
     *
     * @param xyz entity_octonodesに入れられる点群, n*3行列
     * @param labels 各点群のクラスタ番号, nベクトル
     * @param entity 格納するNodeEntity
     * @param entity_replace 該当するentityのkeyを新しくするかどうか, trueの場合、(*,
     * entity)に該当するkeyは全て削除して、xyzが格納される, falseの場合、(null, entity)のkeyだけxyzに更新される
     * @param is_order trueの場合、xyzの行順にデータが保持される
     * @param removed_vox_min_points min/maxの範囲離散座標が入っていなければ除外される,
     * どちらかがnullの場合は除外する処理は行われない
     * @param removed_vox_max_points min/maxの範囲離散座標が入っていなければ除外される,
     * どちらかがnullの場合は除外する処理は行われない
     * @param remove_dist 除外する大きさ, 離散座標における大きさになっている
     * @return OctoTree& 八分木インスタンスそのもののメモリ番地を返す
     * @remark 現状は崖点群を入れるのに使うので、その設定をデフォルト引数にしている
     */
    OctoTree& insert_or_entity_octonodes_with_labels(
        const Eigen::Ref<const Eigen::MatrixXd>& xyz, const Eigen::Ref<const Eigen::VectorXi>& labels,
        const NodeEntity& entity = NodeEntity::CLIFF, bool entity_replace = false, bool is_order = true,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_min_points = std::nullopt,
        const std::optional<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>&
            removed_vox_max_points = std::nullopt,
        int remove_dist = 1);

    /**
     * @brief entity, cluster_idsに対応するentity_octonodesを切り出してkey, valueを保持したままで値を返すメソッド,
     * @remark entityは一つだけだが、cluster_idsは複数取り出すこともできる
     * @param entity 取り出したいNodeEntity
     * @param cluster_ids 取り出したいクラスタ番号, nullの場合entityに該当するもの全てを取り出す
     * @return std::map<NodeClusterKey, OctoMap> {(cluster_id, entity): value for cluster_id in cluster_ids}みたいな辞書
     */
    std::map<NodeClusterKey, OctoMap>
    collect_by_clusters_with_key(const NodeEntity& entity,
                                 const std::optional<std::vector<std::optional<int>>>& cluster_ids = std::nullopt);

    /**
     * @brief entityの中で、cluster_idsに該当するクラスタのChunkOctoNodesをOctoMapでひとまとめにする
     *
     * @param entity 取り出す対象のentity
     * @param cluster_ids 取り出す対象のクラスタ番号, nulloptの場合、該当するentity全てを取り出す
     * @return OctoMap 全部を重ね合わせたOctoMap
     */
    OctoMap collect_nodes_by_clusters(const NodeEntity& entity,
                                      const std::optional<std::vector<std::optional<int>>>& cluster_ids = std::nullopt);

    /**
     * @brief
     * entitiesに該当する八分木ノードでdepthだけ上に登った中でvox_rangesの範囲に入っているものを、登った階層で保持してまとめてOctoMapに変換してを返す
     *
     * @param entities 取り出す対象のentityのリスト
     * @param vox_ranges depthの高さでの離散座標の範囲, nullの場合何もしない
     * @param depth 一番下の階層から何階層登ったところで八分木ノードを取り出すかを表す整数, nullの場合一番下の階層
     * @return OctoMap 取り出した八分木ノードをまとめたOctoMap
     */
    OctoMap collect_nodes_by_entities_with_depth(const std::vector<NodeEntity>& entities,
                                                 const std::vector<std::optional<std::tuple<int, int>>>& vox_ranges,
                                                 std::optional<int> depth = std::nullopt);

    /**
     * @brief entitiesとclusters_idsに該当するクラスタをOctoMapでひとまとめにする
     *
     * @param 各entitiesのcluster_idsを取り出して一つにまとめてOctoMapにして返す
     * @param cluster_ids
     * @return OctoMap
     */
    OctoMap collect_nodes_by_entities_and_clusters(
        const std::vector<NodeEntity>& entities,
        const std::optional<std::vector<std::optional<int>>>& cluster_ids = std::nullopt);

    /**
     * @brief target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層でまとめて実座標に変換して返す
     *
     * @param target_entities 取り出す対象のentityのリスト
     * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
     * @return Eigen::MatrixXd 実座標の点群 n*3
     */
    Eigen::MatrixXd get_np_from_entity_octonodes_by_chunk(const std::vector<NodeEntity>& target_entities,
                                                          std::optional<int> tree_depth = std::nullopt);

    // Eigen::MatrixXd _get_np_from_entity_octonodes_by_chunk(
    //     const std::vector<NodeEntity> &target_entities,
    //     std::optional<int> tree_depth = std::nullopt);

    /**
     * @brief
     * target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層でまとめて離散座標に変換して返す
     *
     * @param target_entities 取り出す対象のentityのリスト
     * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
     * @return Eigen::MatrixXi 離散座標の点群 n*3
     */
    Eigen::MatrixXi get_vox_from_entity_octonodes_by_chunk(const std::vector<NodeEntity>& target_entities,
                                                           std::optional<int> tree_depth = std::nullopt);

    /**
     * @brief target_entitiesに該当するNodeEntityを持つentity_octonodesをtree_depthの階層で実座標に変換して返す,
     * key毎にリストで持つ場合に使うメソッド
     *
     * @param target_entities 取り出す対象のentityのリスト
     * @param tree_depth 一番下の階層から何階層登るか, nullの場合一番下の階層でデータを取り出す
     * @return std::vector<Eigen::MatrixXd> keyの数の要素を持つ配列で、各要素に実座標の点群が入っている
     */
    std::vector<Eigen::MatrixXd> get_np_from_entity_octonodes_by_list(const std::vector<NodeEntity>& target_entities,
                                                                      std::optional<int> tree_depth = std::nullopt);

    /**
     * @brief cluster_entityのクラスタ未割当のChunkOctoNodesを取得し、cluster_depthの階層でLiDAR点群にして返す
     * @remark
     * 普通のget_np...と異なる形で定義しているのは、クラスタリング結果を必要な階層で八分木ノードに入れる必要があり、その辺りの情報を作る部分があるため
     *
     * @param cluster_entity クラスタリング対象となるNodeEntity
     * @param cluster_depth 一番下の階層から何階層上でクラスタリングを行うかを表す整数
     * @return Eigen::MatrixXd クラスタリングに用いる実座標の点群 n*3
     */
    Eigen::MatrixXd get_clustering_data_by_entity(NodeEntity cluster_entity,
                                                  std::optional<int> cluster_depth = std::nullopt);

    /**
     * @brief cluster_entityでクラスタが未割当のものを削除して、削除したものを返す
     * @param cluster_entity 削除されるNodeEntity
     * @return OctoMap 削除されたOctoMap
     */
    OctoMap pop_unlabeled_nodes(NodeEntity cluster_entity);

    /**
     * @brief entitiesに該当するkeyを削除する
     * @param entities 削除されるNodeEntityのリスト
     */
    void erase_nodes_for_entities_noret(const std::vector<NodeEntity>& entities);

    /**
     * @brief entitiesに該当するentity_octonodesのChunkOctoNodesを削除する
     *
     * @param entities 削除されるNodeEntityのリスト
     * @return std::vector<ChunkOctoNodes> 削除されたもののリスト
     */
    std::vector<ChunkOctoNodes> erase_nodes_for_entities(const std::vector<NodeEntity>& entities);

    /**
     * @brief クラスタリング結果を基に、clustered_dataをcluster_entityに格納する,
     * 一番下の階層で各野する場合に呼ばれるメソッド
     * @param clustered_data クラスタリングに使った点群 n*3
     * @param labels クラスタリング結果 n次元ベクトル
     * @param cluster_entity クラスタリング結果を入れるNodeEntity
     * @return OctoTree& 八分木インスタンスそのもののメモリ番地
     */
    OctoTree&
    _insert_labels_and_move_in_octonodes_deepest_layer(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                       const Eigen::Ref<const Eigen::MatrixXi>& labels,
                                                       NodeEntity cluster_entity);

    /**
     * @brief cluster_entityのラベル未割当のChunkOctoNodesに対して、クラスタリングデータとラベルを基にラベルを付与する
     *
     * @param clustered_data クラスタリングに使った点群 n*3
     * @param labels クラスタリング結果 n次元ベクトル
     * @param cluster_entity クラスタリング結果を入れるNodeEntity
     * @param cluster_depth クラスタリングで用いた階層, nullの場合一番下の階層
     * @return OctoTree& 八分木インスタンスそのもののメモリ番地
     */
    OctoTree& insert_labeles_and_move_in_octonodes(const Eigen::Ref<const Eigen::MatrixXd>& clustered_data,
                                                   const Eigen::Ref<const Eigen::MatrixXi>& labels,
                                                   NodeEntity cluster_entity);

    /**
     * @brief entity_octonodesのfrom_keysのNodeEntity, cluster_idをto_keysのNodeEntity, cluster_idに置き換える,
     * 既にkeyが存在する場合は上書きする
     *
     * @param from_keys 置き換え前のNodeClusterKeyのリスト
     * @param to_keys 置き換え後のNodeClusterKeyのリスト
     * @return OctoTree&
     */
    OctoTree& replace_entities_in_octonodes(const NodeEntity from_entity,
                                            const std::map<std::optional<int>, NodeEntity>& cluster_transfered_entity
                                            // const NodeEntity to_entiy,
                                            // const std::optional<std::vector<std::optional<int>>>& clusters_opt
    );

    /**
     * @brief depthにおける最小、最大の離散座標を返す
     * @param depth
     * @return std::pair<VoxelCoord, VoxelCoord> 最小、最大の離散座標のpair
     */
    std::pair<VoxelCoord, VoxelCoord> get_min_max_coord(std::optional<int> depth = std::nullopt);

    /**
     * @brief lidar点群で、min_data, max_dataの外側の点群を取り除く
     * @param pcd_data 対象となるlidar点群
     * @param min/max_data 含める、最小と最大の範囲
     * @return Eigen::VectorXi 含める点群のindex
     */
    static Eigen::VectorXi limit_pcd_range(const Eigen::Ref<const Eigen::MatrixXd>& pcd_data,
                                           const Eigen::Ref<const Eigen::Vector3d>& min_data,
                                           const Eigen::Ref<const Eigen::Vector3d>& max_data);

    /**
     * @brief 各離散座標における統計量を計算して、VoxStatsにして結果を返す
     * @param vox_xyz 離散座標
     * @param w_xyz 実座標
     * @return 離散座標に対するVoxStatsの辞書
     */
    VoxelCoordMap<std::shared_ptr<VoxStats>> calc_vox_statistics(const Eigen::Ref<const Eigen::MatrixXi>& vox_xyz,
                                                                 const Eigen::Ref<const Eigen::MatrixXd>& w_xyz) const;

    /**
     * @brief 各離散座標のquantileを計算する
     * @param vox_xyz 離散座標
     * @param w_xyz 実座標
     * @param quantile 何パーセントの点を取るか
     * @param 各離散座標のquantileに該当する点の辞書
     */
    VoxelCoordMap<std::tuple<float, float, float>> calc_vox_quantile(const Eigen::Ref<const Eigen::MatrixXi>& vox_xyz,
                                                                     const Eigen::Ref<const Eigen::MatrixXd>& w_xyz,
                                                                     float quantile) const;
};
