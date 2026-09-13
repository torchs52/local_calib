#pragma once

#include "octotree/AbstractCollisionDetector.h"

/**
 * @brief 八分木の階層を変えることで接触可能性探索を行う場合に使うクラス
 *
 */
class LayerBasedCollisionDetector : public AbstractCollisionDetector<int>
{
  public:
    /**
     * @brief Construct a new Layer Based Collision Detector object
     *
     * @param coord_method 最短部位計算で用いるノードの代表点の選び方
     */
    LayerBasedCollisionDetector(CoordMethod coord_method = CoordMethod::VOX_MED);

    ~LayerBasedCollisionDetector();

    /**
     * @brief OctoNodeのリストのwindow分上位層のモートン順序の集合を取得する
     *
     * @param nodes OctoNodeインスタンスをiteratorに持つ変数
     * @param window モートン順序の何層上位までを取り出すかを指定するための変数
     * @return std::set<int> モートン順序の上位window層の集合
     */
    static std::set<int> _morton_codes_shift(const std::vector<OctoNode>& nodes, int window = 2);

    /**
     * @brief Create a dilation coord object
     *
     * @param nodes 離散座標に対する八分木ノードが入った辞書
     * @param window モートン順序の何層上位までを取り出すかを指定するための変数
     * @return std::set<int> モートン順序の上位window層の集合
     */
    std::set<int> create_dialation_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes, int window = 2);

    /**
     * @brief dest用の集合計算
     *
     * @param nodes 離散座標に対する八分木ノードが入った辞書
     * @param window モートン順序の何層上位までを取り出すかを指定するための変数
     * @return std::set<int> モートン順序の上位window層の集合
     */
    std::set<int> create_dest_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes, int window = 2);

    /**
     * @brief srcとdestの集合に対して、衝突可能性を判定する
     *
     * @param src_coords srcの集合
     * @param dest_coords destの集合
     * @return true srcとdestで衝突可能性がある
     * @return false 衝突可能性がない
     */
    bool collision_detection(const std::set<int>& src_coords, const std::set<int>& dest_coords);
};
