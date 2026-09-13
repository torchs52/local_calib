#pragma once

#include "AbstractCollisionDetector.h"

class NeighborBasedCollisionDetector : public AbstractCollisionDetector<std::tuple<int, int, int>>
{
  public:
    /**
     * @brief Construct a new Neighbor Based Collision Detector object
     *
     * @param coord_method 最短部位計算で用いるノードの代表点の選び方
     */
    NeighborBasedCollisionDetector(CoordMethod coord_method = CoordMethod::VOX_MED);

    ~NeighborBasedCollisionDetector();

    /**
     * @brief 各座標のwindow分だけ近傍の点の集合を取得する
        -> -window~+windowだけ各座標を変えた座標の組を各点で取り出す
     * @remark
     集合なので重複は含まないが、元々の個数から最悪(2*window+1)^3倍された個数の集合になるため、大きいwindowを入れる場合は、時間がかかるかもしれない
     * @param coords 対象とする座標, 離散座標の想定
     * @param window 近傍の大きさ
     * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
     */
    static std::set<std::tuple<int, int, int>>
    _get_neighbor_coords(const std::vector<std::tuple<int, int, int>>& coords, int window = 2);

    /**
     * @brief dest用の集合計算
     *
     * @param nodes 離散座標に対する八分木ノードが入った辞書
     * @param window 近傍の大きさ
     * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
     */
    std::set<std::tuple<int, int, int>>
    create_dialation_coord(const std::map<std::tuple<int, int, int>, OctoNode>& coords, int window = 2);

    /**
     * @brief Create a dest coord object
     *
     * @param nodes 離散座標に対する八分木ノードが入った辞書
     * @param window 近傍の大きさ
     * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
     */
    std::set<std::tuple<int, int, int>> create_dest_coord(const std::map<std::tuple<int, int, int>, OctoNode>& coords,
                                                          int window = 2);

    /**
     * @brief 離散座標の近傍ベースで衝突可能性を判定する
     *
     * @param src_coords srcの集合
     * @param dest_coords destの集合
     * @return true srcとdestで衝突可能性がある
     * @return false 衝突可能性がない
     */
    bool collision_detection(const std::set<std::tuple<int, int, int>>& src_coords,
                             const std::set<std::tuple<int, int, int>>& dest_coords);
};
