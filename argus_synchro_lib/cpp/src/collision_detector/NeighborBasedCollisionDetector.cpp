#include "octotree/NeighborBasedCollisionDetector.h"

NeighborBasedCollisionDetector::NeighborBasedCollisionDetector(
    // double distance_threshold,
    CoordMethod coord_method)
    : AbstractCollisionDetector(coord_method)
{
}

NeighborBasedCollisionDetector::~NeighborBasedCollisionDetector()
{
}

/**
 * @brief 各座標のwindow分だけ近傍の点の集合を取得する
    -> -window~+windowだけ各座標を変えた座標の組を各点で取り出す
 * @remark
 集合なので重複は含まないが、元々の個数から最悪(2*window+1)^3倍された個数の集合になるため、大きいwindowを入れる場合は、時間がかかるかもしれない
 * @param coords 対象とする座標, 離散座標の想定
 * @param window 近傍の大きさ
 * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
 */
std::set<std::tuple<int, int, int>>
NeighborBasedCollisionDetector::_get_neighbor_coords(const std::vector<std::tuple<int, int, int>>& coords, int window)
{
    std::set<std::tuple<int, int, int>> neightbor_coord;
    for (const auto& coord : coords)
    {
        // (x,y,z)方向にwindowだけ膨張させて、その座標を集合に入れていく
        for (int i0 = -window + std::get<0>(coord); i0 < std::get<0>(coord) + window + 1; i0++)
        {
            for (int i1 = -window + std::get<1>(coord); i1 < std::get<1>(coord) + window + 1; i1++)
            {
                for (int i2 = -window + std::get<2>(coord); i2 < std::get<2>(coord) + window + 1; i2++)
                {
                    neightbor_coord.insert({i0, i1, i2});
                }
            }
        }
    }

    return neightbor_coord;
}

/**
 * @brief dest用の集合計算
 *
 * @param nodes 離散座標に対する八分木ノードが入った辞書
 * @param window 近傍の大きさ
 * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
 */
std::set<std::tuple<int, int, int>>
NeighborBasedCollisionDetector::create_dialation_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes,
                                                       int window)
{
    std::vector<std::tuple<int, int, int>> key_list;
    for (const auto& [key, value] : nodes)
    {
        key_list.push_back(key);
    }
    return NeighborBasedCollisionDetector::_get_neighbor_coords(key_list, window);
}

/**
 * @brief Create a dest coord object
 *
 * @param nodes 離散座標に対する八分木ノードが入った辞書
 * @param window 近傍の大きさ
 * @return std::set<std::tuple<int, int, int>> coordsのそれぞれの近傍の集合
 */
std::set<std::tuple<int, int, int>>
NeighborBasedCollisionDetector::create_dest_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes,
                                                  int window)
{
    std::set<std::tuple<int, int, int>> key_list;
    for (const auto& [key, value] : nodes)
    {
        key_list.insert(key);
    }

    return key_list;
}

/**
 * @brief 離散座標の近傍ベースで衝突可能性を判定する
 *
 * @param src_coords srcの集合
 * @param dest_coords destの集合
 * @return true srcとdestで衝突可能性がある
 * @return false 衝突可能性がない
 */
bool NeighborBasedCollisionDetector::collision_detection(const std::set<std::tuple<int, int, int>>& src_coords,
                                                         const std::set<std::tuple<int, int, int>>& dest_coords)
{
    for (auto dest_it = dest_coords.begin(); dest_it != dest_coords.end(); dest_it++)
    {
        if (src_coords.find(*dest_it) != src_coords.end())
        {
            return true;
        }
    }

    return false;
}
