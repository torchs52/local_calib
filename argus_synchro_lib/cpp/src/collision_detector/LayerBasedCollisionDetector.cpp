#include "octotree/LayerBasedCollisionDetector.h"

LayerBasedCollisionDetector::LayerBasedCollisionDetector(CoordMethod coord_method)
    : AbstractCollisionDetector(coord_method)
{
}

LayerBasedCollisionDetector::~LayerBasedCollisionDetector()
{
}
/**
 * @brief OctoNodeのリストのwindow分上位層のモートン順序の集合を取得する
 *
 * @param nodes OctoNodeインスタンスをiteratorに持つ変数
 * @param window モートン順序の何層上位までを取り出すかを指定するための変数
 * @return std::set<int> モートン順序の上位window層の集合
 */
std::set<int> LayerBasedCollisionDetector::_morton_codes_shift(const std::vector<OctoNode>& nodes, int window)
{
    // window分だけ上位のモートン順序を求める
    std::set<int> morton_code_vector;
    for (const auto& node : nodes)
    {
        morton_code_vector.insert(node.morton_code >> (window * 3));
    }
    return morton_code_vector;
}

/**
 * @brief Create a dilation coord object
 *
 * @param nodes 離散座標に対する八分木ノードが入った辞書
 * @param window モートン順序の何層上位までを取り出すかを指定するための変数
 * @return std::set<int> モートン順序の上位window層の集合
 */
std::set<int>
LayerBasedCollisionDetector::create_dialation_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes,
                                                    int window)
{
    // 八分木ノード側だけ取り出す
    std::vector<OctoNode> value_list;
    for (const auto& [key, value] : nodes)
    {
        value_list.push_back(value);
    }

    return LayerBasedCollisionDetector::_morton_codes_shift(value_list, window);
}

/**
 * @brief dest用の集合計算
 *
 * @param nodes 離散座標に対する八分木ノードが入った辞書
 * @param window モートン順序の何層上位までを取り出すかを指定するための変数
 * @return std::set<int> モートン順序の上位window層の集合
 */
std::set<int> LayerBasedCollisionDetector::create_dest_coord(const std::map<std::tuple<int, int, int>, OctoNode>& nodes,
                                                             int window)
{
    return this->create_dialation_coord(nodes, window);
}

/**
 * @brief srcとdestの集合に対して、衝突可能性を判定する
 *
 * @param src_coords srcの集合
 * @param dest_coords destの集合
 * @return true srcとdestで衝突可能性がある
 * @return false 衝突可能性がない
 */
bool LayerBasedCollisionDetector::collision_detection(const std::set<int>& src_coords, const std::set<int>& dest_coords)
{
    // 同じモートン順序を持つものがあるか調べる
    for (auto dest_it = dest_coords.begin(); dest_it != dest_coords.end(); dest_it++)
    {
        if (src_coords.find(*dest_it) != src_coords.end())
        {
            return true;
        }
    }

    return false;
}
