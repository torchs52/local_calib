#pragma once

#include <optional>
#include <tuple>

// 八分木ノードの属性に対応する列挙型
// OctoNodeも属性を保持できるが、現状はOctoTree.entity_octonodesのkeyの一部で用いる運用になっている
enum class NodeEntity
{
    UNK = 1,                // 属性付与前の状態, クラスタリング失敗で付与される
    HUMAN,                  // 人
    CRANE,                  // 機体
    CRANE_MOBILE,           // 下部走行体の衝突判定に用いる機体点群
    CRANE_IMMOBILE,         // 上部旋回体の衝突判定に用いる機体点群
    CRANE_MOBILE_FOR_DET,   // 下部走行体の接触可能性探索に用いる機体点群
    CRANE_IMMOBILE_FOR_DET, // 上部旋回体の接触可能性探索に用いる機体点群
    CRANE_EXTERNAL_GUARD,   // 安全装置が付いている際の機体点群
    CLIFF,                  // 崖
    HIGH_3D,                // 高い立体物を入れる想定だが、現状使っていない
    LOW_3D,                 // 地面点群
    OTHER, // 蓄積点群, クラスタリング前やクラスタリング後のクラスタされた八分木ノード(人属性が付与される前)が入っている
};

/*
 NodeEntity + クラスタ番号を表現する構造体で、mapのkeyに用いる
 クラスがタ番号nullの場合、クラスタが未割当であることを表す
 entity: ノードの属性を表す
 cluster_id: クラスタ番号を表す, nulloptの場合未割当
*/
struct NodeClusterKey
{
    NodeEntity entity;             // 属性
    std::optional<int> cluster_id; // クラスタ番号

    // map用の順序演算子
    bool operator<(const NodeClusterKey& other) const
    {
        return std::tie(entity, cluster_id) < std::tie(other.entity, other.cluster_id);
    }

    /**
     * @brief keyの等号は, entityとcluster_idが同じ場合trueになることにする
     *
     * @param other
     * @return true
     * @return false
     */
    bool operator==(const NodeClusterKey& other) const
    {
        return entity == other.entity && cluster_id == other.cluster_id;
    }
};
