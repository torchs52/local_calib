
#include "octotree/OctoNode.h"
#include "octotree/alias.h"
//#include "rust-lib.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

OctoNode::OctoNode(VoxelCoord vox_coord, NodeEntity entity, std::shared_ptr<VoxStats> node_stats)
    : morton_code(0), entity(entity), cluster_label(std::nullopt), node_stats(node_stats)
{
    morton_code = morton_encode_3d(std::get<0>(vox_coord), std::get<1>(vox_coord), std::get<2>(vox_coord));
}

OctoNode::OctoNode(const OctoNode& other)
    : morton_code(other.morton_code), entity(other.entity), cluster_label(other.cluster_label),
      node_stats(other.node_stats)
{
}

/**
 * morton_codeとentityが同じかどうかでequalityを判定している
 */
OctoNode& OctoNode::operator=(const OctoNode& other)
{
    if (this == &other)
    {
        return *this;
    }
    morton_code = other.morton_code;
    entity = other.entity;
    cluster_label = other.cluster_label;
    node_stats = other.node_stats;
    return *this;
}

bool OctoNode::operator==(const OctoNode& obj) const
{
    // morton_codeとentityが一致する場合はTrueを返す
    return !(obj.morton_code ^ this->morton_code) && (obj.entity == this->entity);
}

bool OctoNode::operator!=(const OctoNode& obj) const
{
    return !(*this == obj);
}

uint64_t encode(unsigned int n)
{
    uint64_t code = n & 0x1fffff;
    code = (code | code << 32) & 0x1f00000000ffff;
    code = (code | code << 16) & 0x1f0000ff0000ff;
    code = (code | code << 8) & 0x100f00f00f00f00f;
    code = (code | code << 4) & 0x10c30c30c30c30c3;
    code = (code | code << 2) & 0x1249249249249249;
    return code;
}

/* x,y,z座標からモートン順序を計算する
 * 例: morton_encode_3d(x = 5 = 0b101, y = 3 = 0b011, z = 4 = 0b100) -> 001
 * | 000 | 001 + 000 | 010 | 010 + 100 | 000 | 000 -> 0b101010011 = 339
 */
uint64_t OctoNode::morton_encode_3d(unsigned int x, unsigned int y, unsigned int z)
{
    uint64_t morton_code = 0;
    morton_code |= encode(x) | encode(y) << 1 | encode(z) << 2;
    return morton_code;
}

unsigned int decode(uint64_t code)
{
    uint64_t n = code & 0x1249249249249249;
    n = (n | n >> 2) & 0x10c30c30c30c30c3;
    n = (n | n >> 4) & 0x100f00f00f00f00f;
    n = (n | n >> 8) & 0x1f0000ff0000ff;
    n = (n | n >> 16) & 0x1f00000000ffff;
    n = (n | n >> 32);
    return n;
}

/* モートン順序をx,y,z座標に変換する
 * 例: morton_decode_3d(morton_code = 339 = 0b101010011) -> 101 | 010 |
 * 011->x = 0b101 = 5, y = 0b011 = 3, z = 0b101 = 4
 */
VoxelCoord OctoNode::morton_decode_3d(uint64_t morton_code)
{
    auto x_coord = decode(morton_code);
    auto y_coord = decode(morton_code >> 1);
    auto z_coord = decode(morton_code >> 2);
    return {x_coord, y_coord, z_coord};
}

std::string OctoNode::to_string()
{
    return std::to_string(this->morton_code);
}

void OctoNode::set_cluster_label(int cluster_label)
{
    this->cluster_label = cluster_label;
}

std::optional<int> OctoNode::get_cluster_label() const
{
    return this->cluster_label;
}

void OctoNode::free_vox_stats()
{
    this->node_stats.reset();
}

/** ノード内の点群の平均値を返す
平均値計算が行われていない場合はnullを返す
 */
std::optional<std::tuple<float, float, float>> OctoNode::get_mean()
{
    if (!this->node_stats)
    {
        return std::nullopt;
    }
    Point target_point = this->node_stats->get_mean();
    if (target_point.is_null)
    {
        // 構造体Pointはis_nullの場合nullを表しているので、nullを返す
        return std::nullopt;
    }
    return std::make_tuple(this->node_stats->first_moment.x, this->node_stats->first_moment.y,
                           this->node_stats->first_moment.z);
}

/** ノード内の点群のfar_pointを返す
平均値計算が行われていない場合はnullを返す
 */
std::optional<std::tuple<float, float, float>> OctoNode::get_far_point()
{
    if (!this->node_stats)
    {
        return std::nullopt;
    }
    Point target_point = this->node_stats->far_point;
    if (target_point.is_null)
    {
        // 構造体Pointはis_nullの場合nullを表しているので、nullを返す
        return std::nullopt;
    }
    return std::make_tuple(target_point.x, target_point.y, target_point.z);
}

/** ノード内の点群のnear_pointを返す
平均値計算が行われていない場合はnullを返す
 */
std::optional<std::tuple<float, float, float>> OctoNode::get_near_point()
{
    if (!this->node_stats)
    {
        return std::nullopt;
    }
    Point target_point = this->node_stats->near_point;
    if (target_point.is_null)
    {
        // 構造体Pointはis_nullの場合nullを表しているので、nullを返す
        return std::nullopt;
    }
    return std::make_tuple(target_point.x, target_point.y, target_point.z);
}

/** ノード内の点群のquantileを返す
平均値計算が行われていない場合はnullを返す
 */
std::optional<std::tuple<float, float, float>> OctoNode::get_quantile()
{
    if (!this->node_stats)
    {
        return std::nullopt;
    }
    Point target_point = this->node_stats->quantile;
    if (target_point.is_null)
    {
        // 構造体Pointはis_nullの場合nullを表しているので、nullを返す
        return std::nullopt;
    }

    return std::make_tuple(target_point.x, target_point.y, target_point.z);
}
