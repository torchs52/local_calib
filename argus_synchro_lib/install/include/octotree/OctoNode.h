#pragma once
#include "NodeEntity.h"
#include "VoxStats.h"
#include "alias.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

/** 八分木ノードを表現するクラス
 点群を離散化した座標とその点のNodeEntityをコンストラクタとして入力するとインスタンスが生成される
 離散座標内の各点群の位置の統計量もVoxStatsという型で保持できるようにしている
 */
class OctoNode
{
  public:
    int morton_code;   // 該当ノードのモートン順序
    NodeEntity entity; // ノードの属性

    // ノードのクラスタリングラベル
    // Todo: 結局使わない構造に変えたので時間があれば消す,
    // 上位クラスで使っていないので、適宜setterやbind側から消せばエラーなしで消えるはず
    std::optional<int> cluster_label;
    std::shared_ptr<VoxStats> node_stats;

    OctoNode(VoxelCoord vox_coord, NodeEntity entity = NodeEntity::UNK, std::shared_ptr<VoxStats> node_stats = nullptr);
    OctoNode(const OctoNode& other);
    OctoNode& operator=(const OctoNode& other);
    OctoNode(OctoNode&& other) noexcept = default;
    OctoNode& operator=(OctoNode&& other) noexcept = default;

    /**
     * objが点群を含んでいて、該当ノードがobjと同じノードに存在する同じ物体かどうか判定する
     * morton_codeの一致を判定すればよい
     */
    bool operator==(const OctoNode& obj) const;

    bool operator!=(const OctoNode& obj) const;

    /* x,y,z座標からモートン順序を計算する
     * 例: morton_encode_3d(x = 5 = 0b101, y = 3 = 0b011, z = 4 = 0b100) -> 001
     * | 000 | 001 + 000 | 010 | 010 + 100 | 000 | 000 -> 0b101010011 = 339
     */
    static uint64_t morton_encode_3d(unsigned int x, unsigned int y, unsigned int z);

    /* モートン順序をx,y,z座標に変換する
     * 例: morton_decode_3d(morton_code = 339 = 0b101010011) -> 101 | 010 |
     * 011->x = 0b101 = 5, y = 0b011 = 3, z = 0b101 = 4
     */
    static VoxelCoord morton_decode_3d(uint64_t morton_code);

    std::string to_string();

    void set_cluster_label(int cluster_label);
    std::optional<int> get_cluster_label() const;

    /*
    VoxStats関連
    */
    /* vox_statsのメモリを開放する,
     * Rustからc++への変換プログラムを作成中に発生したが、不要かも */
    void free_vox_stats();

    /* VoxStatsの平均値を取得する,
     * Rustからc++への良い感じのライブラリが見つからず、無理くりtupleで返す */
    std::optional<std::tuple<float, float, float>> get_mean();

    /* VoxStatsのfar_pointを取得する,
     * Rustからc++への良い感じのライブラリが見つからず、無理くりtupleで返す */
    std::optional<std::tuple<float, float, float>> get_far_point();

    /* VoxStatsのnear_pointを取得する,
     * Rustからc++への良い感じのライブラリが見つからず、無理くりtupleで返す */
    std::optional<std::tuple<float, float, float>> get_near_point();

    /* VoxStatsのnear_pointを取得する,
     * Rustからc++への良い感じのライブラリが見つからず、無理くりtupleで返す */
    std::optional<std::tuple<float, float, float>> get_quantile();
};
