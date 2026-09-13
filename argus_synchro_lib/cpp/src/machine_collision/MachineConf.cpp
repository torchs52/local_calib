#include "octotree/MachineConf.h"

#include <algorithm>
#include <vector>

/**
 * @brief 機体の点群除去ファイルや衝突判定の機体点群のファイルのパスが入ったようなクラス
 *
 * @param pcd_points_file 衝突判定で用いられる機体点群のパス
 * @param load_order 読み込む順番
 * @param instance_name 機体点群除去のインスタンス名
 * @param offsets 初期のxyzのオフセット
 * @param reverse xyz軸の反転有無
 * @param is_mobile 可動部かどうか, trueの場合可動部
 * @param form_points_pattern 機体点群除去で用いるファイルを識別する文字列
 */
MachineConf::MachineConf(const std::string& pcd_points_file, int load_order, const std::string& instance_name,
                         const std::tuple<double, double, double>& offsets, const std::tuple<bool, bool, bool>& reverse,
                         bool is_mobile, const std::string& form_points_pattern)
    : pcd_points_file(pcd_points_file), load_order(load_order), instance_name(instance_name), offsets(offsets),
      reverse(reverse), is_mobile(is_mobile), form_points_pattern(form_points_pattern)
{
}

/**
 * @brief 機体除去に用いるファイルを取得する
 * @details filename="./CAB.csv", extension=".csv",
 * form_points_pattern="cuboid_points"の場合、./CAB_cuboid_points.csvが得られる
 *
 * @param filename 衝突判定に用いる機体点群のファイルパス
 * @param extension 機体除去に用いるファイルの拡張子, json形式でファイルを保持するものもあるので、引数になっている
 * @return std::string 機体除去に用いるファイルのパス, ./CAB_cuboid_points.csvなど
 */
std::string MachineConf::get_form_points_filename(const std::string& filename, const std::string& extension)
{
    size_t index = filename.find_last_of(".");
    std::string name = filename.substr(0, index);
    return name + "_" + this->form_points_pattern + extension;
};

std::vector<MachineConf> machine_info_lightning = {

    MachineConf("interpolated_SCX900_01_upper_part_lightning.csv", 1, "MachineCollisionImmobileCuboid"),
    MachineConf("interpolated_SCX900_02_CW_lightning.csv", 2, "MachineCollisionImmobileRoundCuboid", {0, 0, 0},
                {false, true, false}, "curound_points"),
    MachineConf("interpolated_SCX900_03_senkai_chushin_lightning.csv", 3, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_04_front_right_L_ji_part_lightning.csv", 4, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_05_front_left_L_ji_part_lightning.csv", 5, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_06_back_right_L_ji_part_lightning.csv", 6, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_07_back_left_L_ji_part_lightning.csv", 7, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_08_crawler_right_lightning.csv", 8, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_09_crawler_left_lightning.csv", 9, "MachineCollisionMobileCuboid"),
};

std::vector<MachineConf> machine_info_weighted = {

    MachineConf("interpolated_SCX900_01_upper_part.csv", 1, "MachineCollisionImmobileCuboid", {0, 0, 0}),
    MachineConf("interpolated_SCX900_02_CW.csv", 2, "MachineCollisionImmobileRoundCuboid", {0, 0, 0},
                {false, true, false}, "curound_points"),
    MachineConf("interpolated_SCX900_03_senkai_chushin.csv", 3, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_04_front_right_L_ji_part.csv", 4, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_05_front_left_L_ji_part.csv", 5, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_06_back_right_L_ji_part.csv", 6, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_07_back_left_L_ji_part.csv", 7, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_08_crawler_right.csv", 8, "MachineCollisionMobileCuboid"),
    MachineConf("interpolated_SCX900_09_crawler_left.csv", 9, "MachineCollisionMobileCuboid"),
};

std::vector<MachineConf> machine_info = machine_info_weighted;
