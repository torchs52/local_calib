#pragma once

#include <string>
#include <tuple>
#include <vector>

/*機体除去, 衝突判定で用いる機体情報に関するクラス
    可視化用で使うMachineConfと同じ名前なので、同時に使う場合は、import先で別名を割り当てる必要がある*/
class MachineConf
{
  public:
    // 読み込むファイルを特定するためのファイル名,
    // 同じディレクトリに同名が存在すると意図したファイルを呼び出せなくなるので、同名が存在しないように設定する
    std::string pcd_points_file;

    // 読み込む順番,
    // 読み込む順が早いものから順に任意の整数を設定すれば、その順に並び替えられる
    int load_order;

    // 呼び出すクラス名,
    // クローラーと上部旋回体、カウンタウェイトなどで使うクラスが異なるので、どのクラスを呼び出すかを指定するための文字列
    std::string instance_name;

    // 初期のオフセット
    std::tuple<double, double, double> offsets;

    // xyz軸の反転有無
    std::tuple<bool, bool, bool> reverse;

    // 機体除去で用いる形状点群ファイル名のパターン,
    // ${ pcd_points_fileから拡張子を除いた文字列
    // } + ${ form_points_patter }.csv をloadする
    std::string form_points_pattern;

    /**
     * @brief 旋回可能な部位かどうかを判定するフラグ, trueの場合、旋回可能
     *
     */
    bool is_mobile;

    /**
     * @brief 機体の点群除去ファイルや衝突判定の機体点群のファイルのパスが入ったようなクラス
     *
     * @param pcd_points_file 衝突判定で用いられる機体点群のパス
     * @param load_order 読み込む順番
     * @param instance_name 機体点群除去のインスタンス名
     * @param offsets 初期のxyzのオフセット
     * @param reverse xyz軸の反転有無
     * @param form_points_pattern 機体点群除去で用いるファイルを識別する文字列
     */
    MachineConf(const std::string& pcd_points_file, int load_order, const std::string& instance_name,
                const std::tuple<double, double, double>& offsets = std::make_tuple(0, 0, 0),
                const std::tuple<bool, bool, bool>& reverse = std::make_tuple(false, true, false),
                bool is_mobile = false, const std::string& form_points_pattern = "cuboid_points");

    /**
     * @brief 機体除去に用いるファイルを取得する
     * @details filename="./CAB.csv", extension=".csv",
     * form_points_pattern="cuboid_points"の場合、./CAB_cuboid_points.csvが得られる
     *
     * @param filename 衝突判定に用いる機体点群のファイルパス
     * @param extension 機体除去に用いるファイルの拡張子, json形式でファイルを保持するものもあるので、引数になっている
     * @return std::string 機体除去に用いるファイルのパス, ./CAB_cuboid_points.csvなど
     */
    std::string get_form_points_filename(const std::string& filename, const std::string& extension = ".csv");
};

extern std::vector<MachineConf> machine_info;
