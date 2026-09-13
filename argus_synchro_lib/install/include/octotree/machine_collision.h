#pragma once

#include "MachineCollisionBase.h"
#include "MachineConf.h"
#include "MachineCollisionImmobileHexaPrism.h"

#include <memory>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * 複数のディレクトリをつなげて返す
 * ディレクトリの区切りは、スラッシュで統合
 */
std::string connect_path(const std::vector<std::string>& str);

/** filenameの拡張子をチェックする
 * filenameの拡張子がextensionの場合, trueを返す
 */
bool check_extension(const std::string& filename, const std::string& extension);

/**
 * file_pathの読み込み, カンマ区切りされたファイルを行列に変換する
 */
Eigen::MatrixXd read_saved_points(const std::string& file_path);

/**
 * @brief json内の情報から必要なインスタンスを生成する
 * @details MachineCollisionImmobileHexaPrismを構成するCuboidRangeやTriPillarインスタンスを作っている
 *
 * @param entry
 * @return std::shared_ptr<machine_rm::BaseRange>
 */
std::shared_ptr<machine_rm::BaseRange> create_base_range_from_json(const json& entry);

/**
 * @brief json_pathからBaseRangeオブジェクトのリストを作ってそれを返す関数
 * @details MachineCollisionImmobileHexaPrismを構成するBaseRangeのリストを作っている
 * @details MachineCollisionImmobileHexaPrism::hex_base_rangesというインスタンス変数を作るのに必要な関数
 *
 * @param json_path
 * @return std::vector<std::shared_ptr<machine_rm::BaseRange>>
 */
std::vector<std::shared_ptr<machine_rm::BaseRange>> load_base_ranges_from_json(const std::string json_path);

/**
 * jsoncの中のコメントを削除する関数
 */
std::string remove_json_comments(const std::string& raw_file);

/**
 * @brief jsoncファイルを読み込んでパースする
 *
 * @param filepath jsoncファイル, コメントは削除して結果を返す
 * @return json nlohmannのjsonインスタンス
 */
json load_jsonc(const std::string& filepath);

/**
 * file_dir以下にある衝突判定に用いる機体の各部位情報をリストにする
 * @param file_dir  衝突判定に用いる機体情報が入っているディレクトリ
 * @retval 戻り値 l_machine_collision : 衝突判定で用いる機体の情報が入ったリスト
 */
std::vector<std::unique_ptr<MachineCollisionBase>>
create_machine_collision_list(const std::string& file_dir, const std::tuple<double, double, double>& initial_offsets,
                              const std::vector<MachineConf>& l_col_machine_conf,
                              const std::tuple<bool, bool, bool>& reverse = std::make_tuple(true, true, false));
