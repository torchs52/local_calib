#include "octotree/machine_collision.h"
#include "octotree/MachineCollisionImmobileCuboid.h"
#include "octotree/MachineCollisionImmobileRoundCuboid.h"
#include "octotree/MachineCollisionMobileCuboid.h"
#include "octotree/MachineCollisionImmobileHexaPrism.h"
#include "octotree/MachineConf.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <regex>

/**
 * 複数のディレクトリをつなげて返す
 * ディレクトリの区切りは、スラッシュで統合
 */
std::string connect_path(const std::vector<std::string>& str)
{
    std::filesystem::path path(str.at(0));
    for (size_t i = 1; i < str.size(); i++)
    {
        path.append(str.at(i));
    }

    return path.generic_string();
}

/** filenameの拡張子をチェックする
 * filenameの拡張子がextensionの場合, trueを返す
 */
bool check_extension(const std::string& filename, const std::string& extension)
{
    if (filename.size() >= extension.size() &&
        filename.compare(filename.size() - extension.size(), extension.size(), extension) == 0)
    {
        return true;
    }
    return false;
}

/**
 * file_pathの読み込み, カンマ区切りされたファイルを行列に変換する
 */
Eigen::MatrixXd read_saved_points(const std::string& file_path)
{
    std::ifstream file(file_path);
    std::string line;
    std::vector<std::vector<double>> data;

    while (std::getline(file, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> row;

        while (std::getline(lineStream, cell, ','))
        {
            std::stringstream ssField(cell);
            std::string subField;
            while (std::getline(ssField, subField, ' '))
            {
                row.push_back(std::stod(subField));
            }
        }
        data.push_back(row);
    }
    file.close();

    Eigen::MatrixXd matrix(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            matrix(i, j) = data[i][j];
        }
    }

    return matrix;
}

/**
 * @brief json内の情報から必要なインスタンスを生成する
 * @details MachineCollisionImmobileHexaPrismを構成するCuboidRangeやTriPillarインスタンスを作っている
 *
 * @param entry
 * @return std::shared_ptr<machine_rm::BaseRange>
 */
std::shared_ptr<machine_rm::BaseRange> create_base_range_from_json(const json& entry)
{
    const std::string class_name = entry.at("class_name");
    const auto& args = entry.at("args");

    if (class_name == "CuboidRange")
    {
        auto base_range_obj = std::make_shared<machine_rm::CuboidRange>(
            std::make_tuple(args.at("x_minmax")[0].get<double>(), args.at("x_minmax")[1].get<double>()),
            std::make_tuple(args.at("y_minmax")[0].get<double>(), args.at("y_minmax")[1].get<double>()),
            std::make_tuple(args.at("z_minmax")[0].get<double>(), args.at("z_minmax")[1].get<double>()),
            std::make_tuple(args.at("x_range_ratio")[0].get<double>(), args.at("x_range_ratio")[1].get<double>()),
            std::make_tuple(args.at("y_range_ratio")[0].get<double>(), args.at("y_range_ratio")[1].get<double>()),
            std::make_tuple(args.at("z_range_ratio")[0].get<double>(), args.at("z_range_ratio")[1].get<double>()));
        return base_range_obj;
    }
    else if (class_name == "TriPillar")
    {
        std::vector<std::tuple<double, double>> tri_points;
        for (auto& pt : args.at("tri_points"))
        {
            tri_points.emplace_back(pt[0].get<double>(), pt[1].get<double>());
        }

        auto base_range_obj = std::make_shared<machine_rm::TriPillar>(
            std::make_tuple(args.at("z_minmax")[0].get<double>(), args.at("z_minmax")[1].get<double>()), tri_points,
            std::make_tuple(args.at("x_remove_offset_ratio")[0].get<double>(),
                            args.at("x_remove_offset_ratio")[1].get<double>()),
            std::make_tuple(args.at("y_remove_offset_ratio")[0].get<double>(),
                            args.at("y_remove_offset_ratio")[1].get<double>()),
            std::make_tuple(args.at("z_remove_offset_ratio")[0].get<double>(),
                            args.at("z_remove_offset_ratio")[1].get<double>()),
            args.value("vec_is_reverse", false));
        return base_range_obj;
    }
    else
    {
        throw std::runtime_error("Unknown class_name: " + class_name);
    }
}

/**
 * json_pathからBaseRangeオブジェクトのリストを作ってそれを返す関数
 */
std::vector<std::shared_ptr<machine_rm::BaseRange>> load_base_ranges_from_json(const std::string json_path)
{
    json json_obj = load_jsonc(json_path);
    std::vector<std::shared_ptr<machine_rm::BaseRange>> base_ranges;

    for (auto& entry : json_obj)
    {
        base_ranges.push_back(create_base_range_from_json(entry));
    }

    return base_ranges;
}

/**
 * jsoncの中のコメントを削除する関数
 */
std::string remove_json_comments(const std::string& raw_file)
{
    std::string output = raw_file;

    output = std::regex_replace(output, std::regex(R"(//[^\n]*)"), "");
    output = std::regex_replace(output, std::regex(R"(/\*[\s\S]*?\*/)"), "");

    return output;
}

/**
 * @brief jsoncファイルを読み込んでパースする
 *
 * @param filepath jsoncファイル, コメントは削除して結果を返す
 * @return json nlohmannのjsonインスタンス
 */
json load_jsonc(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open JSONC file: " + filepath);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    std::string cleaned = remove_json_comments(content);

    return json::parse(cleaned);
}

/**
 * file_dir以下にある衝突判定に用いる機体の各部位情報をリストにする
 * @param file_dir  衝突判定に用いる機体情報が入っているディレクトリ
 * @retval 戻り値 l_machine_collision : 衝突判定で用いる機体の情報が入ったリスト
 */
std::vector<std::unique_ptr<MachineCollisionBase>>
create_machine_collision_list(const std::string& file_dir, const std::tuple<double, double, double>& initial_offsets,
                              const std::vector<MachineConf>& l_col_machine_conf,
                              const std::tuple<bool, bool, bool>& reverse)
{
    // auto _l_col_machine_conf = (l_col_machine_conf == std::nullopt)
    //                                ? machine_info
    //                                : l_col_machine_conf.value();

    std::vector<std::unique_ptr<MachineCollisionBase>> l_machine_collision;

    for (const auto& machine_parts_conf : l_col_machine_conf)
    {
        // MachineCollisionBaseを継承している機体点群除去に用いるインスタンスを
        // instance_nameに応じて生成する
        const auto& name = machine_parts_conf.instance_name;
        if (name == "MachineCollisionImmobileCuboid")
        {
            l_machine_collision.push_back(std::unique_ptr<MachineCollisionBase>(new MachineCollisionImmobileCuboid(
                machine_parts_conf, connect_path({file_dir, machine_parts_conf.pcd_points_file}), initial_offsets,
                reverse)));
        }
        else if (name == "MachineCollisionImmobileRoundCuboid")
        {
            l_machine_collision.push_back(std::unique_ptr<MachineCollisionBase>(new MachineCollisionImmobileRoundCuboid(
                machine_parts_conf, connect_path({file_dir, machine_parts_conf.pcd_points_file}), initial_offsets,
                reverse)));
        }
        else if (name == "MachineCollisionImmobileHexaPrism")
        {
            l_machine_collision.push_back(
                std::unique_ptr<MachineCollisionBase>(new machine_rm::MachineCollisionImmobileHexaPrism(
                    machine_parts_conf, connect_path({file_dir, machine_parts_conf.pcd_points_file}), initial_offsets,
                    reverse)));
        }
        else if (name == "MachineCollisionMobileCuboid")
        {
            l_machine_collision.push_back(std::unique_ptr<MachineCollisionBase>(new MachineCollisionMobileCuboid(
                machine_parts_conf, connect_path({file_dir, machine_parts_conf.pcd_points_file}), initial_offsets,
                reverse)));
        }
        else
        {
            // instance_nameが想定外の場合はエラー落ちすることにする
            std::runtime_error("instance_nameが想定されないクラスを指定しています。");
        }
    }

    return l_machine_collision;
}
