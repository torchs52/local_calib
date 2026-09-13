#include "ui_interface/ui_interface.h"
#include "clsmmap/ClsMMap.h"
#include "octotree/OctoTree.h"
#include "ui_interface/GeneralConf.h"
#include "ui_interface/IFCliffInfo.h"
#include "ui_interface/UIIFConf.h"
#include "ui_interface/helper.h"
#include "ui_interface/status_mmap.h"
#include "ui_interface/vis_octotree.h"
#include "cpp_helper_lib/eigen_operator.h"
#include "logger/py_logger.h"

#include <Eigen/Dense>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

// NOTE: 他のモジュールからも使う必要あれば、移動を検討する(NSW)
template <typename T> static inline std::string VectorToString(const std::vector<T>& vec, size_t max_show = 3)
{
    std::string msg;
    msg.append("[ ");
    if (vec.size() <= max_show * 2)
    {
        for (size_t i = 0; i < vec.size(); ++i)
        {
            msg.append(std::to_string(vec[i]));
            msg.append(".");
            if (i + 1 < vec.size())
                msg.append(" ");
        }
    }
    else
    {
        for (size_t i = 0; i < max_show; ++i)
        {
            msg.append(std::to_string(vec[i]));
            msg.append(". ");
        }
        msg.append("... ");

        for (size_t i = vec.size() - max_show; i < vec.size(); ++i)
        {
            msg.append(std::to_string(vec[i]));
            msg.append(".");
            if (i + 1 < vec.size())
                msg.append(" ");
        }
    }
    msg.append("]");
    return msg;
}

// 実行環境を取得
static inline std::string getPlatform(void)
{
#if defined(_WIN32)
    std::string system = "Windows";
#elif defined(__linux__)
    std::string system = "Linux";
#elif defined(__APPLE__)
    std::string system = "Darwin";
#else
    std::string system = "Unknown OS";
#endif
    return system;
}

UI_interface::UI_interface(const UIIFConf& ui_if, int s_frame, double rotation_radius, int camera_num,
                           bool has_external_guard, double external_guard_offset, const std::string& status_mmap_path,
                           const LoggerFunc logfunc)
    : damp_out(ui_if.damp_out), bbox_3d_num(ui_if.bbox_3d_num), bbox_3d_distance(ui_if.bbox_3d_distance),
      dampPathList(ui_if.damp_mmap), show_unk(ui_if.show_unk), collision_depict_dist(ui_if.collision_depict_dist),
      collision_attention_dist(ui_if.collision_attention_dist), collision_warning_dist(ui_if.collision_warning_dist),
      cliff_attention_dist(ui_if.cliff_attention_dist), cliff_warning_dist(ui_if.cliff_warning_dist),
      draw_bbox_3d(ui_if.draw_bbox_3d), draw_collision(ui_if.draw_collision), s_frame(s_frame),
      rotation_radius(rotation_radius), camera_num(camera_num), damp_fp_list(std::vector<std::ofstream>()),
      clsMMap(createMMapConfig(ui_if.UI_mmap), logfunc), writtenAdr(IsWriting_ADR), collision_level(COLLISION_LEVEL()),
      has_external_guard(has_external_guard), external_guard_offset(external_guard_offset), status(status_mmap_path),
      logger_(PyLogger(logfunc))
{
    this->logger_.info("platform: %s", getPlatform().c_str());
    this->logger_.info("endian: %s", (this->isLittleEndian() ? "little" : "big"));

    auto now = std::chrono::system_clock::now().time_since_epoch();
    sTime = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    preProcessTime = sTime;

    if (damp_out)
    {
        for (const auto& damp_path : dampPathList)
        {
            // damp_file
            if (std::filesystem::exists(damp_path))
            {
                std::filesystem::remove(damp_path);
            }
        }

        // dampファイルポインタ生成
        for (const auto& damp_path : dampPathList)
        {
            std::ofstream damp_fp(damp_path, std::ios::binary | std::ios::out);
            if (!damp_fp)
            {
                throw std::runtime_error("ファイルが開けませんでした");
            }
            damp_fp_list.push_back(std::move(damp_fp));
        }
    }

    // ステータス更新のみ行いたい(create=False)
    this->logger_.info("RUNNING_CODE 設定");
    this->status.write_status(StatusCode::RUNNING);
}

UI_interface::~UI_interface()
{
    this->close_mmap();
}

void UI_interface::close_mmap()
{
    this->clsMMap.dispose();
}

void UI_interface::set_collision_clusters_info(Ccol_res& collision_clusters)
{
    // 衝突危険度は毎回初期化
    this->collision_level = COLLISION_LEVEL::NORMAL;

    // collision_clusters が nullptr または空の場合はそのまま設定して終了
    if (collision_clusters.empty())
    {
        this->collision_clusters = collision_clusters;
        return;
    }

    // 最小値の基準値:無限遠を設定しても良い.
    double min_distance = 10000.0f;
    std::vector<std::optional<int>> del_key_list;

    // 各要素について処理
    // タプルの要素
    // 0: OctoNode, 1: OctoNode, 2: tuple<float,float,float>, 3:
    // tuple<float,float,float>, 4: distance, 5: radius
    for (const auto& [idx, tup] : collision_clusters)
    {
        double distance = std::get<4>(tup);
        std::optional<double> radius = std::get<5>(tup);
        if (!radius.has_value())
        {
            continue;
        }

        // 機体からの距離基準
        // if distance < min_distance:
        // min_distance = distance
        // 旋回中心からの距離基準(min_distance は、後端半径からの距離)
        if (this->has_external_guard)
        { //機体外部に安全器具（ガード）が付いている場合
            // 安全器具までの距離（内側はマイナス距離）
            double distace_for_guard = (radius.value() - this->rotation_radius - this->external_guard_offset);
            if (distace_for_guard > 0)
            {
                min_distance = std::min(min_distance, distace_for_guard);
            }
            // 衝突判定結果を出力しない.常に削除
            del_key_list.push_back(idx);
        }
        else
        {
            // 外部安全器具が付いていない場合（デフォルト）
            min_distance = std::min(min_distance, radius.value() - this->rotation_radius);
            // 距離が閾値より遠いものは削除.(衝突ペアを表示しない)
            if (distance > this->collision_depict_dist)
            {
                del_key_list.push_back(idx);
            }
        }
    }

    for (std::optional<int> del_key : del_key_list)
    {
        collision_clusters.erase(del_key);
    }

    // 衝突危険度の判定
    if (min_distance < this->collision_warning_dist)
    {
        this->collision_level = COLLISION_LEVEL::WARNING;
    }
    else if (min_distance < this->collision_attention_dist)
    {
        this->collision_level = COLLISION_LEVEL::ATTENTION;
    }
    // 最終的に collision_clusters をメンバ変数に設定する
    this->collision_clusters = collision_clusters;
}

bool UI_interface::isLittleEndian()
{
    int num = 1;
    return *(char*)&num == 1;
}

/**
 * LS_pcd_detから崖検出のインターフェース部分を作る
 * n_cliffs: 崖個数
 * cliff_vertices: 崖境界の頂点数
 * cliff_points: 各クラスタの崖境界頂点数
 * cliff_det_level: 崖検知警告
 */
void UI_interface::set_cliff_info_by_octreee(OctoTree& octotree_obj)
{
    std::vector<Eigen::MatrixXd> cliff_cluster = octotree_obj.get_np_from_entity_octonodes_by_list({NodeEntity::CLIFF});
    if (cliff_cluster.empty())
    {
        this->written_cliff_info = IFCliffInfo();
        return;
    }

    // 必要な形に変換する
    int n_cliffs = cliff_cluster.size();
    std::vector<int> cliff_vertices;
    cliff_vertices.reserve(n_cliffs);
    for (int i = 0; i < n_cliffs; i++)
    {
        cliff_vertices.push_back(cliff_cluster.at(i).rows());
    }

    int total_rows = 0;
    for (const auto& matrix : cliff_cluster)
    {
        total_rows += matrix.rows();
    }
    int cols = cliff_cluster.at(0).cols();
    Eigen::MatrixXd stacked(total_rows, cols);

    int current_row = 0;
    for (const auto& matrix : cliff_cluster)
    {
        stacked.block(current_row, 0, matrix.rows(), matrix.cols()) = matrix;
        current_row += matrix.rows();
    }

    Eigen::MatrixXd cliff_2d = stacked.leftCols(2);
    Eigen::VectorXd edge_dist_from_radius = cliff_2d.rowwise().norm().array() - this->rotation_radius;
    double min_edge_dist = edge_dist_from_radius.minCoeff();

    //  判定
    CLIFF_LEVEL cliff_det_level;
    if (min_edge_dist < this->cliff_warning_dist)
    {
        cliff_det_level = CLIFF_LEVEL::WARNING;
    }
    else if (min_edge_dist < this->cliff_attention_dist)
    {
        cliff_det_level = CLIFF_LEVEL::ATTENTION;
    }
    else
    {
        cliff_det_level = CLIFF_LEVEL::NORMAL;
    }
    // vector -> eigen
    Eigen::Map<Eigen::VectorXi> cliff_vertices_matrix(cliff_vertices.data(), cliff_vertices.size());
    this->written_cliff_info = IFCliffInfo(n_cliffs, cliff_vertices_matrix, cliff_2d, cliff_det_level);
}

void UI_interface::write_2d_object_detection_result(const std::tuple<int, int>& frame_shape,
                                                    const CameraDetectionData& bb_box_data)
{
    Eigen::MatrixXf out_boxes = bb_box_data.boxes;
    Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> out_classes = bb_box_data.classes;
    out_classes.resize(1, out_classes.size());

    int num_boxes = bb_box_data.valid_detects;

    int person_num = 0;
    const int person_num_Adr = this->writtenAdr; //書き込み場所を記憶して、最後に戻って書く.
    // 四角形BB個数
    this->logger_.debug("Adr(person_num) (Skip): %d", person_num_Adr);
    this->writtenAdr += static_cast<int>(ByteSize::INT8);

    Eigen::Vector<int16_t, 4> coordinate = Eigen::Vector<int16_t, 4>::Zero(4);
    for (size_t i = 0; i < num_boxes; i++)
    {
        int class_ind = static_cast<int>(out_classes(0, i));
        if ((class_ind < 0) || (class_ind > NUM_CLASSES))
        {
            continue;
        }
        if (class_ind == 0)
        {
            person_num++;

            coordinate(0) = static_cast<int16_t>(out_boxes(i, 1) * std::get<1>(frame_shape)); // x 始点
            coordinate(1) = static_cast<int16_t>(out_boxes(i, 0) * std::get<0>(frame_shape)); // y 始点
            coordinate(2) = static_cast<int16_t>(out_boxes(i, 3) * std::get<1>(frame_shape)); // x 終点
            coordinate(3) = static_cast<int16_t>(out_boxes(i, 2) * std::get<0>(frame_shape)); // y 終点
            this->logger_.debug("Adr(person_coordinate) (Skip): %d", this->writtenAdr);
            // 四角形BB頂点1x, 1y, 2x, 2y (対角線)
            // WARNING iとなっており、再定義していたのので修正。
            for (int coordinate_i = 0; coordinate_i < coordinate.size(); coordinate_i++)
            {
                this->clsMMap.WriteSignedInt16(this->writtenAdr, coordinate(coordinate_i));
                this->writtenAdr += static_cast<int>(ByteSize::INT16);
                this->logger_.debug("coordinate[%d]: %d", coordinate_i, coordinate(coordinate_i));
            }
        }
    }
    // 四角形BB個数
    this->logger_.info("Adr(person_num): %d", person_num_Adr);
    this->clsMMap.WriteInt8(person_num_Adr, person_num);
    this->logger_.info("person_num = %d", person_num);

    // tmpBytes = self.clsMMap.ReadBytes(person_num_Adr, int(1 + 2 * person_num +
    // 3))
    // AppLogger.info("ui_if", tmpBytes) C++移植前からコメント
}
void UI_interface::write_3d_object_detection_result_proj(const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                                                         const Eigen::Ref<const Eigen::VectorXi>& valid_detects,
                                                         const Camera& camera,
                                                         const std::map<int, NodeEntity>& cluster2entity)
{
    int n_clusters = valid_detects(0);

    Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> w_2d_coord = calc_bounding_box_around_entity(
        camera, boxes.block(0, 0, n_clusters * 8, boxes.cols()), cluster2entity, this->draw_bbox_3d, this->logger_);

    // 直方体BB個数(2D射影)
    this->logger_.debug("Adr(3d_obj_num): %d", this->writtenAdr);
    int three_d_obj_num = w_2d_coord.rows();
    this->logger_.debug("three_d_obj_num = %d", three_d_obj_num);
    this->clsMMap.WriteInt8(this->writtenAdr, three_d_obj_num);
    this->writtenAdr += static_cast<int>(ByteSize::INT8);

    // 直方体BB 8隅座標(2D射影)
    this->logger_.debug("Adr(3d_obj_proj_coord): %d", this->writtenAdr);
    Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> transposed_w_2d_coord = w_2d_coord.transpose();
    // NOTE ravel
    Eigen::Vector<int16_t, Eigen::Dynamic> w_2d_coord_16 = Eigen::Map<const Eigen::Vector<int16_t, Eigen::Dynamic>>(
        transposed_w_2d_coord.data(), transposed_w_2d_coord.size());
    this->logger_.debug("w_2d_coord_16 = %s", helper::EigenVectorToString(w_2d_coord_16).c_str());
    for (int i = 0; i < three_d_obj_num * 16; i++)
    {
        this->clsMMap.WriteSignedInt16(this->writtenAdr, w_2d_coord_16(i));
        this->writtenAdr += static_cast<int>(ByteSize::INT16);
    }
}

void UI_interface::write_collision_result_proj(
    const Eigen::Ref<const Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>>& w_2d_coord)
{
    // 衝突ペア個数（2D射影）
    this->logger_.debug("Adr(collision_num): %d", this->writtenAdr);
    int collision_proj_num = 0;
    if (w_2d_coord.size() != 0)
    {
        collision_proj_num = w_2d_coord.rows();
    }

    this->clsMMap.WriteInt8(this->writtenAdr, collision_proj_num);
    this->logger_.debug("collision_proj_num = %d", collision_proj_num);
    this->writtenAdr += static_cast<int>(ByteSize::INT8);

    //  衝突ペア座標(2D射影)
    this->logger_.debug("Adr(collision_proj_coord): %d", this->writtenAdr);
    // NOTE ravel
    Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> transposed_w_2dcoord = w_2d_coord.transpose();
    Eigen::Vector<int16_t, Eigen::Dynamic> w_2d_coord_16 = Eigen::Map<const Eigen::Vector<int16_t, Eigen::Dynamic>>(
        transposed_w_2dcoord.data(), transposed_w_2dcoord.size());
    this->logger_.debug("w_2d_coord_16 = %s", helper::EigenVectorToString(w_2d_coord_16).c_str());
    for (int i = 0; i < (collision_proj_num * 4); i++)
    {
        this->clsMMap.WriteSignedInt16(this->writtenAdr, w_2d_coord_16(i));
        this->writtenAdr += static_cast<int>(ByteSize::INT16);
    }
}

void UI_interface::preprocess_info()
{
    auto now = std::chrono::system_clock::now();
    this->sTime = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    this->logger_.info("Adr(IsWrite): %d", this->writtenAdr);
    //書き込み中フラグ
    this->clsMMap.begin_frame();

    // self.writtenAdr += BS.INT8
    this->writtenAdr = UNIX_TIME_ADR; //直前にReadOnlyアドレスがあるため明示的に指定.
    this->logger_.debug("Adr(unix_time): %d", this->writtenAdr);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    int64_t now_unix_time = now_ms - 1732600000000;
    //書込時間
    this->clsMMap.WriteInt64(this->writtenAdr, now_unix_time);
    // AppLogger.info(now_unix_time)　C++移植前からコメント
    this->writtenAdr += static_cast<int>(ByteSize::INT64);
}

void UI_interface::error_info(int isslow)
{
    this->logger_.debug("Adr(error): %d", this->writtenAdr);
    // エラー情報は専用MMAPで通知するため、この旧領域は0固定とする。
    static_cast<void>(isslow);
    constexpr int code = 0;
    //  エラー種別
    this->logger_.debug("code = %d", code);
    this->clsMMap.WriteInt32(this->writtenAdr, code);
    this->writtenAdr += static_cast<int>(ByteSize::INT32);
}
void UI_interface::machine_info(double angle_deg)
{
    this->logger_.info("Adr(angle): %d", this->writtenAdr);
    float angle = static_cast<float>(angle_deg);
    // 上部旋回体回転角度
    this->logger_.info("angle = %.2f", angle);
    this->clsMMap.WriteFloat(this->writtenAdr, angle);
    this->writtenAdr += static_cast<int>(ByteSize::FLOAT);
}
/**
 * multi_points: 3d BBoxの対角線座標(minx, miny, minz, maxx, maxy, maxz)
 * cluster2entity: 各クラスタのentity
 * 原点からの距離でソートする.
 */
Eigen::VectorXi UI_interface::select_cluster(const Eigen::Ref<const Eigen::MatrixXf>& multi_points,
                                             const std::map<int, NodeEntity>& cluster2entity) const
{
    std::vector<int> human_clusters;
    for (auto& [key, value] : cluster2entity)
    {
        if (value == NodeEntity::HUMAN)
        {
            human_clusters.push_back(key);
        }
    }
    // NOTE vectorからEigen::VecorXi変換
    // Eigen::Map<Eigen::VectorXi> np_human_clusters(human_clusters.data(),
    //                                               human_clusters.size());
    Eigen::MatrixXf first_two = multi_points.topLeftCorner(multi_points.rows(), 2);
    Eigen::MatrixXf last_two = multi_points.block(0, 3, multi_points.rows(), 2);
    Eigen::MatrixXf xy_center = (first_two + last_two) / 2.0;
    std::vector<double> xy_distance;
    xy_distance.reserve(xy_center.rows());
    for (int i = 0; i < xy_center.rows(); ++i)
    {
        double norm = xy_center.row(i).norm();
        xy_distance.push_back(norm);
    }
    // 距離を小さい順にソートするインデックス
    std::vector<int> sorted_indices(xy_distance.size());
    for (size_t i = 0; i < xy_distance.size(); ++i)
    {
        sorted_indices[i] = i;
    }
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&xy_distance](int i1, int i2) { return xy_distance[i1] < xy_distance[i2]; });

    // ソートされた元データ
    // sorted_xy = xy_center[sorted_indices]
    // sorted_multi_points = multi_points[sorted_indices, :]
    // selected_multi_points = sorted_multi_points[: self.bbox_3d_num, :]

    // 人属性がない場合はそのままreturn
    if (human_clusters.size() == 0)
    {
        Eigen::Map<Eigen::VectorXi> np_human_clusters(sorted_indices.data(), sorted_indices.size());
        return np_human_clusters;
    }

    // 人属性は必ず表示するように、先頭に持ってくる
    for (const auto& human_cluster : human_clusters)
    {
        sorted_indices.erase(std::remove(sorted_indices.begin(), sorted_indices.end(), human_cluster),
                             sorted_indices.end());
    }
    human_clusters.insert(human_clusters.end(), sorted_indices.begin(), sorted_indices.end());

    Eigen::Map<Eigen::VectorXi> np_human_clusters(human_clusters.data(), human_clusters.size());
    return np_human_clusters;
}

void UI_interface::detect_3d_info(const Eigen::Ref<const Eigen::MatrixXf>& minmax,
                                  const Eigen::Ref<const Eigen::VectorXi>& valid_detects,
                                  const std::map<int, NodeEntity>& cluster2entity)
{
    uint16_t cluster_3d_num = static_cast<uint16_t>(valid_detects(0));
    // cluster_3d_num = int(max(LS_pcd_det.labels))  # n_clusters

    if (cluster_3d_num == 0)
    {
        this->logger_.info("Adr(cluster_num): %d", this->writtenAdr);
        this->clsMMap.WriteInt8(this->writtenAdr, cluster_3d_num);
        this->logger_.info("cluster_3d_num = %d", cluster_3d_num);
        this->writtenAdr += static_cast<int>(ByteSize::INT8);
        return;
    }
    Eigen::MatrixXf multi_points = minmax.topLeftCorner(cluster_3d_num, minmax.cols())(Eigen::all, {0, 2, 4, 1, 3, 5});

    //点数変更前のindices
    Eigen::VectorXi raw_sorted_indices = this->select_cluster(multi_points, cluster2entity);
    Eigen::VectorXi sorted_indices;
    if (raw_sorted_indices.size() > this->bbox_3d_num)
    {
        sorted_indices = raw_sorted_indices.head(this->bbox_3d_num);
    }
    else
    {
        sorted_indices = raw_sorted_indices;
    }
    // 3D BBoxの数更新.
    cluster_3d_num = sorted_indices.size();
    // アプリ側で上限に制約をかける必要あるかも(要確認)
    this->logger_.info("Adr(cluster_num): %d", this->writtenAdr);
    this->clsMMap.WriteInt8(this->writtenAdr, cluster_3d_num);
    this->logger_.info("cluster_3d_num = %d", cluster_3d_num);
    this->writtenAdr += static_cast<int>(ByteSize::INT8);
    // cluster_3d_num が0の時は一度も処理をせずにpassする.
    // for i in range(cluster_3d_num):
    for (const auto& i : sorted_indices)
    {
        this->logger_.debug("i = %d", i);
        // entity書き込み
        NodeEntity obj_entity = cluster2entity.at(i);
        this->logger_.debug("Adr(obj_entity): %d", this->writtenAdr);
        ENTITY_FOR_UI entity;
        if (obj_entity == NodeEntity::HUMAN)
        {
            entity = ENTITY_FOR_UI::PERSON;
        }
        else
        {
            entity = ENTITY_FOR_UI::OTHER;
        }
        this->clsMMap.WriteInt8(this->writtenAdr, static_cast<int>(entity));
        this->writtenAdr += static_cast<int>(ByteSize::INT8);
        this->logger_.debug("entity_for_UI = %d", entity);

        // BBOX 8隅座標(対角線のみ)
        Eigen::VectorXf bbox_3d_points = multi_points.row(i);
        this->logger_.debug("bbox_3d_points = %s", helper::EigenVectorToString(bbox_3d_points).c_str());
        for (const auto& point : bbox_3d_points)
        {
            this->clsMMap.WriteFloat(this->writtenAdr, point);
            this->writtenAdr += static_cast<int>(ByteSize::FLOAT);
        }
    }
}
void UI_interface::collision_info()
{
    int collision_num = 0;
    if (this->collision_clusters.has_value())
    {
        collision_num = this->collision_clusters.value().size();
    }
    this->logger_.info("Adr(collision_num): %d", this->writtenAdr);
    this->clsMMap.WriteInt8(this->writtenAdr, collision_num);
    this->logger_.info("collision_num = %d", collision_num);
    this->writtenAdr += static_cast<int>(ByteSize::INT8);

    Eigen::MatrixXf w_3d_coord(collision_num, 6);
    this->logger_.debug("collision_pair_point = %d", this->writtenAdr);
    // collision_num が0の時は一度も処理をせずにpassする.
    if (collision_num != 0)
    {
        // 場合分け不要?

        int w_2d_coord_i = 0;
        for (const auto& [idx, data] : this->collision_clusters.value())
        {
            // w_coord_from
            constexpr size_t len_from = std::tuple_size<std::decay_t<decltype(std::get<2>(data))>>::value;
            // w_coord_to
            constexpr size_t len_to = std::tuple_size<std::decay_t<decltype(std::get<3>(data))>>::value;

            Eigen::VectorXf w_coord_from(len_from);

            w_coord_from(0) = std::get<0>(std::get<2>(data));
            w_coord_from(1) = std::get<1>(std::get<2>(data));
            w_coord_from(2) = std::get<2>(std::get<2>(data));

            Eigen::VectorXf w_coord_to(len_to);
            w_coord_to(0) = std::get<0>(std::get<3>(data));
            w_coord_to(1) = std::get<1>(std::get<3>(data));
            w_coord_to(2) = std::get<2>(std::get<3>(data));
            // 前半部分に w_coord_from、後半部分に w_coord_to を格納
            Eigen::VectorXf collision_points = hstack(w_coord_from.transpose(), w_coord_to.transpose()).transpose();
            w_3d_coord.row(w_2d_coord_i) = collision_points.transpose();

            auto distance = std::get<4>(data);
            auto radius = std::get<5>(data);
            std::string radius_s = radius.has_value() ? std::to_string(radius.value()) : "None";
            std::string idx_s = idx.has_value() ? std::to_string(idx.value()) : "None";
            this->logger_.debug("[idx: %s, w_coord_from: %s, w_coord_to: %s, distance: %lf, radius: %s]", idx_s.c_str(),
                               helper::EigenVectorToString(w_coord_from).c_str(),
                               helper::EigenVectorToString(w_coord_to).c_str(), distance, radius_s.c_str());
            w_2d_coord_i++;
        }
        //  衝突判定結果 座標
        // NOTE ravel
        Eigen::MatrixXf transposed_w_3d_coord = w_3d_coord.transpose();
        Eigen::VectorXf collision_pair =
            Eigen::Map<const Eigen::VectorXf>(transposed_w_3d_coord.data(), w_3d_coord.size());
        //  self._logger.info(self, f"{collision_pair = }") C++移植前からコメント
        for (int xyz_i = 0; xyz_i < collision_pair.size(); xyz_i++)
        {
            this->clsMMap.WriteFloat(this->writtenAdr, collision_pair(xyz_i));
            this->writtenAdr += static_cast<int>(ByteSize::FLOAT);
        }
    }
    //  衝突可能性あり警告
    this->logger_.info("Adr(collision_warning_level): %d", this->writtenAdr);
    this->clsMMap.WriteInt8(this->writtenAdr, static_cast<int>(this->collision_level));
    this->logger_.info("this->collision_level = %d", static_cast<int>(this->collision_level));
    this->writtenAdr += static_cast<int>(ByteSize::INT8);
}

/**
 * n_byteバイトだけ0を書き込む
 */
void UI_interface::zero_padding(int n_byte)
{
    for (int i = 0; i < n_byte; i++)
    {
        this->logger_.debug("Adr(zero_padding): %d", this->writtenAdr);
        this->clsMMap.WriteInt8(this->writtenAdr, 0);
        this->writtenAdr += static_cast<int>(ByteSize::INT8);
    }
}

void UI_interface::camera_info(const std::vector<cv::Mat>& frames,
                               // camera可変であるため、vector
                               const std::vector<CameraDetectionData>& bb_box_data,
                               const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                               const Eigen::Ref<const Eigen::MatrixXi>& valid_detects,
                               const std::vector<Camera>& camera, const std::map<int, NodeEntity>& cluster2entity)
{
    for (int i = 0; i < this->camera_num; i++)
    {
        cv::Mat frame = frames.at(i);
        // height
        // this->clsMMap.WriteInt32(this->writtenAdr, frame.rows);
        // this->writtenAdr += static_cast<int>(ByteSize::INT32);
        // width
        // this->clsMMap.WriteInt32(this->writtenAdr, frame.cols);
        // this->writtenAdr += static_cast<int>(ByteSize::INT32);

        // image_size
        int image_size = frame.cols * frame.rows;
        this->clsMMap.WriteInt32(this->writtenAdr, image_size);
        this->writtenAdr += static_cast<int>(ByteSize::INT32);

        cv::Mat flat = frame.reshape(1, frame.total() * frame.channels());
        std::vector<uchar> vec_frame = frame.isContinuous() ? flat : flat.clone();
        this->logger_.debug("cameraindex: %d, vec_frame.size: %ld, writtenAdr: %d", i, vec_frame.size(),
                           this->writtenAdr);

        // カメラ画像バイト数
        this->logger_.debug("Adr(cam.size): %d", this->writtenAdr);

        //  カメラ画像の実体（圧縮データ）
        this->logger_.debug("Adr(cam.data): %d", this->writtenAdr);
        this->clsMMap.WriteBytes(this->writtenAdr, vec_frame);
        this->writtenAdr += (vec_frame.size() * static_cast<int>(ByteSize::INT8));
        this->logger_.debug("vec_frame = %s", VectorToString(vec_frame).c_str());

        // 人検知結果(四角形BB個数、座標)
        this->write_2d_object_detection_result({frame.size().height, frame.size().width}, bb_box_data.at(i));

        // 立体物検知結果(直方体BB個数、座標)
        this->write_3d_object_detection_result_proj(boxes, valid_detects, camera.at(i), cluster2entity);
        // 衝突判定結果(衝突ペア個数、座標)
        Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> w_2d_coord =
            calc_collision_proj(camera.at(i), this->collision_clusters, this->draw_collision);
        this->write_collision_result_proj(w_2d_coord);
    }
}

void UI_interface::octotree_info(OctoTree& octotree_obj)
{
    // 属性ごとに点群を取得
    int octotree_pcd_num = 0;
    const int octotree_pcd_num_Adr = this->writtenAdr;
    //書き込み場所を記憶して、最後に戻って書く.
    this->logger_.info("Adr(octotree_pcd_num): %d", octotree_pcd_num_Adr);
    this->writtenAdr += static_cast<int>(ByteSize::INT32);
    // NOTE:
    // NodeEntityで明示的に点群を入れようと変更したので、入れたつもりでないNodeEntityはmemberから外す
    std::vector<NodeEntity> members = {NodeEntity::UNK, NodeEntity::HUMAN, NodeEntity::OTHER};
    for (const auto& member : members)
    {
        Eigen::MatrixXd member_points = octotree_obj.get_np_from_entity_octonodes_by_chunk({member});
        if (member_points.size() == 0)
        {
            // 点群が格納されていない空属性はスキップ
            continue;
        }
        this->logger_.debug("Adr(octotree_pcd): %d", this->writtenAdr);
        Eigen::MatrixXd transposed_member_points = member_points.transpose();
        Eigen::VectorXf member_points32 = Eigen::Map<const Eigen::VectorXd>(transposed_member_points.data(),
                                                                            transposed_member_points.size())
                                              .cast<float>(); // 8byte -> 4byteに変換
        int member_points_num = static_cast<int>(member_points32.size() / 3);
        this->logger_.debug("member = %d", static_cast<int>(member));
        this->logger_.debug("member_points_num = %d", member_points_num);
        // self._logger.info(f"{member_points32 = }") C++移植前からコメント
        octotree_pcd_num = octotree_pcd_num + member_points_num;
        if (judge_show_member(member, this->show_unk))
        {
            for (int point_i = 0; point_i < member_points32.size(); point_i++)
            {
                // HUMAN, OTHERクラスタに所属する点群. UNK
                // で検出される点群を表示.
                this->clsMMap.WriteFloat(this->writtenAdr, member_points32(point_i));
                this->writtenAdr += static_cast<int>(ByteSize::FLOAT);
                // this->_logger.info(self, point) C++移植前からコメント
            }
        }
    }
    this->logger_.info("Adr(octotree_pcd_num): %d", octotree_pcd_num_Adr);
    this->clsMMap.WriteInt32(octotree_pcd_num_Adr, octotree_pcd_num);
    this->logger_.info("octotree_pcd_num = %d", octotree_pcd_num);
}
void UI_interface::cliff_info()
{
    // 崖個数
    int n_cliffs = this->written_cliff_info.n_cliffs;
    this->logger_.debug("Adr(n_cliffs): %d", this->writtenAdr);
    clsMMap.WriteInt8(writtenAdr, n_cliffs);
    this->logger_.debug("n_cliffs = %d", n_cliffs);
    writtenAdr += static_cast<int>(ByteSize::INT8);

    // 崖境界の書き込み
    if (n_cliffs != 0)
    {
        Eigen::MatrixXf cliff_points = this->written_cliff_info.cliff_points.cast<float>();
        int ind = 0;
        // 各クラスタの崖境界毎に、個数とxy座標を書き込む。
        for (int cliff_vertices_i = 0; cliff_vertices_i < this->written_cliff_info.cliff_vertices.size();
             cliff_vertices_i++)
        {
            this->logger_.debug("Adr(cliff_vertices): %d", this->writtenAdr);
            clsMMap.WriteInt16(writtenAdr, this->written_cliff_info.cliff_vertices(cliff_vertices_i));
            this->logger_.debug("cliff_vertices(%d) = %d", cliff_vertices_i,
                               this->written_cliff_info.cliff_vertices(cliff_vertices_i));

            writtenAdr += static_cast<int>(ByteSize::INT16);

            // 対応する崖の点群の書き込み（各点はxy座標）
            // cliff_points[ind] ~ cliff_points[ind + cliff_vertices - 1]
            for (int target_cliff_point_i = ind;
                 target_cliff_point_i < ind + this->written_cliff_info.cliff_vertices(cliff_vertices_i);
                 ++target_cliff_point_i)
            {
                const auto& target_cliff_point = cliff_points.row(target_cliff_point_i);
                // x座標の書き込み
                this->logger_.debug("target_cliff_point: %s",
                                   helper::EigenMatrixToString(target_cliff_point, true).c_str());
                clsMMap.WriteFloat(writtenAdr, target_cliff_point(0));
                writtenAdr += static_cast<int>(ByteSize::FLOAT);
                // y座標の書き込み
                clsMMap.WriteFloat(writtenAdr, target_cliff_point(1));
                writtenAdr += static_cast<int>(ByteSize::FLOAT);
            }
            // インデックス更新
            ind += this->written_cliff_info.cliff_vertices(cliff_vertices_i);
        }
    }
    // 崖検知警告
    CLIFF_LEVEL cliff_det_level = this->written_cliff_info.cliff_det_level;
    this->logger_.debug("Adr(cliff_det_level): %d", this->writtenAdr);
    this->clsMMap.WriteInt8(this->writtenAdr, static_cast<int>(cliff_det_level));
    this->logger_.debug("cliff_det_level: %d", static_cast<int>(cliff_det_level));
    this->writtenAdr += static_cast<int>(ByteSize::INT8);
}

/**
 * 接触可能性探索の対象としている機体, 周辺立体物,
 * 衝突発生個所の情報を共有メモリに書き込む処理
 *
 * 1. 各種データの数と、そのデータの中心座標のxyzを書き込む
 * 2. write_onがFalseの場合は、各種データの数を0として書き込みを行う
 * 3. 各種データの数が0の場合は、数は書き込むがxyz座標の書き込みは行わない
 */
void UI_interface::vis_octree_info(bool write_on, int vis_tree_depth, const OctoTree& octotree_obj_pcd)
{
    Eigen::MatrixXd w_min_range = octotree_obj_pcd.min_xyz;
    Eigen::MatrixXd w_max_range = octotree_obj_pcd.max_xyz;

    // 書き込まない場合も要素数0を書き込むため、空行列を変数に入れる
    Eigen::MatrixXd voxmed_intersection = Eigen::MatrixXd::Zero(0, 0);
    Eigen::MatrixXd voxmed_machine = Eigen::MatrixXd::Zero(0, 0);
    Eigen::MatrixXd voxmed_pcd = Eigen::MatrixXd::Zero(0, 0);

    if (write_on)
    {
        // 書き込む場合、LiDAR点群や機体点群を基に、衝突部位,
        // 非衝突部位の計算を行う
        auto [new_voxmed_intersection, new_voxmed_machine, new_voxmed_pcd] = create_voxmed_existing_cell_by_entity(
            vis_tree_depth, octotree_obj_pcd, w_max_range, w_min_range,
            {NodeEntity::CRANE_IMMOBILE_FOR_DET, NodeEntity::CRANE_MOBILE_FOR_DET},
            {NodeEntity::UNK, NodeEntity::OTHER, NodeEntity::HUMAN});
        //スコープが外れるので値をコピーする。
        voxmed_intersection = new_voxmed_intersection;
        voxmed_machine = new_voxmed_machine;
        voxmed_pcd = new_voxmed_pcd;
    }
    // 4 byteにして、
    // 機体, 周辺立体物, 衝突発生個所の順番で共有メモリに書き込む
    WriteFloatArray(voxmed_machine.cast<float>(), "machine");
    WriteFloatArray(voxmed_pcd.cast<float>(), "pcd");
    WriteFloatArray(voxmed_intersection.cast<float>(), "machine_pcd_intersection");
}

// 元の名前 _write_float
void UI_interface::WriteFloat(float val)
{
    this->clsMMap.WriteFloat(this->writtenAdr, val);
    this->writtenAdr += static_cast<int>(ByteSize::FLOAT);
}

/**
 * float型のndarrayの配列を共有メモリに書き込む関数, 複数出てくるので関数化
 * 元の名前 _write_ndarray_float
 */
void UI_interface::WriteFloatArray(const Eigen::Ref<const Eigen::MatrixXf>& array,
                                   const std::optional<std::string>& array_name)
{
    std::string _array_name = array_name.has_value() ? array_name.value() : "array";

    int array_len = array.rows();
    this->logger_.debug("Adr(_array_name): %d", this->writtenAdr);
    this->clsMMap.WriteInt16(this->writtenAdr, array_len);
    this->logger_.debug("array_len = %d", array_len);
    this->writtenAdr += static_cast<int>(ByteSize::INT16);

    // 配列長さが0の時は、要素を書き込まないので、early returnする
    if (array_len == 0)
    {
        return;
    }

    //  配列の各要素を書き込む
    assert(array.cols() == 3);
    //"各行の要素数が3である必要があります"
    this->logger_.debug("Adr(%s_point): %d", _array_name.c_str(), this->writtenAdr);
    for (int xyz_i = 0; xyz_i < array.rows(); xyz_i++)
    {
        WriteFloat(array(xyz_i, 0));
        WriteFloat(array(xyz_i, 1));
        WriteFloat(array(xyz_i, 2));
    }
}

void UI_interface::postprocess_info(int ref_t, int process_time_ms)
{
    //  処理フレーム番号をデバッグ用に記述
    this->logger_.info("Adr(frame_num): %d", this->writtenAdr);
    this->clsMMap.WriteInt64(this->writtenAdr, ref_t);
    this->logger_.info("ref_t = %d", ref_t);
    this->writtenAdr += static_cast<int>(ByteSize::INT64);

    // 追加インターフェース（処理時間）
    // this->_logger.info(f"Adr(process_time_ms): {this->writtenAdr!s}")
    this->clsMMap.WriteInt32(this->writtenAdr, process_time_ms);
    // this->_logger.info(f"{ref_t = }")
    this->writtenAdr += static_cast<int>(ByteSize::INT32);

    // 追加インターフェース（負荷低減モード）
    // 負荷低減モード
    int is_load_reduction = 0; //暫定固定値
    // this->_logger.info(f"Adr(is_load_reduction): {this->writtenAdr!s}")
    this->clsMMap.WriteInt8(this->writtenAdr, is_load_reduction);
    // this->_logger.info(f"{ref_t = }")
    this->writtenAdr += static_cast<int>(ByteSize::INT8);

    const bool is_initial = (ref_t == this->s_frame);
    this->clsMMap.end_frame(is_initial);

    auto now_time = std::chrono::system_clock::now().time_since_epoch();
    double now = std::chrono::duration_cast<std::chrono::milliseconds>(now_time).count();

    double MMapdelta = (now - this->sTime);
    double Alldelta = (now - this->preProcessTime);
    this->logger_.info("Adr(last): %d, MMapdelta: %.2lf msec, Alldelta:  %.2lf msec", this->writtenAdr, MMapdelta,
                       Alldelta);
    this->preProcessTime = now;

    this->writtenAdr = Start_ADR;
    this->logger_.info("writtenAdr Reset!");
}

void UI_interface::damp_info()
{
    if (!damp_out)
        return;
    const size_t n = this->clsMMap.get_file_index();
    for (size_t i = 0; i < n; ++i)
    {
        auto bin = this->clsMMap.ReadBytes(i, Start_ADR, -1);
        if (bin.has_value())
        {
            log_mmap(this->damp_fp_list.at(i), bin.value());
        }
        else
        {
            throw std::runtime_error("no bin_data");
        }
    }
}

/**
 *  UI_IFのパラメータ変更を反映させる関数
 * 単純な代入で済むためcheck関数は作成しない
 */
void UI_interface::update_value(const UIIFConf& ui_if, const GeneralConf& general, const LoggerFunc logfunc)
{

    this->bbox_3d_num = ui_if.bbox_3d_num;
    this->bbox_3d_distance = ui_if.bbox_3d_distance;
    this->show_unk = ui_if.show_unk;
    this->collision_depict_dist = ui_if.collision_depict_dist;
    this->collision_attention_dist = ui_if.collision_attention_dist;
    this->collision_warning_dist = ui_if.collision_warning_dist;
    this->draw_bbox_3d = ui_if.draw_bbox_3d;
    this->draw_collision = ui_if.draw_collision;
    this->rotation_radius = general.rotation_radius;
    this->has_external_guard = general.has_external_guard;
    this->external_guard_offset = general.external_guard_offset;
    this->logger_ = std::move(logfunc);
}

MMapProtocol UI_interface::createProtocol() const
{
    return MMapProtocol{.IsWriting_ADR = UI_interface::IsWriting_ADR,
                        .IsReading_ADR = UI_interface::IsReading_ADR,
                        .Start_ADR = UI_interface::Start_ADR,
                        .UNIX_TIME_ADR = UI_interface::UNIX_TIME_ADR};
}

MMapConfig UI_interface::createMMapConfig(std::vector<std::string> mmapPaths) const
{
    return MMapConfig{.paths = mmapPaths,
                      .map_size_bytes = static_cast<size_t>(ByteSize::MAP_ALL),
                      .endian = ByteOrder::LITTLE,
                      .proto = createProtocol()};
}
