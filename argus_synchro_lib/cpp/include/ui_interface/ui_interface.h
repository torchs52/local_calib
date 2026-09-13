#pragma once

#include "logger/py_logger.h"
#include "clsmmap/ClsMMap.h"
#include "ui_interface/GeneralConf.h"
#include "ui_interface/IFCliffInfo.h"
#include "ui_interface/UIIFConf.h"
#include "ui_interface/status_mmap.h"
#include "octotree/OctoTree.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

enum class ByteSize : int
{
    UNIT = 1,
    INT8 = 1,  // Char
    INT16 = 2, // Short
    INT32 = 4, // long
    INT64 = 8, // longlong
    FLOAT = 4,
    MAP_ALL = 10 * 1024 * 1024
    // MAP_ALL = 10 * 1024 * 15  // damp時に巨大すぎないような一時指定
};

class UI_interface
{
  private:
    constexpr static int Start_ADR = 0;
    constexpr static int IsWriting_ADR = 0; // 1byte
    constexpr static int IsReading_ADR = 1; // 1byte
    constexpr static int UNIX_TIME_ADR = 2; // 8byte
    constexpr static int Error_ADR = 10;    // 4byte
    constexpr static int CamImg_ADR = 14;   // ここまでは固定アドレス

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    UI_interface(const UIIFConf& ui_if, int s_frame, double rotation_radius, int camera_num, bool has_external_guard,
                 double external_guard_offset, const std::string& status_mmap_path, const LoggerFunc logfunc);
    ~UI_interface();
    void close_mmap();
    void set_collision_clusters_info(Ccol_res& collision_clusters);
    void set_cliff_info_by_octreee(OctoTree& octotree_obj);
    void camera_info(const std::vector<cv::Mat>& frames, const std::vector<CameraDetectionData>& bb_box_data,
                     const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                     const Eigen::Ref<const Eigen::MatrixXi>& valid_detects, const std::vector<Camera>& camera,
                     const std::map<int, NodeEntity>& cluster2entity);
    void octotree_info(OctoTree& octotree_obj);
    void cliff_info();
    void preprocess_info();
    void error_info(int isslow);
    void write_2d_object_detection_result(const std::tuple<int, int>& frame_shape,
                                          const CameraDetectionData& bb_box_data);
    void write_3d_object_detection_result_proj(const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                                               const Eigen::Ref<const Eigen::VectorXi>& valid_detects,
                                               const Camera& camera, const std::map<int, NodeEntity>& cluster2entity);
    void write_collision_result_proj(
        const Eigen::Ref<const Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>>& w_2d_coord);
    void machine_info(double angle_deg);
    Eigen::VectorXi select_cluster(const Eigen::Ref<const Eigen::MatrixXf>& multi_points,
                                   const std::map<int, NodeEntity>& cluster2entity) const;
    void detect_3d_info(const Eigen::Ref<const Eigen::MatrixXf>& minmax,
                        const Eigen::Ref<const Eigen::VectorXi>& valid_detects,
                        const std::map<int, NodeEntity>& cluster2entity);
    void collision_info();
    void zero_padding(int n_byte);
    void vis_octree_info(bool write_on, int vis_tree_depth, const OctoTree& octotree_obj_pcd);
    void postprocess_info(int ret_t, int process_time_ms);
    void damp_info();
    void update_value(const UIIFConf& ui_if, const GeneralConf& general, const LoggerFunc logfunc);

  private:
    MMapConfig createMMapConfig(std::vector<std::string> mmapPaths) const;
    MMapProtocol createProtocol() const;

    bool isLittleEndian();

    void WriteFloat(float val);
    void WriteFloatArray(const Eigen::Ref<const Eigen::MatrixXf>& array, const std::optional<std::string>& array_name);

    double sTime;
    double preProcessTime;
    bool damp_out;
    int bbox_3d_num;
    double bbox_3d_distance;
    std::vector<std::string> dampPathList;
    bool show_unk;
    double collision_depict_dist;
    double collision_attention_dist;
    double collision_warning_dist;
    double cliff_attention_dist;
    double cliff_warning_dist;
    bool draw_bbox_3d;
    bool draw_collision;
    int s_frame;
    double rotation_radius;
    int camera_num;
    classMMap clsMMap;
    std::vector<std::ofstream> damp_fp_list;
    int writtenAdr;
    COLLISION_LEVEL collision_level;
    IFCliffInfo written_cliff_info;
    bool has_external_guard;
    double external_guard_offset;
    StatusMMAP status; // ステータス操作用のクラス
    std::optional<Ccol_res> collision_clusters;
    PyLogger logger_;
};
