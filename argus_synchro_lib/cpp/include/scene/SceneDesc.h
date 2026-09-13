#pragma once

#include <iostream>
#include <tuple>
#include <string>
#include <map>
#include <vector>
#include <Eigen/Dense>

#include "octotree/OctoTree.h"
#include "octotree/NodeEntity.h"
#include "dataclass/camera.h"
#include "scene/app_config.h"
#include "scene/common.h"
#include "logger/py_logger.h"

std::tuple<double, double, double, double> calc_iou(double ax_min, double ay_min, double ax_max, double ay_max,
                                                    double bx_min, double by_min, double bx_max, double by_max);

struct PassesVerticalHardGateOptions
{
    std::optional<double> box_h_px = std::nullopt;
    double vIoU_min = 0.30;
    double height_ratio_max = 0.60;
    double phi_max_deg = 8.0;
};

class Scene
{
  private:
    PyLogger logger_;
    std::tuple<double, double, double> vertical_consistency_scores(const Eigen::Vector4f& box2d, int width, int height,
                                                                   const Eigen::Matrix<float, 8, 2>& proj8);

    double augment_cost_with_verticals(double base_cost, double vIoU, double height_ratio, double phi_rad,
                                       std::optional<double> w1 = std::nullopt, std::optional<double> w2 = std::nullopt,
                                       std::optional<double> w3 = std::nullopt);

    bool passes_coarse_height_gate(const Eigen::Vector4f& box2d, int width, int height,
                                   const Eigen::Matrix<float, 8, 2>& proj8, std::optional<double> lo = std::nullopt,
                                   std::optional<double> hi = std::nullopt);

    bool passes_vertical_hard_gate(double vIoU, double height_ratio, double phi_rad,
                                   const PassesVerticalHardGateOptions& opts = {});

    int correspondence_by_endpoints(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds,
                                    int num_3d);

    int correspondence_by_iou(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds,
                              int num_3d);

    int correspondence_by_center(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds,
                                 int num_3d);

    int get_human_3bb(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds, int num_3d,
                      std::string method = "center");

    bool passes_human_size(const Eigen::VectorXf& minmax_tuple);

  public:
    using t_cluster2entity = std::map<std::optional<int>, NodeEntity>;

    // メンバ変数としてパラメータを定義
    double coarse_lo;
    double coarse_hi;
    double k_min;
    int h_ref_px;
    double lo_gain;
    double hi_gain;
    double lo_floor;
    double hi_ceil;
    double vertical_w_iou;
    double vertical_w_scale;
    double vertical_w_phi;
    double final_threshold;
    bool use_human_gate;
    double H_min;
    double H_max;
    double W_min;
    double W_max;
    double D_min;
    double D_max;
    double tall_ratio_min;

    Scene()
        : coarse_lo(0), coarse_hi(0), k_min(0), h_ref_px(0), lo_gain(0), hi_gain(0), lo_floor(0), hi_ceil(0),
          vertical_w_iou(0), vertical_w_scale(0), vertical_w_phi(0), final_threshold(0), use_human_gate(0), H_min(0),
          H_max(0), W_min(0), W_max(0), D_min(0), D_max(0), tall_ratio_min(0), logger_(nullptr)
    {
    }

    Scene(const SceneDescriptionConf& scene_conf, const LoggerFunc logfunc);

    void update(const SceneDescriptionConf& scene_conf, const LoggerFunc logfunc);

    t_cluster2entity integrate2d3d(const Camera& camera, const CameraDetectionData& bb_box_data,
                                   const Eigen::MatrixXd& boxes, const Eigen::MatrixXf& minmax,
                                   const int& valid_detects, const NodeEntity& from_entity = NodeEntity::OTHER,
                                   const std::string& method = "center");

    std::tuple<OctoTree, t_cluster2entity>
    aggregate2d3d_results(const std::vector<t_cluster2entity>& camera_cluster2entities, const OctoTree& octotree_obj,
                          const NodeEntity& from_entity = NodeEntity::OTHER);

    t_py_col_res append_distance_info(const t_py_col_res& collision_clusters, const Eigen::MatrixXf& minmax,
                                      const Eigen::Vector3d& origin = Eigen::Vector3d(0.0, 0.0, 0.0));
};
