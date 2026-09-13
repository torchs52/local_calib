#pragma once

#include "logger/py_logger.h"
#include "ui_interface/GeneralConf.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoNode.h"
#include "octotree/OctoTree.h"
#include "ui_interface/UIIFConf.h"
#include "ui_interface/ui_interface.h"

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

class GodotUIVisualizer
{
  private:
    PyLogger logger_;
    bool CheckValidVisOctree(bool octotree_func_on, bool collisiondetection_func_on, bool display_octree) const;

  public:
    explicit GodotUIVisualizer(const UIIFConf& ui_if_config, int s_frame, double rotation_radius, int camera_num,
                               bool has_external_guard, double external_guard_offset,
                               const std::string& status_mmap_path, const LoggerFunc logfunc);

    void update(const UIIFConf& ui_if_config, const GeneralConf& general_config, const LoggerFunc logfunc);

    void summary(int isslow, const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                 const Eigen::Ref<const Eigen::MatrixXf>& minmax,
                 const Eigen::Ref<const Eigen::VectorXi>& valid_detects, OctoTree octotree_obj, double angle_deg,
                 const std::vector<cv::Mat>& frames, const std::vector<CameraDetectionData>& bb_box_data,
                 const std::vector<Camera>& camera, Ccol_res collision_clusters,
                 const std::map<int, NodeEntity>& cluster2entity, int ref_t, int max_tree_depth, int dialate_point_size,
                 bool octotree_func_on, bool collisiondetection_func_on, bool display_octree, bool damp_out,
                 int process_time_ms);
    void close_mmap();

  private:
    UI_interface ui_if;
};