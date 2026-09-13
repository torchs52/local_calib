#include "ui_interface/visualizer.h"
#include "ui_interface/GeneralConf.h"
#include "ui_interface/UIIFConf.h"
#include "ui_interface/ui_interface.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoNode.h"
#include "octotree/OctoTree.h"
#include "logger/py_logger.h"

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

GodotUIVisualizer::GodotUIVisualizer(const UIIFConf& ui_if_config, int s_frame, double rotation_radius, int camera_num,
                                     bool has_external_guard, double external_guard_offset,
                                     const std::string& status_mmap_path, const LoggerFunc logfunc)
    : ui_if(ui_if_config, s_frame, rotation_radius, camera_num, has_external_guard, external_guard_offset,
            status_mmap_path, logfunc),
      logger_(PyLogger(logfunc))
{
}

void GodotUIVisualizer::update(const UIIFConf& ui_if_config, const GeneralConf& general_config,
                               const LoggerFunc logfunc)
{
    ui_if.update_value(ui_if_config, general_config, logfunc);
    logger_ = PyLogger(logfunc);
}

// check_valid_vis_octree
bool GodotUIVisualizer::CheckValidVisOctree(bool octotree_func_on, bool collisiondetection_func_on,
                                            bool display_octree) const
{
    return octotree_func_on && collisiondetection_func_on && display_octree;
}

void GodotUIVisualizer::summary(int isslow, const Eigen::Ref<const Eigen::MatrixXd>& boxes,
                                const Eigen::Ref<const Eigen::MatrixXf>& minmax,
                                const Eigen::Ref<const Eigen::VectorXi>& valid_detects, OctoTree octotree_obj,
                                double angle_deg, const std::vector<cv::Mat>& frames,
                                const std::vector<CameraDetectionData>& bb_box_data, const std::vector<Camera>& camera,
                                Ccol_res collision_clusters, const std::map<int, NodeEntity>& cluster2entity, int ref_t,
                                int max_tree_depth, int dialate_point_size, bool octotree_func_on,
                                bool collisiondetection_func_on, bool display_octree, bool damp_out,
                                int process_time_ms)
{
    // vis_tree_depth = max_tree_depth - dialate_point_size
    int vis_tree_depth = max_tree_depth - dialate_point_size;

    ui_if.preprocess_info();
    ui_if.error_info(isslow);
    ui_if.machine_info(angle_deg);
    ui_if.set_collision_clusters_info(collision_clusters);
    ui_if.set_cliff_info_by_octreee(octotree_obj);
    ui_if.camera_info(frames, bb_box_data, boxes, valid_detects, camera, cluster2entity);
    ui_if.octotree_info(octotree_obj);
    ui_if.detect_3d_info(minmax, valid_detects, cluster2entity);
    ui_if.collision_info();
    ui_if.zero_padding(1);

    bool valid_vis_octree = this->CheckValidVisOctree(octotree_func_on, collisiondetection_func_on, display_octree);

    ui_if.vis_octree_info(valid_vis_octree, vis_tree_depth, octotree_obj);
    ui_if.cliff_info();
    ui_if.postprocess_info(ref_t, process_time_ms);
    if (damp_out)
    {
        this->logger_.info("=======damp_mmap=======");
        ui_if.damp_info();
    }
}

void GodotUIVisualizer::close_mmap()
{
    ui_if.close_mmap();
}