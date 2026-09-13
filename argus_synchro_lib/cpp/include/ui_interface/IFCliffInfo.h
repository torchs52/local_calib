#pragma once

#include "clsmmap/ClsMMap.h"
#include "octotree/NodeEntity.h"
#include "octotree/OctoNode.h"
#include "dataclass/camera.h"
#include "logger/py_logger.h"
#include <Eigen/Core>
#include <map>
constexpr int NUM_CLASSES = 1; // 物体検知クラスの種類数

// constexpr int ROTATION_RADIUS 4.2  //
// 旋回半径(いずれ機種別パラメータを読み込む)

using Ccol_res =
    std::map<std::optional<int>, std::tuple<OctoNode, OctoNode, std::tuple<double, double, double>,
                                            std::tuple<double, double, double>, double, std::optional<double>>>;

// _logger_ui_if = AppLoggerFactory.from_name("UI_IF")

enum class ENTITY_FOR_UI : int
{
    OTHER = 0,
    PERSON = 1
};
enum class COLLISION_LEVEL : int
{
    NORMAL = 0,
    ATTENTION = 1,
    WARNING = 2
};
enum class CLIFF_LEVEL : int
{
    NORMAL = 0,    //なし
    ATTENTION = 1, //注意
    WARNING = 2    //警告
};

struct IFCliffInfo
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int n_cliffs;
    Eigen::VectorXi cliff_vertices;
    Eigen::MatrixXd cliff_points;
    CLIFF_LEVEL cliff_det_level;
    IFCliffInfo();
    IFCliffInfo(int n_cliffs, const Eigen::Ref<const Eigen::VectorXi>& cliff_vertices,
                const Eigen::Ref<const Eigen::MatrixXd>& cliff_points, CLIFF_LEVEL cliff_det_level);
};

void log_mmap(std::ofstream& damp_fp, const Eigen::Ref<const Eigen::Vector<uint8_t, Eigen::Dynamic>>& bindata);

bool judge_show_member(NodeEntity member, bool show_unk);

Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>
calc_collision_proj(const Camera& camera, const std::optional<Ccol_res>& collision_clusters, bool draw_collision);
// Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>

Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>
calc_bounding_box_around_entity(const Camera& camera, const Eigen::MatrixXd& boxes,
                                const std::map<int, NodeEntity>& cluster2entity, bool draw_bbox_3d,
                                const PyLogger& logger, NodeEntity target_entity = NodeEntity::HUMAN);