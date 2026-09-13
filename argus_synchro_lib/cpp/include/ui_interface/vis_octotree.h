#pragma once
#include "octotree/NodeEntity.h"
#include "octotree/OctoTree.h"
#include <Eigen/Core>
#include <memory>
// #include <open3d/Open3D.h>
#include <tuple>
#include <vector>

// using namespace open3d;

// std::shared_ptr<geometry::LineSet> create_unit_bbox_by_lineset(
//     const std::optional<std::tuple<double, double, double>> &color_list =
//         std::nullopt,
//     std::optional<Eigen::MatrixXd> trans_vec = std::nullopt,
//     std::optional<Eigen::MatrixXd> scale = std::nullopt);

std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
_vox2w_coords(const Eigen::Ref<const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>>& vox_coords,
              int tree_depth, const Eigen::Ref<const Eigen::VectorXd>& w_max_range,
              const Eigen::Ref<const Eigen::VectorXd>& w_min_range, std::tuple<double, double, double> offset);

std::tuple<Eigen::Matrix<int64_t, Eigen::Dynamic, 3>, Eigen::Matrix<int64_t, Eigen::Dynamic, 3>,
           Eigen::Matrix<int64_t, Eigen::Dynamic, 3>>
_split_intersection(const Eigen::Ref<const Eigen::MatrixXd>& np_source,
                    const Eigen::Ref<const Eigen::MatrixXd>& np_dest);

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
create_voxmed_existing_cell_by_entity(int vis_tree_depth, OctoTree octotree_obj,
                                      const Eigen::Ref<const Eigen::VectorXd>& w_max_range,
                                      const Eigen::Ref<const Eigen::VectorXd>& w_min_range,
                                      const std::vector<NodeEntity>& src_entities = {NodeEntity::CRANE_IMMOBILE_FOR_DET,
                                                                                     NodeEntity::CRANE_MOBILE_FOR_DET},
                                      const std::vector<NodeEntity>& dest_entities = {
                                          NodeEntity::UNK,
                                          NodeEntity::OTHER,
                                          NodeEntity::HUMAN,
                                      });