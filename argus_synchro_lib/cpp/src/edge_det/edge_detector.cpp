#include "octotree/edge_detector.h"

#include <algorithm>
#include <iostream>
#include <memory>

namespace edge_det
{
EdgeDetectionResult::EdgeDetectionResult(int frame_, double time_, Eigen::MatrixXd edge_points_,
                                         Eigen::MatrixXd edge_lines_, Eigen::VectorXi edge_length_)
    : frame(frame_), time(time_), edge_points(std::move(edge_points_)), edge_lines(std::move(edge_lines_)),
      edge_length(std::move(edge_length_))
{
}

Eigen::MatrixXd EdgeDetectionResult::get_edge_points_on_ground() const
{
    const int dst_rows = (this->edge_points.rows() + 1) / 2;
    const int cols = this->edge_points.cols();

    Eigen::MatrixXd result(dst_rows, cols);
    int dst_r = 0;
    for (int row = 0; row < this->edge_points.rows(); row += 2)
    {
        result.row(dst_r++) = this->edge_points.row(row);
    }
    return result;
}
std::pair<Eigen::MatrixXd, Eigen::VectorXi> EdgeDetectionResult::get_edge_cluster() const
{
    Eigen::MatrixXd edge_points_ground = this->get_edge_points_on_ground();
    const int num_points = edge_points_ground.rows();

    Eigen::VectorXi edge_cluster(num_points);
    edge_cluster.setZero();

    int ind = 0;
    for (int cluster_id = 0; cluster_id < edge_length.size(); ++cluster_id)
    {
        const int offset = edge_length(cluster_id);

        assert(ind + offset <= num_points);
        edge_cluster.segment(ind, offset).setConstant(cluster_id);
        ind += offset;
    }

    return {edge_points_ground, edge_cluster};
}

EdgeDetectionConfig::EdgeDetectionConfig(float target_edge_dist_th_) : target_edge_dist_th(target_edge_dist_th_)
{
}

EdgeDetectorCpp::EdgeDetectorCpp()
{
}

/**
 * @brief 崖検出のメイン処理を記述するメソッド,
 * 空の結果EdgeDetectionResultインスタンスを返すように書いているが、実装時は修正する
 *
 * @param octotree_obj
 * @param target_entities
 */
EdgeDetectionResult EdgeDetectorCpp::main(OctoTree& octotree_obj, std::vector<NodeEntity>& target_entities)
{
    return EdgeDetectionResult(0, 0.0, Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::VectorXi());
}

/**
 * @brief app_config更新時の処理, インスタンス変数の一部に変更が必要な場合は変更を行う
 *
 */
void EdgeDetectorCpp::update(EdgeDetectionConfig edge_conf)
{
}
} // namespace edge_det
