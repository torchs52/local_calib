#pragma once

#include "logger/py_logger.h"

#include <tuple>
#include <string>
#include <Eigen/Core>

// *** クラス ***
class PcdDet
{
  public:
    static constexpr Eigen::Index MAX_TOTAL_SIZE = 500;
    static constexpr Eigen::Index MAX_TOTAL_COLLISION_SIZE = 20;
    static constexpr Eigen::Index MAX_TOTAL_EDGE_SIZE = 50;
    static constexpr Eigen::Index MAX_CELL_NUM = 32768;
};

// *** 関数プロトタイプ宣言 ***

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi>
main_accum(const Eigen::Ref<const Eigen::MatrixXd>& xyz, const std::string& debug_log, double eps, int min_samples,
           LoggerFunc logfunc);
