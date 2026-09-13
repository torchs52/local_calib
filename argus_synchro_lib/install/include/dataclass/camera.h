#pragma once

#include <Eigen/Core>

struct Camera
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int cam_index;
    int width;
    int height;
    Eigen::MatrixXd rvec;
    Eigen::Vector3d tvec;
    Eigen::Matrix3d ncm1;
    Eigen::Matrix4d extrmat;

    Camera(int cam_index_, int width_, int height_, const Eigen::MatrixXd& rvec_, const Eigen::Vector3d& tvec_,
           const Eigen::Matrix3d& ncm1_, const Eigen::Matrix4d& extrmat_)
        : cam_index(cam_index_), width(width_), height(height_), rvec(rvec_), tvec(tvec_), ncm1(ncm1_),
          extrmat(extrmat_)
    {
    }
};

struct CameraDetectionData
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::MatrixXf boxes;
    Eigen::MatrixXf scores;
    Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic> classes;
    int valid_detects;

    CameraDetectionData(const Eigen::MatrixXf& boxes_, const Eigen::MatrixXf& scores_,
                        const Eigen::Matrix<int64_t, Eigen::Dynamic, Eigen::Dynamic>& classes_, int valid_detects_)
        : boxes(boxes_), scores(scores_), classes(classes_), valid_detects(valid_detects_)
    {
    }
};
