#include "ui_interface/IFCliffInfo.h"
#include "ui_interface/helper.h"
#include "octotree/NodeEntity.h"
#include "logger/py_logger.h"
#include <Eigen/Dense>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sstream>

IFCliffInfo::IFCliffInfo(int n_cliffs, const Eigen::Ref<const Eigen::VectorXi>& cliff_vertices,
                         const Eigen::Ref<const Eigen::MatrixXd>& cliff_points, CLIFF_LEVEL cliff_det_level)
    : n_cliffs(n_cliffs), cliff_det_level(cliff_det_level)
{
    this->cliff_vertices = cliff_vertices;
    this->cliff_points = cliff_points;
};

IFCliffInfo::IFCliffInfo()
    : n_cliffs(0), cliff_vertices(Eigen::VectorXi()), cliff_points(Eigen::MatrixXd()),
      cliff_det_level(CLIFF_LEVEL::NORMAL){};

void log_mmap(std::ofstream& damp_fp, const Eigen::Ref<const Eigen::Vector<uint8_t, Eigen::Dynamic>>& bindata)
{
    // 16 進数表記に変換して文字列化
    // 各バイトを 2 桁の大文字 16 進数で表記
    damp_fp.setf(std::ios::uppercase);
    for (size_t i = 0; i < bindata.size(); i++)
    {
        damp_fp << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bindata(i));
        if (i != bindata.size() - 1)
        {
            damp_fp << " ";
        }
    }
    damp_fp << std::endl;
    damp_fp.flush();
    // AppLogger.info("ui_if",hex_string) C++移行前からコメント
    // テキストファイルに書き込み
};

bool judge_show_member(NodeEntity member, bool show_unk)
{
    return (member == NodeEntity::HUMAN) || (member == NodeEntity::OTHER) || ((member == NodeEntity::UNK) && show_unk);
}

Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>
calc_collision_proj(const Camera& camera, const std::optional<Ccol_res>& collision_clusters, bool draw_collision)
{
    Eigen::Matrix<int16_t, Eigen::Dynamic, 4> w_2d_coord;
    if (!draw_collision)
    {
        return w_2d_coord;
    }
    if (!collision_clusters.has_value() || collision_clusters.value().empty())
    {
        return w_2d_coord;
    }
    for (const auto& [idx, value] : collision_clusters.value())
    {
        // w_coord1を始点, w_coord2を終点にして線を作る
        // value 2 : w_coord_from
        Eigen::Vector3d w_coord_from;
        w_coord_from << std::get<0>(std::get<2>(value)), std::get<1>(std::get<2>(value)),
            std::get<2>(std::get<2>(value));
        // value 3 : w_coord_to
        Eigen::Vector3d w_coord_to;
        w_coord_to << std::get<0>(std::get<3>(value)), std::get<1>(std::get<3>(value)), std::get<2>(std::get<3>(value));

        std::vector<cv::Point2d> w_coord;
        cv::Mat objectPoints =
            (cv::Mat_<double>(2, 3) << std::get<0>(std::get<2>(value)), std::get<1>(std::get<2>(value)),
             std::get<2>(std::get<2>(value)), std::get<0>(std::get<3>(value)), std::get<1>(std::get<3>(value)),
             std::get<2>(std::get<3>(value)));

        cv::Mat cv2_rvec;
        cv::Mat cv2_tvec;
        cv::Mat cv2_ncm1;
        cv::eigen2cv(camera.rvec, cv2_rvec);
        cv::eigen2cv(camera.tvec, cv2_tvec);
        cv::eigen2cv(camera.ncm1, cv2_ncm1);
        Eigen::Vector<int16_t, 2> w_2d_coord_from;
        Eigen::Vector<int16_t, 2> w_2d_coord_to;

        cv::projectPoints(objectPoints, cv2_rvec, cv2_tvec, cv2_ncm1, cv::Mat::zeros(1, 5, CV_64F), w_coord);
        w_2d_coord_from(0) = static_cast<int16_t>(std::round(w_coord.at(0).x));
        w_2d_coord_from(1) = static_cast<int16_t>(std::round(w_coord.at(0).y));
        w_2d_coord_to(0) = static_cast<int16_t>(std::round(w_coord.at(1).x));
        w_2d_coord_to(1) = static_cast<int16_t>(std::round(w_coord.at(1).y));

        // 3D座標がカメラ手前にあるか否かのテスト　外部パラメータ行列を同次３次元座標に掛けると[2,:]がカメラ座標上のZ軸に対応するのでそれで判別
        Eigen::Matrix<double, 2, 3> points3d = vstack(w_coord_from.transpose(), w_coord_to.transpose());
        Eigen::Matrix<double, 4, 2> homogenous_points =
            hstack(points3d, Eigen::Vector<double, 2>::Ones(points3d.rows(), 1)).transpose();
        Eigen::VectorXd camera_coordin_z = (camera.extrmat * homogenous_points).row(2);
        if ((camera_coordin_z.array() < 0.0).any())
        {
            continue;
        }
        if (w_2d_coord.size() == 0)
        {
            w_2d_coord = hstack(w_2d_coord_from.transpose(), w_2d_coord_to.transpose());
        }
        else
        {
            w_2d_coord = vstack(w_2d_coord, hstack(w_2d_coord_from.transpose(), w_2d_coord_to.transpose()));
        }
    }
    return w_2d_coord;
}

/***
 * - 引数:
 *   - camera: カメラのインスタンス
 *   - boxes: bounding boxのの8隅の座標(n_cluster*8, 3)の行列
 *   - cluster2entity:  各クラスタの属性辞書
 *   - draw_bbox_3d: 3dBBoxを描画するか否か
 *   - target_entity: bounding boxを表示する属性
 * - 戻り値:
 *   - w_2d_coord: bounding boxの座標リスト（空リストで初期化）
 */
Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>
calc_bounding_box_around_entity(const Camera& camera, const Eigen::MatrixXd& boxes,
                                const std::map<int, NodeEntity>& cluster2entity, bool draw_bbox_3d,
                                const PyLogger& logger, NodeEntity target_entity)
{
    Eigen::Matrix<int16_t, Eigen::Dynamic, 2 * 8> w_2d_coord =
        Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(0, 2 * 8);
    if (!draw_bbox_3d)
    {
        // bbox_3d を描画しない設定では、空行列を返して終了.
        return w_2d_coord;
    }
    if (boxes.rows() == 0)
    {

        return w_2d_coord;
    }
    Eigen::MatrixXd boxes_2d_d;
    Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic> boxes_2d;
    std::vector<cv::Point2d> cv2_boxes_2d;
    cv::Mat cv2_boxes;
    cv::Mat cv2_rvec;
    cv::Mat cv2_tvec;
    cv::Mat cv2_ncm1;
    cv::eigen2cv(boxes, cv2_boxes);
    cv::eigen2cv(camera.rvec, cv2_rvec);
    cv::eigen2cv(camera.tvec, cv2_tvec);
    cv::eigen2cv(camera.ncm1, cv2_ncm1);

    try
    {
        // 一旦全部のboxを2dに射影する
        cv::projectPoints(cv2_boxes, cv2_rvec, cv2_tvec, cv2_ncm1, cv::Mat::zeros(1, 5, CV_64F), cv2_boxes_2d);
        boxes_2d_d.resize(cv2_boxes_2d.size(), 2);
        for (size_t i = 0; i < cv2_boxes_2d.size(); ++i)
        {
            boxes_2d_d(i, 0) = cv2_boxes_2d[i].x;
            boxes_2d_d(i, 1) = cv2_boxes_2d[i].y;
        }
        // cv::cv2eigen(cv2_boxes_2d, boxes_2d);
        // NOTE
        // round()はEigenにもあるが、rint()の方がNumpyの実装に近いためrintを採用
        boxes_2d = boxes_2d_d.array().rint().cast<int16_t>();
    }
    catch (const std::exception& e)
    {
        logger.warning("UI_IF %s", e.what());
    }
    // 3D座標がカメラ手前にあるか否かのテスト　外部パラメータ行列を同次３次元座標に掛けると[2,:]がカメラ座標上のZ軸に対応するのでそれで判別
    Eigen::MatrixXd homogeneous_points = hstack(boxes, Eigen::VectorXd::Ones(boxes.rows(), 1)).transpose();
    Eigen::VectorXd camera_coordin_z = (camera.extrmat * homogeneous_points.cast<double>()).row(2);

    assert(camera_coordin_z.rows() / 8 == (int(cluster2entity.rbegin()->first) + 1));

    // target_entityに該当するcluster_idxを取り出す
    for (const auto& [cluster_idx, elem_entity] : cluster2entity)
    {
        if (elem_entity != target_entity)
        {
            continue;
        };
        Eigen::VectorXd idx_camera_coordin_z = camera_coordin_z.segment(cluster_idx * 8, 8);
        if ((idx_camera_coordin_z.array() < 0).any())
        {
            continue;
        }
        // NOTE ravel

        Eigen::Matrix<int16_t, Eigen::Dynamic, 1> idx_boxes_2d(8 * boxes_2d.cols());
        for (int i = 0; i < 8 * boxes_2d.cols(); ++i)
        {
            idx_boxes_2d(i) = boxes_2d(cluster_idx * 8 + (i / 2), i % 2);
        }
        w_2d_coord = vstack(w_2d_coord, idx_boxes_2d.transpose());
    }
    return w_2d_coord;
}