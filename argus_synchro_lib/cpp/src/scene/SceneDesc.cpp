#include <iostream>
#include <tuple>
#include <string>
#include <map>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>

#include "dataclass/camera.h"
#include "octotree/OctoTree.h"
#include "octotree/NodeEntity.h"
#include "scene/SceneDesc.h"
#include "scene/app_config.h"
#include "scene/common.h"
#include "logger/py_logger.h"

std::tuple<double, double, double, double> calc_iou(double ax_min, double ay_min, double ax_max, double ay_max,
                                                    double bx_min, double by_min, double bx_max, double by_max)
{
    /*
    aとbのiouを計算する関数
    */

    double a_area = (ax_max - ax_min) * (ay_max - ay_min);
    double b_area = (bx_max - bx_min) * (by_max - by_min);

    if (a_area == 0 && b_area == 0)
    {
        return std::make_tuple(0.0f, 0.0f, 0.0f, 0.0f);
    }

    double abx_min = std::max(ax_min, bx_min);
    double aby_min = std::max(ay_min, by_min);
    double abx_max = std::min(ax_max, bx_max);
    double aby_max = std::min(ay_max, by_max);

    double intersect = std::max(0.0, abx_max - abx_min) * std::max(0.0, aby_max - aby_min);
    double iou = intersect / (a_area + b_area - intersect);

    return std::make_tuple(iou, intersect, a_area, b_area);
}

Scene::Scene(const SceneDescriptionConf& scene_conf, const LoggerFunc logfunc)
    : coarse_lo(scene_conf.coarse_lo), coarse_hi(scene_conf.coarse_hi), k_min(scene_conf.k_min),
      h_ref_px(scene_conf.h_ref_px), lo_gain(scene_conf.lo_gain), hi_gain(scene_conf.hi_gain),
      lo_floor(scene_conf.lo_floor), hi_ceil(scene_conf.hi_ceil), vertical_w_iou(scene_conf.vertical_w_iou),
      vertical_w_scale(scene_conf.vertical_w_scale), vertical_w_phi(scene_conf.vertical_w_phi),
      final_threshold(scene_conf.final_threshold), use_human_gate(scene_conf.use_human_gate), H_min(scene_conf.H_min),
      H_max(scene_conf.H_max), W_min(scene_conf.W_min), W_max(scene_conf.W_max), D_min(scene_conf.D_min),
      D_max(scene_conf.D_max), tall_ratio_min(scene_conf.tall_ratio_min), logger_(PyLogger(logfunc))
{
}

void Scene::update(const SceneDescriptionConf& scene_conf, const LoggerFunc logfunc)
{
    coarse_lo = scene_conf.coarse_lo;
    coarse_hi = scene_conf.coarse_hi;
    k_min = scene_conf.k_min;
    h_ref_px = scene_conf.h_ref_px;
    lo_gain = scene_conf.lo_gain;
    hi_gain = scene_conf.hi_gain;
    lo_floor = scene_conf.lo_floor;
    hi_ceil = scene_conf.hi_ceil;
    vertical_w_iou = scene_conf.vertical_w_iou;
    vertical_w_scale = scene_conf.vertical_w_scale;
    vertical_w_phi = scene_conf.vertical_w_phi;
    final_threshold = scene_conf.final_threshold;
    use_human_gate = scene_conf.use_human_gate;
    H_min = scene_conf.H_min;
    H_max = scene_conf.H_max;
    W_min = scene_conf.W_min;
    W_max = scene_conf.W_max;
    D_min = scene_conf.D_min;
    D_max = scene_conf.D_max;
    tall_ratio_min = scene_conf.tall_ratio_min;
    this->logger_.setLogger(logfunc);
}

// -----------------------------------------------------------
// 共通補助指標：縦方向の一致度を評価（俯瞰配置で前後の距離一致を重く評価する）
//   - 3D投影BBox(8点)の縦範囲と、YOLO人BBoxの縦範囲を比較
//   - ① 縦IoU（1D）… 前後の重なり度合い（大きいほど良い）
//   - ② 縦スケール整合 … 縦の大きさ比 |h3d/h2d - 1|（小さいほど良い）
//   - ③ 縦ベクトルの向き差 … 姿勢の見え方の差（rad、小さいほど良い）
// 戻り値: (vIoU, height_ratio, phi)
// ------------------------------------------------------------
std::tuple<double, double, double>
Scene::vertical_consistency_scores(const Eigen::Vector4f& box2d, int width, int height,
                                   const Eigen::Matrix<float, 8, 2>& proj8 // shape (8,2) の1クラスタ分
)
{
    // YOLO人BBox（正規化→px）
    double v1 = box2d(0) * height;
    double v2 = box2d(2) * height;
    double h2d = std::max(1.0, v2 - v1);

    // 3D投影BBox（8点）から2D縦範囲・縦方向ベクトルを作る
    Eigen::VectorXf ys = proj8.col(1);
    double vmin3d = ys.minCoeff();
    double vmax3d = ys.maxCoeff();
    double h3d = std::max(1.0, vmax3d - vmin3d);

    // 1D-vertical IoU（縦方向の重なり）
    double inter = std::max(0.0, std::min(v2, vmax3d) - std::max(v1, vmin3d));
    double den = (v2 - v1) + (vmax3d - vmin3d) - inter;
    double vIoU = (den > 0) ? (inter / den) : 0.0;

    // 縦スケール整合（1に近いほど良い）
    double height_ratio = std::fabs((h3d / h2d) - 1.0);

    // 縦ベクトルの向き差：
    //   3D投影BBoxの「上面中心→下面中心」を2Dベクトルに
    Eigen::RowVector2f top_c2d = proj8.topRows<4>().colwise().mean();
    Eigen::RowVector2f bot_c2d = proj8.bottomRows<4>().colwise().mean();
    Eigen::RowVector2f v2d_j = bot_c2d - top_c2d;
    double phi;
    if (v2d_j.norm() < 1e-6f)
    {
        phi = static_cast<double>(M_PI) / 2.0f; //ほぼ無効な場合は90度相当として扱う
    }
    else
    {
        // 画像の“鉛直”は (0, +h2d) とみなす（ロールがあるなら重力の画像投影に差し替え可）
        Eigen::Vector2f v2d_h(0.0f, h2d);
        Eigen::Vector2f a = v2d_j / v2d_j.norm();
        Eigen::Vector2f b = v2d_h / v2d_h.norm();
        double cosang = std::clamp(a.dot(b), -1.0f, 1.0f);
        phi = std::acos(cosang); //[rad] 小さいほど縦方向の見え方が近い
    }

    return std::make_tuple(vIoU, height_ratio, phi); //: contentReference[oaicite:1]{index=1}
}

// ------------------------------------------------------------
// 共通ヘルパ：方式固有の base コストに“縦の一致”ペナルティを加える
//   cost = base + w1*(1-vIoU) + w2*height_ratio + w3*phi
// ------------------------------------------------------------
double Scene::augment_cost_with_verticals(double base_cost, double vIoU, double height_ratio, double phi_rad,
                                          std::optional<double> w1, std::optional<double> w2, std::optional<double> w3)
{
    w1 = vertical_w_iou;
    w2 = vertical_w_scale;
    w3 = vertical_w_phi;
    double cost = base_cost + w1.value() * (1.0 - vIoU) + w2.value() * height_ratio + w3.value() * phi_rad;
    return cost;
}

// ------------------------------------------------------------
// 共通ヘルパ：軽量プリゲート（候補の早刈り）
//   - h3d/h2d が極端にズレるものはそもそも評価しない
//   - 遠方・小BBOXでは緩める（画像高さpxに依存して自動調整）
// ------------------------------------------------------------
bool Scene::passes_coarse_height_gate(const Eigen::Vector4f& box2d, int width, int height,
                                      const Eigen::Matrix<float, 8, 2>& proj8, std::optional<double> lo,
                                      std::optional<double> hi)
{
    // 2D人BBox高さ(px)
    double v1 = box2d(0) * height;
    double v2 = box2d(2) * height;
    double h2d = std::max(1.0, v2 - v1);

    // 3D投影BBoxの縦サイズ(px)
    Eigen::VectorXf ys = proj8.col(1);
    double h3d = std::max(1.0f, ys.maxCoeff() - ys.minCoeff());
    double r = h3d / h2d;

    // [SceneDesc] から取得（無ければ既定値）
    double lo_val = lo.value_or(coarse_lo);
    double hi_val = hi.value_or(coarse_hi);

    //遠方・小BBoxで厳しすぎないよう緩和係数を適用（k∈[k_min,1]）
    double k = std::max(k_min, std::min(1.0, h2d / h_ref_px)); //小さい箱ほど k が小さい
    double lo_eff = std::max(lo_floor, lo_val * k * lo_gain);
    double hi_eff = std::min(hi_ceil, hi_val / k * hi_gain);

    return (lo_eff <= r && r <= hi_eff); //: contentReference[oaicite:3]{index=3}
}

// ------------------------------------------------------------
// 共通ヘルパ：本ゲート（誤対応の強制抑止）※関数は残置（呼出しは無効化可能）
//   - 縦IoU / 縦スケール整合 / 縦向き
// ------------------------------------------------------------
bool Scene::passes_vertical_hard_gate(double vIoU, double height_ratio, double phi_rad,
                                      const PassesVerticalHardGateOptions& opts)
{
    // 必要なら「小さい箱に優しい」適応も利用可（現状は呼び出し側で未使用）
    double vIoU_min = opts.vIoU_min;
    double height_ratio_max = opts.height_ratio_max;
    double phi_max_deg = opts.phi_max_deg;

    if (opts.box_h_px.has_value())
    {

        double k = std::max(0.3, std::min(1.0, opts.box_h_px.value() / 80.0));
        vIoU_min = std::max(0.05, vIoU_min * k);
        height_ratio_max = std::min(1.50, height_ratio_max / k);
        phi_max_deg = std::min(40.0, phi_max_deg + (1.0 - k) * 32.0);
    }

    return ((vIoU >= vIoU_min) && (height_ratio <= height_ratio_max) &&
            (phi_rad <= phi_max_deg * M_PI / 180.0)); //: contentReference[oaicite:4]{index=4}
}

int Scene::get_human_3bb(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds, int num_3d,
                         std::string method)
{
    /*
    box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
    width:画像の幅
    height:画像の高さ
    box3ds:立体物数*[x, y]*8点が1列に並んでいる
    cat3ds:立体物のクラス、0:人, それ以外も判別はされている
    num_3d:3dbbの数
    返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
    */
    if (method == "center")
    {
        return correspondence_by_center(box2d, width, height, box3ds, num_3d);
    }
    if (method == "iou")
    {
        return correspondence_by_iou(box2d, width, height, box3ds, num_3d);
    }
    if (method == "endpoints")
    {
        return correspondence_by_endpoints(box2d, width, height, box3ds, num_3d);
    }

    std::string msg = "method should be 'center' or 'iou', current method = " + method;
    throw std::invalid_argument(msg); // :contentReference[oaicite:5]{index=5}
}

// ------------------------------------------------------------
// 手法１：端点距離
// ------------------------------------------------------------
int Scene::correspondence_by_endpoints(const Eigen::Vector4f& box2d, int width, int height,
                                       const Eigen::MatrixXd& box3ds, int num_3d)
{
    /*
    2dと3dのbounding boxの端点の近さで選ぶ
    */
    // bboxが無い場合はそのままリターン
    if (num_3d == 0 || box3ds.rows() == 0)
    {
        return -1;
    }

    Eigen::Vector2d box2d_min(box2d(1) * width, box2d(0) * height);
    Eigen::Vector2d box2d_max(box2d(3) * width, box2d(2) * height);
    int best_idx = -1;
    double best_cost = 1e9;

    for (int j = 0; j < num_3d; j++)
    {
        Eigen::Matrix<double, 4, 2> box_3d_in_2d;
        for (int k = 0; k < 4; k++)
        {
            box_3d_in_2d(k, 0) = box3ds(j * 8 + 4 + k, 0);
            box_3d_in_2d(k, 1) = box3ds(j * 8 + 4 + k, 1);
        }

        Eigen::Vector2d box3d_min = box_3d_in_2d.colwise().minCoeff();
        Eigen::Vector2d box3d_max = box_3d_in_2d.colwise().maxCoeff();

        double base = 0.5 * ((box3d_min - box2d_min).norm() + (box3d_max - box2d_max).norm());

        // 縦整合のペナルティを加点
        Eigen::Matrix<float, 8, 2> proj8;
        for (int k = 0; k < 8; ++k)
        {
            proj8(k, 0) = box3ds(j * 8 + k, 0);
            proj8(k, 1) = box3ds(j * 8 + k, 1);
        }

        auto [vIoU, hratio, phi] = vertical_consistency_scores(box2d, width, height, proj8);

        double cost = augment_cost_with_verticals(base, vIoU, hratio, phi);

        if (cost < best_cost)
        {
            best_cost = cost;
            best_idx = j;
        }
    }

    return best_idx;
}

// ------------------------------------------------------------
// 手法２：IoUベース
// ------------------------------------------------------------
int Scene::correspondence_by_iou(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds,
                                 int num_3d)
{
    /*
    2D IoU 最大のものを選ぶ方式。
    俯瞰配置での前後取り違え/近距離誤吸着を抑制するため、
    - 粗い縦スケールゲート
    - （必要なら）縦の強制ゲート
    - 上記をペナルティ加点して総合コスト化
    */ // noqa: RUF002
    if (num_3d == 0 || box3ds.rows() == 0)
    {
        return -1;
    }
    int best_idx = -1;
    double best_cost = 1e9;
    // 人BBox（px）
    double box2d_xy[4] = {box2d(1) * width, box2d(0) * height, box2d(3) * width, box2d(2) * height};

    for (int j = 0; j < num_3d; j++)
    {
        Eigen::Matrix<float, 8, 2> proj8;
        for (int k = 0; k < 8; k++)
        {
            proj8(k, 0) = static_cast<float>(box3ds(j * 8 + k, 0));
            proj8(k, 1) = static_cast<float>(box3ds(j * 8 + k, 1));
        }

        //早刈り
        if (!passes_coarse_height_gate(box2d, width, height, proj8))
        {
            continue;
        }

        auto [vIoU, hratio, phi] = vertical_consistency_scores(box2d, width, height, proj8);

        // （必要なら）強制ゲートを戻せる（デフォルトは使わない）
        // box_h_px = float((box2d[2] - box2d[0]) * height)
        // if not self.passes_vertical_hard_gate(vIoU, hratio, phi, box_h_px=box_h_px):
        //     continue

        // 3D投影BBoxの2D矩形（下面4点で安定）
        Eigen::Matrix<float, 4, 2> bottom = proj8.block<4, 2>(4, 0);
        Eigen::Vector2f min_pos = bottom.colwise().minCoeff();
        Eigen::Vector2f max_pos = bottom.colwise().maxCoeff();

        Eigen::Vector4f box3d_xy;
        box3d_xy << min_pos(0), min_pos(1), max_pos(0), max_pos(1);

        auto [iou, _, __, ___] = calc_iou(box2d_xy[0], box2d_xy[1], box2d_xy[2], box2d_xy[3], box3d_xy[0], box3d_xy[1],
                                          box3d_xy[2], box3d_xy[3]);
        double base = 1.0f - iou;

        double cost = augment_cost_with_verticals(base, vIoU, hratio, phi);

        // 任意：最終保険ゲート（設定にあれば有効）
        if (cost >= final_threshold)
        {
            continue;
        }

        if (cost < best_cost)
        {
            best_cost = cost;
            best_idx = j;
        }
    }

    return best_idx; // :contentReference[oaicite:6]{index=6}
}

// ------------------------------------------------------------
// 手法３：重心（下辺中心）ベース
// ------------------------------------------------------------
int Scene::correspondence_by_center(const Eigen::Vector4f& box2d, int width, int height, const Eigen::MatrixXd& box3ds,
                                    int num_3d)
{
    // 重心（下辺中心）距離ベースで2Dと3Dを対応付け。
    // 俯瞰配置での前後取り違え/近距離誤吸着を抑えるため、
    // - 粗い縦スケールゲート
    // - （必要なら）縦の強制ゲート
    // - 上記をペナルティ加点して総合コスト化
    if (num_3d == 0 || box3ds.rows() < num_3d * 8 || box3ds.cols() < 2)
    {
        return -1;
    }

    int best_idx = -1;
    double best_cost = 1e9;

    // 人BBoxの下辺中心（接地点相当, px）
    double bottom_x_2d = (box2d(3) + box2d(1)) * width * 0.5;
    double bottom_y_2d = box2d(2) * height;
    Eigen::Vector2f bottom_2d(bottom_x_2d, bottom_y_2d);

    for (int j = 0; j < num_3d; ++j)
    {
        Eigen::Matrix<float, 8, 2> proj8;
        for (int k = 0; k < 8; ++k)
        {
            proj8(k, 0) = static_cast<float>(box3ds(j * 8 + k, 0));
            proj8(k, 1) = static_cast<float>(box3ds(j * 8 + k, 1));
        }

        // 早刈り
        if (!passes_coarse_height_gate(box2d, width, height, proj8))
        {
            continue;
        }

        // 縦方向のサイズで前チェック。厳しすぎる場合はコメントアウトでもいい
        auto [vIoU, hratio, phi] = vertical_consistency_scores(box2d, width, height, proj8);

        // box_h_px = float((box2d[2] - box2d[0]) * height)
        // if not Scene._passes_vertical_hard_gate(vIoU, hratio, phi, box_h_px=box_h_px):
        //     continue

        // 3D側の下面中心（px）：median
        std::vector<float> xs, ys;
        for (int k = 4; k < 8; ++k)
        {
            xs.push_back(proj8(k, 0));
            ys.push_back(proj8(k, 1));
        }
        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        Eigen::Vector2f bottom_c2d((xs[1] + xs[2]) * 0.5f, (ys[1] + ys[2]) * 0.5f);
        // baseコスト：下辺中心と下面中心の距離（px）
        double base = (bottom_2d - bottom_c2d).norm();
        double cost = augment_cost_with_verticals(base, vIoU, hratio, phi);

        // 任意：最終保険ゲート（必要なら有効化）
        // final_th = self.final_threshold
        // if cost >= final_th:
        //     continue

        if (cost < best_cost)
        {
            best_cost = cost;
            best_idx = j;
        }
    }

    return best_idx; // :contentReference[oaicite:7]{index=7}
}

bool Scene::passes_human_size(const Eigen::VectorXf& minmax_tuple)
{
    // minmax: (x_min, x_max, y_min, y_max, z_min, z_max)
    try
    {
        float x_min = minmax_tuple[0];
        float x_max = minmax_tuple[1];
        float y_min = minmax_tuple[2];
        float y_max = minmax_tuple[3];
        float z_min = minmax_tuple[4];
        float z_max = minmax_tuple[5];

        double H = static_cast<double>(z_max - z_min); // 身長方向
        double W = static_cast<double>(x_max - x_min);
        double D = static_cast<double>(y_max - y_min);
        // 現場向けに広めの初期値（必要に応じて調整）
        if (!(H_min <= H && H <= H_max))
        {
            return false;
        }

        if (!(W_min <= W && W <= W_max) || !(D_min <= D && D <= D_max))
        {
            return false;
        }

        // 縦長比（人らしさ）
        if ((H / std::max({W, D, 1e-6})) < tall_ratio_min)
        {
            return false;
        }

        return true;
    }
    catch (...)
    {
        return true; // 情報が無いときは通す（必要ならログ）
    }
}

Scene::t_cluster2entity Scene::integrate2d3d(const Camera& camera, const CameraDetectionData& bb_box_data,
                                             const Eigen::MatrixXd& boxes, const Eigen::MatrixXf& minmax,
                                             const int& valid_detects, const NodeEntity& from_entity,
                                             const std::string& method)
{
    // 立体物と人検知の紐づけ処理
    // dm = camera.dm
    int width = camera.width;
    int height = camera.height;
    Eigen::MatrixXd ncm1 = camera.ncm1;
    Eigen::MatrixXd rvec = camera.rvec;
    Eigen::MatrixXd tvec = camera.tvec;

    cv::Mat cv_ncm1(ncm1.rows(), ncm1.cols(), CV_64F);
    Eigen::Map<Eigen::Matrix<double, -1, -1, 1>> ncm1_map(cv_ncm1.ptr<double>(), ncm1.rows(), ncm1.cols());
    ncm1_map = ncm1;
    cv::Mat cv_rvec(rvec.rows(), rvec.cols(), CV_64F);
    Eigen::Map<Eigen::Matrix<double, -1, -1, 1>> rvec_map(cv_rvec.ptr<double>(), rvec.rows(), rvec.cols());
    rvec_map = rvec;
    cv::Mat cv_tvec(tvec.rows(), tvec.cols(), CV_64F);
    Eigen::Map<Eigen::Matrix<double, -1, -1, 1>> tvec_map(cv_tvec.ptr<double>(), tvec.rows(), tvec.cols());
    tvec_map = tvec;

    cv::Mat box3ds(boxes.rows(), boxes.cols(), CV_64F);
    Eigen::Map<Eigen::Matrix<double, -1, -1, 1>> box3ds_map(box3ds.ptr<double>(), boxes.rows(), boxes.cols());
    box3ds_map = boxes; // 3次元bb座標, 8点*3*(valid_detects[0]個)が入っている
    // n_clusters = int(max(LS_pcd_det.labels))

    int n_clusters = valid_detects;

    t_cluster2entity camera_cluster2entity; // カメラ単位の属性リスト
    for (int i = 0; i < n_clusters; i++)
    {
        camera_cluster2entity[i] = from_entity; //初期値はすべてfrom_entity
    }

    if (n_clusters != 0)
    {

        std::vector<cv::Point2d> box3ds_vector;
        cv::projectPoints(box3ds, cv_rvec, cv_tvec, cv_ncm1, cv::Mat::zeros(1, 5, CV_64F),
                          box3ds_vector); // 2次元座標に変換
        // 形状として(1,N,3)を想定。この形が崩れると後の処理で意図しない動作をする可能性がある
        // (reshape時に順序が崩れる、次元が合わない等)
        assert(boxes.rows() == 1);
        Eigen::MatrixXd box3ds_reproj(box3ds_vector.size(), 2);
        for (size_t i = 0; i < box3ds_vector.size(); ++i)
        {
            box3ds_reproj(i, 0) = box3ds_vector[i].x;
            box3ds_reproj(i, 1) = box3ds_vector[i].y;
        }

        Eigen::MatrixXd homogeneous_points(boxes.rows(), 4);
        homogeneous_points << boxes, Eigen::VectorXd::Ones(boxes.rows());
        Eigen::MatrixXd camera_coordinate_pts = camera.extrmat * homogeneous_points.transpose();
        Eigen::VectorXd camera_coordin_z = camera_coordinate_pts.row(2);

        const int BBOX_VERTEX_POINTS = 8;

        // box3ds_reproj:
        // bbox8点分ずつ格納。前からn_clusters*8点分のみ有効（n_clusters*8以降は不定？）対応するz座標を8個ずつ見て1つでも<0なら8点全て除去する必要がある
        // 本当に除去してしまうとintegrated_retults_2d3d反映時のインデックスと整合が取れなくなるので-1e6に飛ばすことで対応。
        Eigen::MatrixXd camera_coordin_z_bboxset =
            camera_coordin_z.reshaped<Eigen::RowMajor>(boxes.rows() / BBOX_VERTEX_POINTS, BBOX_VERTEX_POINTS);
        Eigen::Array<bool, -1, 1> camera_coordin_z_bboxset_flag =
            (camera_coordin_z_bboxset.array() > 0).rowwise().all();
        Eigen::Array<bool, -1, 2> box3ds_zfilter =
            camera_coordin_z_bboxset_flag.replicate(1, BBOX_VERTEX_POINTS * 2)
                .reshaped<Eigen::RowMajor>(camera_coordin_z_bboxset.rows() * BBOX_VERTEX_POINTS, 2);

        box3ds_reproj = box3ds_zfilter.cast<double>().select(box3ds_reproj.array(), -1e6);

        this->logger_.info("box3ds shape: (%d, %d), valid_bbox_count: %d, camera_coordin_z_bboxset_flag: %d, "
                           "box3ds_zfilter shape: (%ld, %ld), box3ds_reproj shape: (%ld, %ld)",
                           box3ds.rows, box3ds.cols, n_clusters,
                           (camera_coordin_z_bboxset_flag.array() != false).count(), box3ds_zfilter.rows(),
                           box3ds_zfilter.cols(), box3ds_reproj.rows(), box3ds_reproj.cols());

        // 改良版：人らしさの寸法ゲートを追加  # noqa: RUF003
        for (int i = 0; i < bb_box_data.valid_detects; i++)
        { // 2dbbを順番にチェック
            if (bb_box_data.classes(0, i) == 0)
            { // 人である場合
                Eigen::VectorXf box2d = bb_box_data.boxes.row(i);
                int index = Scene::get_human_3bb(box2d, width, height, box3ds_reproj, n_clusters, method);
                if (index >= 0)
                {
                    // ---- 寸法ゲート（人らしさ）で最終確認：外れたら HUMAN を取り消す ----
                    // 人寸法ゲートは使わない設定の場合は、そのまま追加する。
                    const bool pass_human_gate = !use_human_gate || Scene::passes_human_size(minmax.row(index));
                    if (pass_human_gate)
                    {
                        camera_cluster2entity[index] = NodeEntity::HUMAN;
                    }
                    else if (use_human_gate)
                    {
                        // 人寸法を外れるので、元の属性のまま（誤通知抑止）
                        // AppLogger.debug(f"Rejected HUMAN by size gate: idx={index}")
                    }
                }
            }
        }
    }
    return camera_cluster2entity;
}

std::tuple<OctoTree, Scene::t_cluster2entity>
Scene::aggregate2d3d_results(const std::vector<Scene::t_cluster2entity>& camera_cluster2entities,
                             const OctoTree& octotree_obj, const NodeEntity& from_entity)
{
    OctoTree octoTree = octotree_obj;
    t_cluster2entity integrated_results_2d3d;

    for (const auto& camera_cluster2entity : camera_cluster2entities)
    {
        for (const auto& [cluster_id, entity] : camera_cluster2entity)
        {
            if (entity == NodeEntity::HUMAN)
            {
                integrated_results_2d3d[cluster_id] = NodeEntity::HUMAN;
                continue;
            }
            if (integrated_results_2d3d.find(cluster_id) == integrated_results_2d3d.end())
            {
                integrated_results_2d3d[cluster_id] = from_entity;
            }
        }
    }

    octoTree.replace_entities_in_octonodes(from_entity, integrated_results_2d3d);
    return std::make_tuple(octoTree, integrated_results_2d3d);
}

t_py_col_res Scene::append_distance_info(const t_py_col_res& collision_clusters, const Eigen::MatrixXf& minmax,
                                         const Eigen::Vector3d& origin)
{
    /*衝突判定の結果にクラスタの重心からoriginまでの距離の情報を追加する
    距離は末尾に追加する
    */
    t_py_col_res _collision_clusters;

    for (const auto& [label, collision_info] : collision_clusters)
    {

        if (!label.has_value())
        {
            //クラスタリングと関連するcollision_cluster以外ははじく
            continue;
        }

        // Note1: LS_pcd_det.boxesからも最小, 最大は取れるが、
        // クラスタ毎にn個間隔でデータが入っているデータを触るのは怖いので、
        // クラスタ番号毎に最小・最大が入っているフィールドから情報を取り出す
        //
        // Note2: LS_pcd_det.minmaxはクラスタ番号0から始まる(クラスタ番号-1は除外されている)ので、
        // クラスタ番号=labelのminmaxを取り出したい場合は、minmax[label]で問題ないが、
        // minmaxの行番号の持つ意味が変わった場合、影響を受ける
        int cluster_id = label.value();
        float x_min = minmax(cluster_id, 0);
        float x_max = minmax(cluster_id, 1);
        float y_min = minmax(cluster_id, 2);
        float y_max = minmax(cluster_id, 3);

        Eigen::Vector3d cluster_med((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 0.0);
        double distance = (origin - cluster_med).norm();

        const auto& [node1, node2, pos1, pos2, val1, _] = collision_info;

        _collision_clusters.emplace(label, std::make_tuple(node1, node2, pos1, pos2, val1, distance));
    }
    return _collision_clusters;
}
