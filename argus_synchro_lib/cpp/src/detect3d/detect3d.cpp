#include "logger/py_logger.h"
#include "detect3d/detect3d.h"
#include "detect3d/utils.h"
#include "dataclass/pcd.h"

#include <Eigen/Core>
#include <set>
#include <tuple>
#include <vector>
#include <string>

/* 定数 */
constexpr Eigen::Index MULTI_POINTS_TOTAL_SIZE = 8L * PcdDet::MAX_TOTAL_SIZE; // BoundingBox頂点数の合計
constexpr Eigen::Index MULTI_LINE_TOTAL_SIZE = 12L * PcdDet::MAX_TOTAL_SIZE;  // BoundingBox辺数の合計
constexpr Eigen::Index MULTI_MINMAX_TOTAL_SIZE = 1L * PcdDet::MAX_TOTAL_SIZE; // BoundingBox最大最小値の合計数

/* static関数宣言 */
template <typename Derived>
static inline Eigen::Vector<typename Derived::Scalar, Eigen::Dynamic>
EigenUnique(const Eigen::MatrixBase<Derived>& matrix);

// numpy.unique()相当の処理
template <typename Derived>
static Eigen::Vector<typename Derived::Scalar, Eigen::Dynamic> EigenUnique(const Eigen::MatrixBase<Derived>& matrix)
{
    using Scalar = typename Derived::Scalar;
    using Vector = Eigen::Vector<Scalar, Eigen::Dynamic>;

    // Std::vectorに詰めなおし(std::sort, std::uniqueを使用するため)
    auto mtx = matrix.derived();
    std::vector<Scalar> tmp_vec(mtx.data(), mtx.data() + mtx.size());

    // ソートと重複削除
    std::sort(tmp_vec.begin(), tmp_vec.end());
    auto last = std::unique(tmp_vec.begin(), tmp_vec.end());
    tmp_vec.erase(last, tmp_vec.end());

    // Eigen::Vectorに再度詰め替え
    Vector unq_vec = Eigen::Map<Vector>(tmp_vec.data(), tmp_vec.size());

    return unq_vec;
}

// NOTE: 引数debug_logは移植元でも使用されていない。
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi>
main_accum(const Eigen::Ref<const Eigen::MatrixXd>& xyz, const std::string& debug_log, double eps, int min_samples,
           const LoggerFunc logfunc)
{
    // # 地面検出用クラスのインスタンス化
    // # PW = ground_removal.patchwork()

    Eigen::MatrixXd boxes_write = Eigen::MatrixXd::Zero(MULTI_POINTS_TOTAL_SIZE, 3L);
    Eigen::MatrixXd lines_write = Eigen::MatrixXd::Zero(MULTI_LINE_TOTAL_SIZE, 2L);
    Eigen::MatrixXd minmax_write = Eigen::MatrixXd::Zero(MULTI_MINMAX_TOTAL_SIZE, 6L);
    Eigen::VectorXi valid_detects_write = Eigen::VectorXi::Zero(1L);
    Eigen::VectorXi labels_write = Eigen::VectorXi::Zero(1 * PcdData::SIZE);

    // NOTE: no_g_pcd_writeとno_g_pcd_num_writeは移植元でも出力されていないのでコメントアウトしている。
    // Eigen::MatrixXd no_g_pcd_write = Eigen::MatrixXd::Zero(
    //     static_cast<Eigen::Index>(PcdData::SIZE),
    //     static_cast<Eigen::Index>(PCD::CH));
    // Eigen::VectorXi no_g_pcd_num_write = Eigen::VectorXi::Zero(1L);

    // NOTE: 移植元の下記コメントの処理を1行にまとめている
    // # 点群数がPcdData.SIZEを超えたら、超えた分を捨てる
    // # 今はｚのしきい値で地面を分離しているので、何もせず点群をそのまま処理に回す
    Eigen::MatrixXd non_ground_pc = ResizeMatrix(xyz, static_cast<Eigen::Index>(PcdData::SIZE), 3L);

    auto labels = dbscan(non_ground_pc, eps, min_samples);

    // # DBSCAN結果からbounding boxを計算する
    auto [multi_points, multi_lines, multi_minmax] = bounding_box(non_ground_pc, EigenUnique(labels), labels);

    // NOTE: 移植元のコメントの意味が良く分からない。
    // # 共有データへの書き込み
    auto valid_detect_num = multi_minmax.rows();

    // # 立体物が最大値を超えたら、それ以上の立体物データは捨てる。
    if (valid_detect_num >= PcdDet::MAX_TOTAL_SIZE)
    {
        valid_detect_num = PcdDet::MAX_TOTAL_SIZE;
        multi_points = ResizeMatrix(multi_points, MULTI_POINTS_TOTAL_SIZE, NO_SLICE);
        multi_lines = ResizeMatrix(multi_lines, MULTI_LINE_TOTAL_SIZE, NO_SLICE);
        multi_minmax = ResizeMatrix(multi_minmax, MULTI_MINMAX_TOTAL_SIZE, NO_SLICE);
    }

    // NOTE: 移植前のPythonコードの時点でコメントアウトされていたログ出力処理
    // PyLogger logger(logfunc);
    // logger.info("valid_detect_num=%ld", valid_detect_num);
    // logger.info("multi_points=(%ld, %ld)", multi_points.rows(), multi_points.cols());
    // logger.info("multi_minmax=(%ld, %ld)", multi_minmax.rows(), multi_minmax.cols());
    // logger.info("no_g_pcd=(%ld, %ld)", no_g_pcd);

    // # この辺の微妙な書き込みタイミングのずれが問題になるかも。
    // # 変動しうるのは、縦方向のサイズのみだが、横方向も行列の大きさから判定.
    boxes_write.block(0, 0, multi_points.rows(), multi_points.cols()) = multi_points;
    lines_write.block(0, 0, multi_lines.rows(), multi_lines.cols()) = multi_lines;
    minmax_write.block(0, 0, multi_minmax.rows(), multi_minmax.cols()) = multi_minmax;
    valid_detects_write(0) = valid_detect_num;
    labels_write.block(0, 0, labels.rows(), labels.cols()) = labels;

    // NOTE: 以下の変数は移植元でも出力されていないのでコメントアウトしている。
    // no_g_pcd_write.block(0,0,non_ground_pc.rows(), non_ground_pc.cols()) = non_ground_pc;
    // no_g_pcd_num_write(0) = non_ground_pc.rows();

    // NOTE: 移植元にはこのタイミングでfloat64に変換して新たに詰めなおす処理が有るが、
    //       C++化に際しfloat64(double)で一貫して計算するようにしたためここでは処理なし

    std::tuple<Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>, Eigen::Ref<const Eigen::MatrixXd>,
               Eigen::Ref<const Eigen::VectorXi>, Eigen::Ref<const Eigen::VectorXi>>
        result = {boxes_write, lines_write, minmax_write, valid_detects_write, labels_write};

    return result;
}
