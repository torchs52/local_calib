#pragma once

#include "MachineConf.h"
#include <optional>

#include <Eigen/Core>

using Matrix3d = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
// Use Matrix3d instead of Matrix3d
using arrayXb = Eigen::Array<bool, Eigen::Dynamic, 1>;

class MachineCollisionBase
{
    /**
     * @brief 機体点群除去と衝突判定に関連する機体情報が入ったクラス
     *
     */
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 機体の点群を表す配列, 衝突判定で用いる
    Eigen::MatrixXd machine_pcd_points;
    // 機体除去の代表点, 機体除去で用いる,
    // オフセット済みなので、こちらは初回で変換しないようにする
    Eigen::MatrixXd machine_form_points;

    // 主にcol_machine_info.jsoncに入っている属性を持っているインスタンス
    MachineConf machine_info;

    // 点群ファイルの名前
    std::string pcd_points_file;

    // オフセットと座標反転の有無
    std::tuple<double, double, double> offsets_initial;
    std::tuple<bool, bool, bool> reverse_initial;

    /**
     * @brief Construct a new Machine Collision Base object
     *
     * @param machine_info col_machine_info.jsoncに入っている属性を持つインスタンス
     * @param filename 衝突判定に用いる機体点群のファイルパス
     * @param initial_offsets 衝突判定に用いる機体点群の初期オフセット
     * @param reverse 機体点群の各軸を反転させるかどうかを表すbool, trueの場合、軸を反転させる
     * @param machine_form_extension 読み込むファイルの拡張子,
     * csvファイルではなく、jsonでファイルを読み込むクラスも必要になったので追加
     */
    MachineCollisionBase(MachineConf machine_info, const std::string& filename,
                         const std::optional<std::tuple<double, double, double>>& initial_offsets = std::nullopt,
                         const std::optional<std::tuple<bool, bool, bool>>& reverse = std::nullopt,
                         const std::string& machine_form_extension = ".csv");

    virtual ~MachineCollisionBase() = default;

    /**
     * @brief 衝突判定に用いる機体点群をオフセットしたり、軸反転したりする
     *
     * @param points 衝突判定に用いる機体点群 (n, 3)行列
     * @param offsets 機体点群のオフセット (3,)ベクトル
     * @return Eigen::MatrixXd
     */
    Eigen::MatrixXd _move_and_adjust_crane_position(const Eigen::Ref<const Eigen::MatrixXd>& points,
                                                    const std::tuple<double, double, double>& offsets);

    /**
     * @brief 衝突判定に用いる機体点群の位置を初期化するメソッド
     *
     * @param common_offsets 機体点群のオフセット量
     * @return MachineCollisionBase&
     */
    MachineCollisionBase& _initialize_machine_status(const std::tuple<double, double, double>& common_offsets);

    /**
     * @brief xyzが該当する機体の部品と同じ位置に存在するかどうか判定する
     * @details クローラーや上部旋回体、カウンタウェイトによって除去方法が異なるため、抽象メソッドで扱う
     *
     * @param xyz lidar 点群
     * @param remove_dist 機体点群に対してどれだけの距離まで同じ位置とみなすか, 実座標を基に
     * @param roll_angle roll角
     * @param pitch_angle pitch角
     * @param yaw_angle yaw角
     * @param transform_mat 変換行列, nullの場合roll, pitch, yawによる座標変換を行う
     * @return arrayXb xyzと同じ長さのboolが入ったarrayで除去する箇所にtrueが入っている
     */
    virtual arrayXb
    check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                      const std::tuple<double, double, double>& remove_dist = std::make_tuple(0.11, 0.11, 0.11),
                      double roll_angle = 0, double pitch_angle = 0, double yaw_angle = 0,
                      const std::optional<Eigen::MatrixXd>& transform_mat = std::nullopt) const;

    /**
     * @brief デバッグ文字列を表示するメソッド, 機体点群ファイルが入ったファイルのパスを返す
     *
     * @return std::string
     */
    std::string to_string();
};
