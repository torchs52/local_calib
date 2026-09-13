#include "octotree/MachineCollisionBase.h"
#include "octotree/machine_collision.h"

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
MachineCollisionBase::MachineCollisionBase(MachineConf machine_info, const std::string& filename,
                                           const std::optional<std::tuple<double, double, double>>& initial_offsets,
                                           const std::optional<std::tuple<bool, bool, bool>>& reverse,
                                           const std::string& machine_form_extension)
    : machine_info(machine_info)
{
    this->offsets_initial = (initial_offsets == std::nullopt) ? machine_info.offsets : initial_offsets.value();
    this->reverse_initial = (reverse == std::nullopt) ? machine_info.reverse : reverse.value();

    this->pcd_points_file = machine_info.pcd_points_file;
    this->machine_pcd_points = read_saved_points(filename);

    if (check_extension(machine_form_extension, ".csv"))
    {
        this->machine_form_points = read_saved_points(machine_info.get_form_points_filename(filename));
    }
    else if (check_extension(machine_form_extension, ".jsonc"))
    {
        // jsoncの場合は、個別にmachine_form_pointsを作るようにする
        // Todo: 構造の修正は要検討 =>
        // 結局個々の具象クラスのcheck_pcd_selfで使うので、具象クラスで自分のcheck_pcd_selfで必要になる変数を設定する方が自然な気がする
        this->machine_form_points = Eigen::MatrixXd();
    }
    else
    {
        throw std::logic_error("機体構造データの拡張子はcsvかjsoncだけ想定しています");
    }
    this->_initialize_machine_status(this->offsets_initial);
}

/**
 * @brief 衝突判定に用いる機体点群をオフセットしたり、軸反転したりする
 * @details 軸方向をずらした後で軸反転が必要な場合は反転する
 *
 * @param points 衝突判定に用いる機体点群 (n, 3)行列
 * @param offsets 機体点群のオフセット (3,)ベクトル
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MachineCollisionBase::_move_and_adjust_crane_position(const Eigen::Ref<const Eigen::MatrixXd>& points,
                                                                      const std::tuple<double, double, double>& offsets)
{
    Eigen::MatrixXd np_crane = points;

    np_crane(Eigen::all, 0) = np_crane(Eigen::all, 0).array() + std::get<0>(offsets);
    if (std::get<0>(this->reverse_initial))
    {
        np_crane(Eigen::all, 0) = -1.0 * np_crane(Eigen::all, 0);
    }
    np_crane(Eigen::all, 1) = std::get<1>(offsets) + np_crane(Eigen::all, 1).array();
    if (std::get<1>(this->reverse_initial))
    {
        np_crane(Eigen::all, 1) = -1.0 * np_crane(Eigen::all, 1);
    }

    np_crane(Eigen::all, 2) = std::get<2>(offsets) + np_crane(Eigen::all, 2).array();
    if (std::get<2>(this->reverse_initial))
    {
        np_crane(Eigen::all, 2) = -1.0 * np_crane(Eigen::all, 2);
    }

    return np_crane;
}

/**
 * @brief 衝突判定に用いる機体点群の位置を初期化するメソッド
 *
 * @param common_offsets
 * @return MachineCollisionBase&
 */
MachineCollisionBase&
MachineCollisionBase::_initialize_machine_status(const std::tuple<double, double, double>& common_offsets)
{
    std::tuple<float, float, float> offsets = {std::get<0>(common_offsets) + std::get<0>(this->machine_info.offsets),
                                               std::get<1>(common_offsets) + std::get<1>(this->machine_info.offsets),
                                               std::get<2>(common_offsets) + std::get<2>(this->machine_info.offsets)};

    this->machine_pcd_points = this->_move_and_adjust_crane_position(this->machine_pcd_points, offsets);
    return *this;
}

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
arrayXb MachineCollisionBase::check_pcd_on_self(const Eigen::Ref<const Eigen::MatrixXd>& xyz,
                                                const std::tuple<double, double, double>& remove_dist,
                                                double roll_angle, double pitch_angle, double yaw_angle,
                                                const std::optional<Eigen::MatrixXd>& transform_mat) const
{
    throw std::logic_error("継承してください");
};

/**
 * @brief デバッグ文字列を表示するメソッド, 機体点群ファイルが入ったファイルのパスを返す
 *
 * @return std::string
 */
std::string MachineCollisionBase::to_string()
{
    return this->pcd_points_file;
}
