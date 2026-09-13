#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "OctoTree.h"
#include "OctoNode.h"

/**
 * @brief 崖検出のメインの処理を行うクラスなどを置いておくheader
 *
 */

namespace edge_det
{
struct EdgeDetectionResult
{
    /**
     * @brief 崖検出の結果を入れておくクラス
     * @details
     * 不変なクラスを意識して、コンストラクタ生成時からインスタンス変数をなるべく変更できないような作りにしている
     *
     */
    const int frame;                   // フレーム番号
    const double time;                 // タイムスタンプ
    const Eigen::MatrixXd edge_points; //崖のエッジ点
    const Eigen::MatrixXd edge_lines;  //崖のライン情報, godotでは要らないかも
    const Eigen::VectorXi edge_length; //崖の長さ, 崖のクラスタを取り出すのに必要

    /**
     * @brief コンストラクタ
     *
     * @param frame_  フレーム番号
     * @param time_ タイムスタンプ
     * @param edge_points_ 崖検出のエッジの点
     * @param edge_lines_ 崖検出のラインの点
     * @param edge_length_ 崖検出の各クラスタの長さ
     */
    EdgeDetectionResult(int frame_, double time_, Eigen::MatrixXd edge_points_, Eigen::MatrixXd edge_lines_,
                        Eigen::VectorXi edge_length_);

    /**
     * @brief edge_pointsは高さに対して短冊の始点との点の座標を持っていて[(x_1, y_1, z_1_lowest), (x_1, y_1,
     * z_1_hightest), ...]という形で座標を保持しているので、highest側を除く形で点を取得する
     *
     * @return Eigen::MatrixXd highestを除いたedge_points
     */
    Eigen::MatrixXd get_edge_points_on_ground() const;

    /**
     * @brief edge_points, edge_lengthから地面位置の点を取得して、その点が属するクラスタを紐づけて結果を返す
     *
     * @return std::pair<Eigen::MatrixXd, Eigen::VectorXi> (n,3)の地面点群と各点群が属するクラスタ番号である(n,)ベクトル
     */
    std::pair<Eigen::MatrixXd, Eigen::VectorXi> get_edge_cluster() const;
};

struct EdgeDetectionConfig
{
    /**
     * @brief 崖検出で用いる設定値のC++上での実装
     * @details PythonのAppConfigを使いたいが、C++上から見えないので、多重管理になるがC++側でも実装,
     * 他に必要な属性が出てくれば加える
     *
     */
    // 崖の総距離の閾値
    const float target_edge_dist_th;

    EdgeDetectionConfig(float target_edge_dist_th_ = 20.0);
};

class EdgeDetectorIF
{
    /**
     * @brief 崖検出のメイン処理を行うクラスのインターフェース
     * このクラスを継承して崖検出を行う想定
     *
     */
  public:
    virtual ~EdgeDetectorIF() = default;

    /**
     * @brief 崖検出のメイン処理のインターフェース
     *
     * @param octotree_obj 八分木インスタンス
     * @param target_entities 崖検出の対象となるNodeEntityのリスト,
     * クラスタ番号に依らず全てのtarget_entitiesを持つChunkOctoNodesに対して処理は行われる
     * @return edge_det::EdgeDetectionResult 崖検出のエッジなどが入った構造体
     */
    virtual edge_det::EdgeDetectionResult main(OctoTree& octotree_obj, std::vector<NodeEntity>& target_entities) = 0;

    /**
     * @brief パラメータの更新を行うメソッドのインターフェース
     * @details
     * Python側のAppConfigが必要だが、AppConfigがPython固有のクラスなので、崖検出専用のパラメータ経由でパラメータを更新できるようにする
     */
    virtual void update(EdgeDetectionConfig edge_conf) = 0;
};

class EdgeDetectorCpp : public EdgeDetectorIF
{
    /**
     * @brief EdgeDetectorIFを継承したクラス, 将来C++に変換する用のクラスのテンプレ
     *
     */
  public:
    /**
     * @brief Python側から受け取りたい属性はここで引数にしてメンバ変数に追加する
     *
     */
    EdgeDetectorCpp();

    EdgeDetectionResult main(OctoTree& octotree_obj, std::vector<NodeEntity>& target_entities) override;

    /**
     * @brief
     * Python側から更新したい変数をEdgeDetectionConfigに追加している前提で、edge_confインスタンス経由でこのインスタンスの変数を更新する
     *
     * @param edge_conf 更新させたい情報が入った構造体,
     */
    void update(EdgeDetectionConfig edge_conf) override;
};

} // namespace edge_det
