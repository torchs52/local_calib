#pragma once

#include "Point.h"

#include <optional>

/** 一つのボクセルに対する統計量が入った構造体
Rustで実装していた部分で、Rust実装 -> C -> Pythonという流れで呼び出そうとしたものがそのままになっている
まず初期化を行い、同じ格子に追加の点が入ってくるたびに、insert_pointsを呼んで各統計量を更新するような使われ方をしている
 */
class VoxStats
{
  public:
    Point first_moment;  // ボクセル内の平均
    Point second_moment; // ボクセル内の標準偏差
    int counts;          // ボクセル内の点群数
    Point far_point;     // 原点から一番離れている点群の座標
    Point near_point;    // 原点から一番近い点群の座標
    Point quantile;
    float far_dist;  // optionが使えないので、-1をNoneとして扱う
    float near_dist; // optionが使えないので、-1をNoneとして扱う
    VoxStats(Point first_moment, Point second_moment, int counts, Point far_point, Point near_point, Point quantile,
             float far_dist, float near_dist);
    VoxStats();

    /** 追加の点を基に統計量を更新するメソッド
     */
    void insert_points(Point points);

    /** 以下、getter関連のメソッド
     */
    Point get_mean() const;
    int get_counts() const;
    Point get_far_point() const;
    Point get_near_point() const;
    Point get_quantile() const;

    /** モーメントの更新を行うメソッド
     */
    static Point increment_point_statistics(int count, const Point& statistics, const Point& new_point);
};
