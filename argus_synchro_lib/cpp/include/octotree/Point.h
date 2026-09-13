#pragma once

/** 八分木ノードの各格子で統計量を保持するために作った構造体
 Rustで実装していた部分で、Rust実装 -> C -> Pythonという流れで呼び出そうとしたものがそのままになっていて、
 RustにおけるOption型をCに変換できなかったりした結果、null管理を行うboolがあったりしている
 */
struct Point
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool is_null = true; // c++の変換でoption型が使えなかったので、苦肉の策でnull判定をするフラグをstructに追加
    Point(double x, double y, double z, bool is_null) : x(x), y(y), z(z), is_null(is_null)
    {
    }
    Point()
    {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
        this->is_null = true;
    };
};
