#pragma once

#include "octotree/OctoTree.h"
#include <tuple>

// 3次元座標(x,y,z)
using t_point_tuple = std::tuple<double, double, double>;

//衝突判定の結果としてScrutinizer上で扱う型, key側は良いが、value側はdataclassとかで扱ったほうが良いかも
using t_py_col_res =
    std::unordered_map<std::optional<int>,
                       std::tuple<OctoNode, OctoNode, t_point_tuple, t_point_tuple, double, std::optional<double>>>;
