#include "octotree/VoxStats.h"
#include "octotree/OctoTree.h"

#include <cmath>

VoxStats::VoxStats(Point first_moment, Point second_moment, int counts, Point far_point, Point near_point,
                   Point quantile, float far_dist, float near_dist)
    : first_moment(first_moment), second_moment(second_moment), counts(counts), far_point(far_point),
      near_point(near_point), quantile(quantile), far_dist(far_dist), near_dist(near_dist){};
VoxStats::VoxStats()
{
    this->first_moment = Point(0, 0, 0, false);
    this->second_moment = Point(0, 0, 0, false);
    this->counts = 0;
    this->far_point = Point(0, 0, 0, true);
    this->near_point = Point(0, 0, 0, true);
    this->quantile = Point(0, 0, 0, true);
    this->far_dist = -1;
    this->near_dist = -1;
}
// 追加の点を基に統計量を更新する
void VoxStats::insert_points(Point points)
{
    Point new_first_moment = points;
    Point new_second_moment = Point(std::pow(points.x, 2), std::pow(points.y, 2), std::pow(points.z, 2), false);
    float new_dist = std::sqrt(std::pow(points.x, 2) + std::pow(points.y, 2) + std::pow(points.z, 2));
    this->first_moment = VoxStats::increment_point_statistics(this->counts, this->first_moment, new_first_moment);
    this->second_moment = VoxStats::increment_point_statistics(this->counts, this->second_moment, new_second_moment);

    // far_distの更新, -1はNoneを表す
    if (this->far_point.is_null)
    {
        this->far_dist = new_dist;
        this->far_point = points;
    }
    else
    {
        if (this->far_dist < new_dist)
        {
            this->far_dist = new_dist;
            this->far_point = points;
        }
    }

    // near_distの更新, -1はNoneを表す
    if (this->near_point.is_null)
    {
        this->near_dist = new_dist;
        this->near_point = points;
    }
    else
    {
        if (this->near_dist > new_dist)
        {
            this->near_dist = new_dist;
            this->near_point = points;
        }
    }
    this->counts++;
}

Point VoxStats::get_mean() const
{
    return this->first_moment;
}

int VoxStats::get_counts() const
{
    return this->counts;
}
Point VoxStats::get_far_point() const
{
    return this->far_point;
}

Point VoxStats::get_near_point() const
{
    return this->near_point;
}

Point VoxStats::get_quantile() const
{
    return this->quantile;
}

/// モーメントの更新の計算式は一律なので、それを実施するための関数
Point VoxStats::increment_point_statistics(int count, const Point& statistics, const Point& new_point)
{
    return Point((float(count) * statistics.x + new_point.x) / (float(count) + 1.),
                 (float(count) * statistics.y + new_point.y) / (float(count) + 1.),
                 (float(count) * statistics.z + new_point.z) / (float(count) + 1.), false);
};
