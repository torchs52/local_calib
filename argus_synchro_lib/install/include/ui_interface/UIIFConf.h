#pragma once

#include <string>
#include <vector>

struct UIIFConf
{
  public:
    bool damp_out;
    int bbox_3d_num;
    double bbox_3d_distance;
    std::vector<std::string> UI_mmap;
    std::vector<std::string> damp_mmap;
    bool show_unk;
    double collision_depict_dist;
    double collision_attention_dist;
    double collision_warning_dist;
    double cliff_attention_dist;
    double cliff_warning_dist;
    bool draw_bbox_3d;
    bool draw_collision;

    UIIFConf(bool damp_out, int bbox_3d_num, double bbox_3d_distance, const std::vector<std::string>& UI_mmap,
             const std::vector<std::string>& damp_mmap, bool show_unk, double collision_depict_dist,
             double collision_attention_dist, double collision_warning_dist, double cliff_attention_dist,
             double cliff_warning_dist, bool draw_bbox_3d, bool draw_collision)
        : damp_out(damp_out), bbox_3d_num(bbox_3d_num), bbox_3d_distance(bbox_3d_distance), UI_mmap(UI_mmap),
          damp_mmap(damp_mmap), show_unk(show_unk), collision_depict_dist(collision_depict_dist),
          collision_attention_dist(collision_attention_dist), collision_warning_dist(collision_warning_dist),
          cliff_attention_dist(cliff_attention_dist), cliff_warning_dist(cliff_warning_dist),
          draw_bbox_3d(draw_bbox_3d), draw_collision(draw_collision)
    {
    }
};
