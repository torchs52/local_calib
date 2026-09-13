#pragma once

struct SceneDescriptionConf
{
  public:
    double coarse_lo;
    double coarse_hi;
    double k_min;
    int h_ref_px;
    double lo_gain;
    double hi_gain;
    double lo_floor;
    double hi_ceil;
    double vertical_w_iou;
    double vertical_w_scale;
    double vertical_w_phi;
    double final_threshold;
    bool use_human_gate;
    double H_min;
    double H_max;
    double W_min;
    double W_max;
    double D_min;
    double D_max;
    double tall_ratio_min;
};
